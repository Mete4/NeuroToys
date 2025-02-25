""" 
Detects blinks in EEG data and plots the filtered EEG data with detected spikes and blinks
"""
import numpy as np
import matplotlib
from scipy.signal import lfilter, lfilter_zi, firwin, butter, lfilter, welch, find_peaks
from time import sleep
from pylsl import StreamInlet, resolve_byprop
import seaborn as sns
from threading import Thread
from constants import VIEW_BUFFER, VIEW_SUBSAMPLE, LSL_SCAN_TIMEOUT, LSL_EEG_CHUNK
import time



def detect_eeg_spikes(data, threshold=200, min_distance=50, prominence=30):
    """Detect spikes in EEG data based on amplitude and prominence thresholds."""
    pos_peaks, pos_properties = find_peaks(data, 
                                         height=threshold,
                                         distance=min_distance,
                                         prominence=prominence)
    
    neg_peaks, neg_properties = find_peaks(-data,
                                         height=threshold,
                                         distance=min_distance,
                                         prominence=prominence)
    
    return pos_peaks, neg_peaks, pos_properties, neg_properties

def detect_blink_pattern(pos_peaks, neg_peaks, timestamps, max_interval=0.2):
    """
    Detect blink patterns: for each positive spike, if there is a subsequent negative spike within max_interval, add the timestamp of the positive spike
    Returns a list of blink timestamps
    """
    blink_times = []
    if len(pos_peaks) == 0 or len(neg_peaks) == 0:
        return blink_times
    for pos_peak in pos_peaks:
        # Find the first negative spike after this positive spike satisfying condition
        for neg_peak in neg_peaks:
            if neg_peak > pos_peak:
                interval = timestamps[neg_peak] - timestamps[pos_peak]
                if interval < max_interval:
                    blink_times.append(timestamps[pos_peak])
                    break  # Move to next positive peak once a pair is found
    return blink_times

def bandpass(sig, lowcut, highcut, fs, order=4):
    B, A = butter(order, [lowcut/(fs/2), highcut/(fs/2)], btype='bandpass')
    return lfilter(B, A, sig, axis=0)

def view(window, scale, refresh, figure, backend, version=1):
    matplotlib.use(backend)
    sns.set(style="whitegrid")

    figsize = np.int16(figure.split('x'))

    print("Looking for an EEG stream...")
    streams = resolve_byprop('type', 'EEG', timeout=LSL_SCAN_TIMEOUT)

    if len(streams) == 0:
        raise(RuntimeError("Can't find EEG stream."))
    print("Start acquiring data.")

    fig, ax = matplotlib.pyplot.subplots(figsize=figsize)
    lslv = LSLViewer(streams[0], fig, ax, window, scale)
    fig.canvas.mpl_connect('close_event', lslv.stop)

    lslv.start()
    matplotlib.pyplot.show()

class LSLViewer():
    def __init__(self, stream, fig, ax, window, scale, dejitter=True):
        """Init"""
        self.stream = stream
        self.window = window
        self.scale = scale
        self.dejitter = dejitter
        self.inlet = StreamInlet(stream, max_chunklen=LSL_EEG_CHUNK)
        self.filt = True
        self.subsample = VIEW_SUBSAMPLE

        info = self.inlet.info()
        description = info.desc()

        self.sfreq = info.nominal_srate()
        self.n_samples = int(self.sfreq * self.window)
        self.n_chan = info.channel_count()

        ch = description.child('channels').first_child()
        ch_names = [ch.child_value('label')]

        for i in range(self.n_chan):
            ch = ch.next_sibling()
            ch_names.append(ch.child_value('label'))

        self.ch_names = ch_names
        # Filter for only AF7 and AF8 channels
        self.channel_indices = [i for i, name in enumerate(self.ch_names) if name in ['AF7', 'AF8']] 
        print(f"Plotting channels: {[self.ch_names[i] for i in self.channel_indices]}")
        
        self.fig = fig
        self.ax = ax
        
        sns.despine(left=True)

        self.data = np.zeros((self.n_samples, self.n_chan))
        self.times = np.arange(-self.window, 0, 1. / self.sfreq)
        self.lines = []

        # Create lines for raw plot
        for ii in self.channel_indices:
            line, = self.ax.plot(self.times[::self.subsample],
                            self.data[::self.subsample, ii], 
                            lw=1, 
                            label=self.ch_names[ii])
            self.lines.append(line)

        # Configure axes
        self.ax.set_ylim(-scale, scale)
        self.ax.set_title('EEG with Blink Detection')
        self.ax.set_xlabel('Time (s)')
        self.ax.legend(loc='lower left')
        
        self.display_every = int(0.2 / (12 / self.sfreq))

        self.bf = firwin(32, np.array([1, 40]) / (self.sfreq / 2.), width=0.05,
                         pass_zero=False)
        self.af = [1.0]

        zi = lfilter_zi(self.bf, self.af)
        self.filt_state = np.tile(zi, (self.n_chan, 1)).transpose()
        self.data_f = np.zeros((self.n_samples, self.n_chan))
        self.last_blink_time = 0
        self.blink_cooldown = 0.1  # Minimum time between blink detections
        self.detected_blinks = set()  # Store timestamps of detected blinks
        self.total_blinks = 0  # blink counter
        
        # blink markers
        self.blink_markers = self.ax.scatter([], [], c='yellow', marker='*', s=200,
                                           label='Detected Blinks', zorder=3)
        
        # Update legend location to show all markers
        self.ax.legend(loc='lower left')
        
        self.blink_positions = []  # Store positions of detected blinks

    def update_plot(self):
        k = 0
        try:
            while self.started:
                # get data
                samples, timestamps = self.inlet.pull_chunk(timeout=1.0, max_samples=LSL_EEG_CHUNK)
                if timestamps:
                    if self.dejitter:
                        timestamps = np.float64(np.arange(len(timestamps)))
                        timestamps /= self.sfreq
                        timestamps += self.times[-1] + 1./self.sfreq
                    self.times = np.concatenate([self.times, timestamps])
                    self.n_samples = int(self.sfreq * self.window)
                    self.times = self.times[-self.n_samples:]
                    self.data = np.vstack([self.data, samples])
                    self.data = self.data[-self.n_samples:]
                    filt_samples, self.filt_state = lfilter(self.bf, self.af, samples, axis=0, zi=self.filt_state)
                    self.data = self.data[-self.n_samples:]
                    filt_samples, self.filt_state = lfilter(
                        self.bf, self.af,
                        samples,
                        axis=0, zi=self.filt_state)
                    self.data_f = np.vstack([self.data_f, filt_samples])
                    self.data_f = self.data_f[-self.n_samples:]
                    k += 1
                    if k == self.display_every:
                        plot_data = self.data_f if self.filt else self.data - self.data.mean(axis=0)
                        plot_data = bandpass(plot_data, 0.5, 5, self.sfreq, order=4)
                        
                        current_time = time.time()
                        
                        for idx, ii in enumerate(self.channel_indices):
                            # Update raw plot
                            self.lines[idx].set_xdata(self.times[::self.subsample] - self.times[-1])
                            self.lines[idx].set_ydata(plot_data[::self.subsample, ii])
                            
                            # Detect spikes using only the most recent 80% of plot_data
                            start_idx = int(len(plot_data) * 0.2)
                            recent_data = plot_data[start_idx:, ii]
                            recent_times = self.times[start_idx:]
                            pos_peaks, neg_peaks, _, _ = detect_eeg_spikes(recent_data)
                            
                            # Use recent times when detecting blink patterns
                            blink_times = detect_blink_pattern(pos_peaks, neg_peaks, recent_times)
                            for blink_time in blink_times:
                                blink_time_rounded = round(blink_time, 3)
                                if (blink_time_rounded not in self.detected_blinks and 
                                    current_time - self.last_blink_time > self.blink_cooldown):
                                    self.total_blinks += 1
                                    print(f"Blink #{self.total_blinks} detected on channel {self.ch_names[ii]}")
                                    self.detected_blinks.add(blink_time_rounded)
                                    
                                    self.last_blink_time = current_time
                                    
                                    # Find the closest time index for the blink
                                    time_idx = np.argmin(np.abs(self.times - blink_time))
                                    if time_idx < len(plot_data):
                                        # Add blink marker position using valid index
                                        self.blink_positions.append([blink_time, plot_data[time_idx, ii]])
                        
                        # Update blink markers and count display
                        if self.blink_positions:
                            positions = np.array(self.blink_positions)
                            # Calculate current window start time
                            current_window_start = self.times[-1] - self.window
                            # Only show markers within current time window
                            mask = positions[:, 0] > current_window_start
                            visible_positions = positions[mask]
                            self.blink_positions = list(positions[mask])
                            
                            if len(visible_positions) > 0:
                                self.blink_markers.set_offsets(np.c_[visible_positions[:, 0] - self.times[-1],
                                                                    visible_positions[:, 1]])
                            else:
                                self.blink_markers.set_offsets(np.c_[[], []])
                        
                        self.ax.set_xlim(-self.window, 0)
                        self.fig.canvas.draw()
                        k = 0
                else:
                    sleep(0.2)
        except RuntimeError as e:
            raise

    def start(self):
        self.started = True
        self.thread = Thread(target=self.update_plot)
        self.thread.daemon = True
        self.thread.start()

    def stop(self, close_event):
        self.started = False

if __name__ == "__main__":
    # Parameters for the viewer
    window = 5  # window size in seconds
    scale = 1500  # scale factor for the plot
    refresh = 0.1  # refresh rate in seconds
    figure = "15x6"  # figure size in inches
    backend = 'TkAgg'  # matplotlib backend

    # Start the viewer
    view(window=window, 
         scale=scale, 
         refresh=refresh, 
         figure=figure, 
         backend=backend)