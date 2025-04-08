import numpy as np
from scipy.signal import lfilter, lfilter_zi, firwin, butter, find_peaks
from time import sleep, time
from pylsl import StreamInlet, resolve_byprop
from PyQt6.QtCore import QObject, pyqtSignal

import constants as C
import utils

def detect_eeg_spikes(data, threshold=C.BLINK_SPIKE_THRESHOLD, min_distance=C.BLINK_SPIKE_MIN_DISTANCE, prominence=C.BLINK_SPIKE_PROMINENCE):
    """Detect spikes in EEG data based on amplitude and prominence thresholds."""
    pos_peaks, pos_properties = find_peaks(data, height=threshold, distance=min_distance, prominence=prominence)

    neg_peaks, neg_properties = find_peaks(-data, height=threshold, distance=min_distance, prominence=prominence)

    return pos_peaks, neg_peaks 

def detect_blink_pattern(pos_peaks, neg_peaks, timestamps, max_interval=C.BLINK_MAX_INTERVAL):
    """
    Detect blink patterns: positive spike followed by negative spike within max_interval.
    Returns a list of blink timestamps (timestamp of the positive peak).
    """
    blink_times = []
    if len(pos_peaks) == 0 or len(neg_peaks) == 0:
        return blink_times

    neg_peak_times = timestamps[neg_peaks]
    pos_peak_times = timestamps[pos_peaks]

    # Find matching negative peaks for each positive peak
    neg_idx = 0
    for i, pos_peak_time in enumerate(pos_peak_times):
        # Find the first negative peak *after* this positive peak
        while neg_idx < len(neg_peak_times) and neg_peak_times[neg_idx] <= pos_peak_time:
            neg_idx += 1

        # Check next negative peaks within the interval
        current_neg_idx = neg_idx
        while current_neg_idx < len(neg_peak_times):
            interval = neg_peak_times[current_neg_idx] - pos_peak_time
            if interval > 0 and interval < max_interval:
                 # Found a potential match, use the positive peak's time
                 # Move to the next positive peak once a pair is found for positive peak
                 # This prevents one negative peak matching multiple preceding positive peaks too closely
                 blink_times.append(pos_peak_time)
                 break
            elif interval >= max_interval:
                 # Do not need to check more negative peaks for this positive peak
                 break
            current_neg_idx += 1

    return blink_times


def bandpass_butter(sig, lowcut, highcut, fs, order=4):
    """Apply Butterworth bandpass filter."""
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    # Avoid invalid frequency limits
    low = max(0.01, low)
    high = min(0.99, high)
    if low >= high:
         print(f"Warning: Lowcut ({lowcut}) >= Highcut ({highcut}). Skipping bandpass.")
         return sig # Return original signal if frequencies are invalid
    try:
        b, a = butter(order, [low, high], btype='bandpass')
        y = lfilter(b, a, sig, axis=0)
        return y
    except ValueError as e:
        # Return original signal on error
        print(f"Error in Butterworth filter: {e}. Check frequencies: low={low}, high={high}")
        return sig 


class BlinkDetector(QObject):
    blinkDetected = pyqtSignal(str)
    statusUpdate = pyqtSignal(str)
    finished = pyqtSignal()
    # Signal for Plotting 
    # Emits: timestamps (1D array), channel_data (dict like {'AF7': array, 'AF8': array}), blinks (list of [ts, val])
    plotDataReady = pyqtSignal(np.ndarray, dict, list)

    def __init__(self):
        super().__init__()
        self._running = False
        self.inlet = None
        self.filt_state_fir = None
        self.data_buffer = None
        self.time_buffer = None
        self.sfreq = C.MUSE_SAMPLING_EEG_RATE
        self.n_samples_window = int(self.sfreq * C.BLINK_WINDOW_SECONDS)
        self.n_chan = C.MUSE_NB_EEG_CHANNELS
        self.channel_indices = [] # Indices for AF7, AF8 within the full stream
        self.plot_channel_indices_map = {} # Map global index (AF7/AF8) to name
        self.ch_names = []
        self.last_blink_time = 0
        self.detected_blinks_timestamps = set() # Store raw timestamps for cooldown
        self.blink_plot_markers = [] # Store [ts, value] for plotting

        # FIR filter
        self.bf_fir = firwin(32, np.array([1, 40]) / (self.sfreq / 2.), width=0.05, pass_zero=False)
        self.af_fir = [1.0]

        self.plot_update_counter = 0
        # Update plot every chunk with data 
        self.plot_update_interval = 1

    def _initialize_stream(self):
        self.statusUpdate.emit("Blink Detector: Looking for EEG stream...")
        streams = resolve_byprop('type', 'EEG', timeout=C.LSL_SCAN_TIMEOUT)
        if not streams:
            self.statusUpdate.emit("Blink Detector: EEG stream not found!")
            return False

        self.statusUpdate.emit("Blink Detector: Connecting to EEG stream...")
        try:
            self.inlet = StreamInlet(streams[0], max_chunklen=C.LSL_EEG_CHUNK)
            info = self.inlet.info()
            self.sfreq = int(info.nominal_srate())
            self.n_chan = info.channel_count()
            self.n_samples_window = int(self.sfreq * C.BLINK_WINDOW_SECONDS)
            self.plot_update_interval = max(1, int(self.sfreq / C.LSL_EEG_CHUNK / 5)) # Recalculate

            desc = info.desc()
            ch = desc.child('channels').first_child()
            current_ch_names = []
            self.channel_indices = [] # Reset before filling
            self.plot_channel_indices_map = {} # Reset map
            for i in range(self.n_chan):
                label = ch.child_value('label')
                current_ch_names.append(label)
                if label in ['AF7', 'AF8']:
                # if label in ['TP9', 'TP10']:
                    self.channel_indices.append(i) # Store global index
                    self.plot_channel_indices_map[i] = label # Map global index to name
                ch = ch.next_sibling()
            self.ch_names = current_ch_names

            if not self.channel_indices:
                 self.statusUpdate.emit("Blink Detector: AF7/AF8 channels not found!")
                 self.inlet.close_stream(); return False

            self.data_buffer = np.zeros((self.n_samples_window, self.n_chan))
            self.time_buffer = np.zeros(self.n_samples_window)
            zi = lfilter_zi(self.bf_fir, self.af_fir)
            self.filt_state_fir = np.tile(zi, (self.n_chan, 1)).transpose()
            self.bf_fir = firwin(32, np.array([1, 40]) / (self.sfreq / 2.), width=0.05, pass_zero=False)

            self.statusUpdate.emit(f"Blink Detector: Connected to {info.name()} ({self.sfreq}Hz)")
            print(f"Blink Detector: Found channels {self.ch_names}")
            print(f"Blink Detector: Using channels {[self.ch_names[i] for i in self.channel_indices]}")
            return True
        except Exception as e:
            self.statusUpdate.emit(f"Blink Detector: Error connecting stream: {e}"); return False

    def run(self):
        self._running = True
        if not self._initialize_stream():
            self._running = False; self.finished.emit(); return

        # Store markers detected since last plot emit
        markers_since_last_emit = []

        while self._running:
            try:
                samples, timestamps = self.inlet.pull_chunk(timeout=1.0, max_samples=C.LSL_EEG_CHUNK)

                if timestamps:
                    timestamps = np.array(timestamps)
                    samples = np.array(samples)
                    num_new = len(timestamps)

                    # Update buffers (raw data)
                    self.data_buffer = np.vstack((self.data_buffer[num_new:], samples))
                    self.time_buffer = np.concatenate((self.time_buffer[num_new:], timestamps))

                    # Apply FIR filter 

                    current_time = time()
                    # Use temporary list for markers in this chunk
                    newly_detected_blinks_in_chunk = []

                    for global_chan_idx in self.channel_indices:
                        channel_name = self.plot_channel_indices_map[global_chan_idx]
                        channel_data = self.data_buffer[:, global_chan_idx] # Get raw data window

                        # Apply Butterworth bandpass (0.5-5Hz) for detection
                        bp_data = bandpass_butter(channel_data, 0.5, 5, self.sfreq, order=4)

                        # Use recent part for spike detection
                        recent_samples = int(min(2.0, C.BLINK_WINDOW_SECONDS) * self.sfreq)
                        start_idx = self.n_samples_window - recent_samples
                        if start_idx < 0: 
                            start_idx = 0

                        recent_bp_data = bp_data[start_idx:]
                        recent_times = self.time_buffer[start_idx:]

                        if len(recent_bp_data) < C.BLINK_SPIKE_MIN_DISTANCE: continue

                        pos_peaks_idx, neg_peaks_idx = detect_eeg_spikes(recent_bp_data)

                        if len(recent_times) > 0:
                            blink_event_times = detect_blink_pattern(pos_peaks_idx, neg_peaks_idx, recent_times)

                            for blink_ts in blink_event_times:
                                # Use raw timestamp for cooldown check
                                if (blink_ts not in self.detected_blinks_timestamps and
                                    current_time - self.last_blink_time > C.BLINK_COOLDOWN):

                                    self.last_blink_time = current_time
                                    self.detected_blinks_timestamps.add(blink_ts)

                                    # Find index in the full buffer for plot marker value
                                    time_diff = np.abs(self.time_buffer - blink_ts)
                                    if np.min(time_diff) < (1.0 / self.sfreq * 2): # Check if time exists in buffer
                                        buffer_idx = np.argmin(time_diff)
                                        # Use the value from the bandpassed data for the marker
                                        marker_value = bp_data[buffer_idx]
                                        newly_detected_blinks_in_chunk.append([blink_ts, marker_value])
                                    else:
                                         newly_detected_blinks_in_chunk.append([blink_ts, 0]) # Fallback marker value


                                    direction = "left" if channel_name == 'AF7' else "right"
                                    print(f"Blink Detector: Detected {direction} blink @ {blink_ts:.3f}s")
                                    self.blinkDetected.emit(direction)

                    # Add new markers to the list for next emit
                    markers_since_last_emit.extend(newly_detected_blinks_in_chunk)

                    # Prepare and Emit Plot Data 
                    self.plot_update_counter += 1
                    if self.plot_update_counter >= self.plot_update_interval:
                        self.plot_update_counter = 0
                        plot_ch_data = {}
                        # Get the filtered data for the plot channels
                        for global_idx in self.channel_indices:
                            ch_name = self.plot_channel_indices_map[global_idx]
                            # Apply the 0.5-5Hz filter again for consistent plot data or could maintain a fully filtered buffer
                            plot_data_ch = (bandpass_butter(self.data_buffer[:, global_idx], 0.5, 5, self.sfreq, order=4))
                            # plot_data_ch = abs(self.data_buffer[:, global_idx])
                            plot_ch_data[ch_name] = plot_data_ch

                        # No longer filter blink markers in the detector
                        # Send only markers collected since last emit
                        markers_to_send = list(markers_since_last_emit)
                        markers_since_last_emit = [] # Clear for next interval
                        
                        # Emit data for plotting
                        times_copy = self.time_buffer.copy()
                        plot_ch_data_copy = {k: v.copy() for k, v in plot_ch_data.items()}
                        
                        self.plotDataReady.emit(times_copy, plot_ch_data_copy, markers_to_send)


                    # Cleanup old detected blink timestamps for cooldown
                    min_valid_time = time() - max(C.BLINK_WINDOW_SECONDS, 5.0) # Keep history longer than window
                    self.detected_blinks_timestamps = {t for t in self.detected_blinks_timestamps if t >= min_valid_time}


                else: sleep(0.05)
            except Exception as e:
                self.statusUpdate.emit(f"Blink Detector: Error in loop: {e}")
                sleep(0.5)

        if self.inlet:
            try: self.inlet.close_stream()
            except Exception as e: self.statusUpdate.emit(f"Blink Detector: Error closing stream: {e}")
        self.statusUpdate.emit("Blink Detector: Stopped.")
        self.finished.emit()

    def stop(self):
        self.statusUpdate.emit("Blink Detector: Stopping...")
        self._running = False