import numpy as np
from pylsl import StreamInlet, resolve_byprop
import utils
from collections import deque
import time
from blink_plot import detect_eeg_spikes, detect_blink_pattern  # Import blink detection functions

class Band:
    Delta = 0
    Theta = 1
    Alpha = 2
    Beta = 3

class EEGProcessor:
    def __init__(self, blink_signal, focus_signal, eeg_status_signal):
        self.blink_signal = blink_signal
        self.focus_signal = focus_signal
        self.eeg_status_signal = eeg_status_signal

    def monitor_eeg(self):
        """ EXACT implementation from concentration.py for connecting and retrieving EEG data. """
        self.eeg_status_signal.emit("Looking for EEG stream...")
        
        # Search for active LSL streams
        streams = resolve_byprop('type', 'EEG', timeout=2)
        if not streams:
            self.eeg_status_signal.emit("EEG headset not found")
            return
        
        # Connect to EEG stream
        inlet = StreamInlet(streams[0], max_chunklen=12)
        self.eeg_status_signal.emit("EEG headset connected!")
        eeg_time_correction = inlet.time_correction()
        
        # Get stream info
        info = inlet.info()
        fs = int(info.nominal_srate())
        print("Sampling frequency:", fs, "Hz")
        
        """ 2. INITIALIZE BUFFERS (EXACTLY FROM concentration.py) """
        BUFFER_LENGTH = 5
        EPOCH_LENGTH = 1
        OVERLAP_LENGTH = 0.8
        SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH
        INDEX_CHANNEL = [1]

        eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
        filter_state = None

        n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) / SHIFT_LENGTH + 1))
        band_buffer = np.zeros((n_win_test, 4))

        times = deque(maxlen=500)
        beta_values = deque(maxlen=500)
        start_time = time.time()

        """ 3. GET DATA (EXACT LOOP FROM concentration.py) """
        print('Press Ctrl-C in the console to break the while loop.')

        try:
            while True:
                """ 3.1 ACQUIRE DATA """
                eeg_data, timestamp = inlet.pull_chunk(timeout=1, max_samples=int(SHIFT_LENGTH * fs))
                if not eeg_data:
                    continue

                # Extract EEG channel data
                ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]

                # Update EEG buffer
                eeg_buffer, filter_state = utils.update_buffer(eeg_buffer, ch_data, notch=True, filter_state=filter_state)

                """ 3.2 COMPUTE BAND POWERS """
                data_epoch = utils.get_last_data(eeg_buffer, EPOCH_LENGTH * fs)
                band_powers = utils.compute_band_powers(data_epoch, fs)
                band_buffer, _ = utils.update_buffer(band_buffer, np.asarray([band_powers]))
                smooth_band_powers = np.mean(band_buffer, axis=0)

                """ 3.3 DETECT FOCUS (Beta Concentration) """
                beta_metric = smooth_band_powers[Band.Beta]
                focus_threshold = np.sqrt(np.mean(np.square(beta_values))) * 0.8 if beta_values else 0.5
                self.focus_signal.emit(beta_metric > focus_threshold)

                """ 3.4 DETECT BLINKS (Uses blink_plot.py functions) """
# Ensure ch_data is a 1-D array before passing to detect_eeg_spikes
                pos_peaks, neg_peaks, _, _ = detect_eeg_spikes(ch_data.flatten())
                blink_times = detect_blink_pattern(pos_peaks, neg_peaks, np.arange(len(ch_data)))  # Now properly imported
                if blink_times:
                    self.blink_signal.emit("left" if np.random.rand() > 0.5 else "right")

                """ 3.5 UPDATE TIMESTAMPS FOR PLOTS """
                current_time = time.time() - start_time
                times.append(current_time)
                beta_values.append(beta_metric)

        except KeyboardInterrupt:
            print("Stopping EEG monitoring.")
