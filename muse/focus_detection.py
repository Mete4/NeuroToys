# --- In focus_detection.py ---

import numpy as np
from scipy.signal import lfilter, lfilter_zi
from time import sleep, time
from pylsl import StreamInlet, resolve_byprop
from PyQt6.QtCore import QObject, pyqtSignal

import constants as C
import utils

class Band: Delta = 0; Theta = 1; Alpha = 2; Beta = 3

class FocusDetector(QObject):
    focusStateChanged = pyqtSignal(bool)
    statusUpdate = pyqtSignal(str)
    finished = pyqtSignal()
    # Emits: timestamp (float), beta_metric (float), threshold (float)
    plotDataReady = pyqtSignal(float, float, float)

    def __init__(self):
        super().__init__()
        self._running = False
        self.inlet = None
        self.sfreq = C.MUSE_SAMPLING_EEG_RATE
        self.channel_index = C.FOCUS_INDEX_CHANNEL[0]
        self.buffer_length_samples = int(C.FOCUS_BUFFER_LENGTH * self.sfreq)
        self.epoch_length_samples = int(C.FOCUS_EPOCH_LENGTH * self.sfreq)
        self.shift_length_samples = int(C.FOCUS_SHIFT_LENGTH * self.sfreq)
        self.eeg_buffer = None
        self.band_buffer = None
        self.filter_state_notch = None
        self.focus_levels = []
        self.prev_state = None
        self.n_win_band = int(np.floor((C.FOCUS_BUFFER_LENGTH - C.FOCUS_EPOCH_LENGTH) / C.FOCUS_SHIFT_LENGTH + 1))
        self.start_time = time() # Record start time for relative plot timestamps
        
        # New manual threshold parameters
        self.use_manual_threshold = False
        self.manual_threshold_value = 0.5

    def set_manual_threshold(self, threshold_value, enabled=True):
        """Set the manual threshold value and enable/disable manual mode"""
        self.manual_threshold_value = threshold_value
        self.use_manual_threshold = enabled
        print(f"Focus Detector: Manual threshold {'enabled' if enabled else 'disabled'}, value: {threshold_value:.2f}")

    def _initialize_stream(self):
        self.statusUpdate.emit("Focus Detector: Looking for EEG stream...")
        streams = resolve_byprop('type', 'EEG', timeout=C.LSL_SCAN_TIMEOUT)
        if not streams:
            self.statusUpdate.emit("Focus Detector: EEG stream not found!"); return False

        self.statusUpdate.emit("Focus Detector: Connecting to EEG stream...")
        try:
            self.inlet = StreamInlet(streams[0], max_chunklen=C.LSL_EEG_CHUNK)
            info = self.inlet.info()
            self.sfreq = int(info.nominal_srate())
            # Recalculate buffer sizes
            self.buffer_length_samples = int(C.FOCUS_BUFFER_LENGTH * self.sfreq)
            self.epoch_length_samples = int(C.FOCUS_EPOCH_LENGTH * self.sfreq)
            self.shift_length_samples = int(C.FOCUS_SHIFT_LENGTH * self.sfreq)
            self.n_win_band = int(np.floor((C.FOCUS_BUFFER_LENGTH - C.FOCUS_EPOCH_LENGTH) / C.FOCUS_SHIFT_LENGTH + 1))
            if self.channel_index >= info.channel_count():
                 self.statusUpdate.emit(f"Focus Detector: Channel index ({self.channel_index}) out of bounds."); return False
            self.eeg_buffer = np.zeros((self.buffer_length_samples, 1))
            self.band_buffer = np.zeros((self.n_win_band, 4))
            self.start_time = time() # Reset start time on connect
            self.statusUpdate.emit(f"Focus Detector: Connected to {info.name()} ({self.sfreq}Hz)")
            print(f"Focus Detector: Using channel index {self.channel_index}"); return True
        except Exception as e:
            self.statusUpdate.emit(f"Focus Detector: Error connecting stream: {e}"); return False

    def run(self):
        self._running = True
        if not self._initialize_stream():
            self._running = False; self.finished.emit(); return

        while self._running:
            try:
                samples, timestamps = self.inlet.pull_chunk(timeout=1.0, max_samples=self.shift_length_samples)
                if timestamps:
                    samples = np.array(samples)
                    ch_data = samples[:, [self.channel_index]]
                    self.eeg_buffer, self.filter_state_notch = utils.update_buffer(
                        self.eeg_buffer, ch_data, notch=True, filter_state=self.filter_state_notch)
                    data_epoch = utils.get_last_data(self.eeg_buffer, self.epoch_length_samples)
                    if data_epoch.shape[0] < self.epoch_length_samples or np.all(data_epoch == 0): continue

                    band_powers = utils.compute_band_powers(data_epoch, self.sfreq)
                    self.band_buffer, _ = utils.update_buffer(self.band_buffer, np.asarray([band_powers]))
                    smooth_band_powers = np.mean(self.band_buffer, axis=0)
                    beta_metric = -smooth_band_powers[Band.Beta]
                    self.focus_levels.append(beta_metric)
                    max_history = int(3 * 60 / C.FOCUS_SHIFT_LENGTH); # ~3 mins history
                    if len(self.focus_levels) > max_history: self.focus_levels = self.focus_levels[-max_history:]

                    # Use manual threshold if enabled, otherwise calculate automatically
                    if self.use_manual_threshold:
                        current_threshold = self.manual_threshold_value
                    else:
                        current_threshold = np.sqrt(np.mean(np.square(self.focus_levels))) * C.FOCUS_RMS_CONSTANT if len(self.focus_levels) > 0 else 0.25
                    
                    new_state = beta_metric > current_threshold

                    current_plot_time = time() - self.start_time # Time relative to start
                    self.plotDataReady.emit(current_plot_time, beta_metric, current_threshold)

                    if self.prev_state is None or new_state != self.prev_state:
                        self.focusStateChanged.emit(new_state)
                        threshold_type = "manual" if self.use_manual_threshold else "auto"
                        print(f"Focus Detector: State changed to {'FOCUSED' if new_state else 'UNFOCUSED'} " 
                              f"(Metric: {beta_metric:.3f}, {threshold_type} Thr: {current_threshold:.3f})")
                        self.prev_state = new_state
                else: sleep(0.05)
            except Exception as e:
                self.statusUpdate.emit(f"Focus Detector: Error in loop: {e}"); sleep(0.5)

        if self.inlet:
             try: self.inlet.close_stream()
             except Exception as e: self.statusUpdate.emit(f"Focus Detector: Error closing stream: {e}")
        self.statusUpdate.emit("Focus Detector: Stopped.")
        self.finished.emit()

    def reset_threshold(self):
        print("Focus Detector: Resetting threshold history.")
        self.focus_levels = []
        self.prev_state = None

    def stop(self):
        self.statusUpdate.emit("Focus Detector: Stopping...")
        self._running = False