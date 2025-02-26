import numpy as np
import matplotlib.pyplot as plt
from pylsl import StreamInlet, resolve_byprop
import time
import utils  
from collections import deque
from signals import PlotSignals  # Import signal communication class

# Enum for frequency bands
class Band:
    Delta = 0
    Theta = 1
    Alpha = 2
    Beta = 3

""" EXPERIMENTAL PARAMETERS """
BUFFER_LENGTH = 5            # seconds of EEG data buffer
EPOCH_LENGTH = 1             # seconds for FFT
OVERLAP_LENGTH = 0.8         # seconds overlap between epochs
SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH
INDEX_CHANNEL = [1]          # channel index

# Thresholding constant 
rms_constant = .8

if __name__ == "__main__":
    # Create signal client to send messages to main GUI
    signal_client = PlotSignals(is_server=False)
    
    # Connect to LSL stream
    print('Looking for an EEG stream...')
    signal_client.send_message("status", "Focus plot: Looking for an EEG stream...")
    
    streams = resolve_byprop('type', 'EEG', timeout=2)
    if not streams:
        signal_client.send_message("status", "Focus plot: EEG stream not found!")
        raise RuntimeError("Can't find EEG stream.")
        
    inlet = StreamInlet(streams[0], max_chunklen=12)
    fs = int(inlet.info().nominal_srate())
    print("Sampling frequency:", fs, "Hz")
    signal_client.send_message("status", "Focus plot: Connected to EEG stream!")
    
    # Initialize EEG and band buffers
    eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
    filter_state = None
    n_win = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) / SHIFT_LENGTH + 1))
    band_buffer = np.zeros((n_win, 4))
    
    # Setup plot
    plt.ion()
    fig, ax = plt.subplots(figsize=(12, 6))
    times = deque(maxlen=500)         # Time stamps
    beta_values = deque(maxlen=500)     # Beta power values
    line_beta, = ax.plot([], [], label="Beta Power")
    line_thresh, = ax.plot([], [], 'r--', label="Threshold")
    ax.set_ylim(-.5, 1)                
    ax.set_title('Beta Concentration Over Time')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Beta Power')
    ax.legend()
    start_time = time.time()
    
    prev_state = None  

    focus_levels = []  # history of beta values
    
    try:
        while True:
            # get data
            eeg_data, _ = inlet.pull_chunk(timeout=1, max_samples=int(SHIFT_LENGTH * fs))
            if not eeg_data:
                continue
            ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]
            eeg_buffer, filter_state = utils.update_buffer(eeg_buffer, ch_data, notch=True, filter_state=filter_state)
            
            # Compute band powers
            data_epoch = utils.get_last_data(eeg_buffer, EPOCH_LENGTH * fs)
            band_powers = utils.compute_band_powers(data_epoch, fs)
            band_buffer, _ = utils.update_buffer(band_buffer, np.asarray([band_powers]))
            smooth_band_powers = np.mean(band_buffer, axis=0)
            
            # Compute beta power metric
            beta_metric = -smooth_band_powers[Band.Beta]
            focus_levels.append(beta_metric)
            current_threshold = np.sqrt(np.mean(np.square(focus_levels))) * rms_constant
            
            # Thresholding with state change detection 
            new_state = "above" if beta_metric > current_threshold else "below"
            if prev_state is not None and new_state != prev_state:
                # Send a message to the main GUI before printing
                signal_client.send_message("focus", new_state=="above")
                print("Above threshold" if new_state=="above" else "Below threshold")
            prev_state = new_state
            
            # Update plot
            current_time = time.time() - start_time
            times.append(current_time)
            beta_values.append(beta_metric)
            
            line_beta.set_xdata(list(times))
            line_beta.set_ydata(list(beta_values))
            if times:
                xmin, xmax = ax.get_xlim()
                line_thresh.set_data([xmin, xmax], [current_threshold, current_threshold])
            
            ax.set_xlim(max(0, current_time - 100), current_time + 0.5)
            fig.canvas.draw()
            fig.canvas.flush_events()
    except KeyboardInterrupt:
        plt.ioff()
        signal_client.send_message("status", "Focus plot: Closed")
        print("Closing!")
        signal_client.stop()

