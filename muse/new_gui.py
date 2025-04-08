import sys
import asyncio
import threading
import numpy as np
import os
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QSizePolicy 
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, QEventLoop

import matplotlib
matplotlib.use('QtAgg') 
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from collections import deque
import constants as C 
from blink_detection import BlinkDetector
from focus_detection import FocusDetector


try:
    sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Bluetooth/bluetooth_setup'))
    from bluetooth import BluetoothController
except ImportError:
    print("ERROR: Could not import BluetoothController.")

try:
    from muselsl import stream, list_muses
    from bleak.exc import BleakError
except ImportError:
    print("Warning: muselsl/BleakError not found.")
    stream, list_muses, BleakError = None, None, Exception



class BluetoothThread(QThread):
    bluetooth_status_signal = pyqtSignal(str)
    connection_lost_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.controller = BluetoothController()
        self.connection_check_timer = QTimer()
        self.loop = None
        self.was_connected = False

    def run(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            self.connection_check_timer.moveToThread(self.thread())
            self.connection_check_timer.timeout.connect(self.check_connection_status)
            self.loop.run_until_complete(self.connect_bluetooth())
            if self.controller.connected:
                self.connection_check_timer.start(2000)
            self.loop.run_forever()
        except Exception as e:
            print(f"Error in Bluetooth thread: {e}")
            try:
                self.bluetooth_status_signal.emit(f"Bluetooth Error: {str(e)}")
            except RuntimeError:
                 print("Error emitting BT status signal - likely closing")
        finally:
            if self.loop:
                if self.loop.is_running():
                     self.loop.call_soon_threadsafe(self.loop.stop)
                self.msleep(100)
                if not self.loop.is_closed():
                    self.loop.close()

    def check_connection_status(self):
        if not self.controller.connected and self.was_connected:
             print("BT Check: Connection lost detected by periodic check.")
             self.was_connected = False
             self.bluetooth_status_signal.emit("Bluetooth Disconnected (Checked)")
             self.connection_lost_signal.emit()
             if self.connection_check_timer.isActive():
                 self.connection_check_timer.stop()

    async def connect_bluetooth(self):
        print("BluetoothThread: Attempting connection...")
        try:
            connected = await self.controller.connect("ESP_CAR")
            print(f"BluetoothThread: controller.connect returned: {connected}")
            if connected:
                self.was_connected = True
                self.bluetooth_status_signal.emit("Bluetooth Connected!")
                if not self.connection_check_timer.isActive():
                    self.connection_check_timer.start(2000)
            else:
                self.was_connected = False
                self.bluetooth_status_signal.emit("Bluetooth Failed to Connect")
        except Exception as e:
            print(f"BluetoothThread: Exception during connection: {e}")
            self.was_connected = False
            self.bluetooth_status_signal.emit(f"Bluetooth Connection Error: {e}")

    def send_command(self, command):
        if not self.controller.connected or not self.was_connected:
            print(f"BT Send: Not connected. Cannot send: {command}")
            if self.was_connected:
                 self.was_connected = False
                 self.bluetooth_status_signal.emit("Bluetooth Disconnected (Send Fail)")
                 self.connection_lost_signal.emit()
                 if self.connection_check_timer.isActive():
                     self.connection_check_timer.stop()
            return

        if not self.loop or self.loop.is_closed():
            print("BT Send: Event loop closed. Cannot send.")
            if self.was_connected:
                 self.was_connected = False
                 self.bluetooth_status_signal.emit("Bluetooth Disconnected (Loop Closed)")
                 self.connection_lost_signal.emit()
                 if self.connection_check_timer.isActive():
                     self.connection_check_timer.stop()
            return

        async def _send():
            success = False
            try:
                await self.controller.send_command(command) 
                success = True
                print(f"BT Send successful (no response wait): {command}")
            except Exception as e:
                print(f"BT Send: Failed to send command '{command}' in _send: {e}")
                if self.was_connected:
                     self.was_connected = False
                     self.bluetooth_status_signal.emit(f"Bluetooth Disconnected (Send Error: {e})")
                     self.connection_lost_signal.emit()
                     if self.connection_check_timer.isActive():
                         self.connection_check_timer.stop()
        asyncio.run_coroutine_threadsafe(_send(), self.loop)

    def stop_thread(self):
        print("BluetoothThread: Stopping...")
        if self.connection_check_timer.isActive():
            self.connection_check_timer.stop()
        if self.loop and self.loop.is_running():
            print("BluetoothThread: Disconnecting controller (if connected)...")
            if self.controller and self.controller.connected:
                 future = asyncio.run_coroutine_threadsafe(self.controller.disconnect(), self.loop)
                 try: future.result(timeout=2.0)
                 except TimeoutError: print("BluetoothThread: Disconnect timed out.")
                 except Exception as e: print(f"BluetoothThread: Error during disconnect: {e}")
            print("BluetoothThread: Stopping loop...")
            self.loop.call_soon_threadsafe(self.loop.stop)
        self.requestInterruption()
        if self.isRunning():
            print("BluetoothThread: Waiting for thread to finish...")
            self.wait(2000)
            if self.isRunning():
                 print("BluetoothThread: Thread did not finish gracefully, terminating.")
                 self.terminate()



class EEGMonitorGUI(QWidget):
    _muse_scan_result_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        # Detectors/Threads
        self.blink_detector = None
        self.focus_detector = None
        self.blink_thread = None
        self.focus_thread = None
        # Bluetooth
        self.bluetooth_thread = None
        # Muse Streaming
        self.scan_thread = None
        self.streaming_thread = None
        # State
        self.blink_detector_connected = False
        self.focus_detector_connected = False
        self.is_moving_forward = False
        self.reverse_mode = False
        # Timers
        self.direction_timer = QTimer(self)
        self.direction_timer.setSingleShot(True)
        self.direction_timer.timeout.connect(self.reset_direction_display)
        self._muse_scan_result_signal.connect(self.on_muse_scan_complete)

        # Plotting
        self.blink_canvas = None
        self.blink_ax = None
        self.blink_lines = {} #  {'AF7': line, 'AF8': line}
        self.blink_markers = None
        # Add persistent marker list for GUI
        self.gui_blink_markers_list = []
        self.focus_canvas = None
        self.focus_ax = None
        self.focus_beta_line = None
        self.focus_thresh_line = None
        self.focus_times = deque(maxlen=500) 
        self.focus_beta_values = deque(maxlen=500)
        self.focus_threshold_values = deque(maxlen=500)

        self.initUI()

    def initUI(self):
        layout = QVBoxLayout()

        # Status Labels and Movement Display
        self.eeg_status_label = QLabel("Status: Ready")
        layout.addWidget(self.eeg_status_label)
        self.bluetooth_status_label = QLabel("Bluetooth Status: Not Connected")
        layout.addWidget(self.bluetooth_status_label)
        self.movement_display = QLabel("Stop")
        self.movement_display.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.movement_display.setStyleSheet("font-size: 32px; text-align: center; padding: 10px;")
        layout.addWidget(self.movement_display)

        #REVERSE GUI LOGIC
        self.direction_mode_label = QLabel("Direction: FORWARD")
        self.direction_mode_label.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.direction_mode_label.setStyleSheet("font-size: 20px; text-align: center; padding: 6px;")
        layout.addWidget(self.direction_mode_label)

        # Buttons
        self.scan_stream_button = QPushButton("Scan and Start Muse Stream") 
        self.scan_stream_button.clicked.connect(self.scan_and_start_muse)
        self.scan_stream_button.setEnabled(list_muses is not None)
        if list_muses is None: self.scan_stream_button.setToolTip("muselsl library not found")
        layout.addWidget(self.scan_stream_button)

        self.connect_eeg_button = QPushButton("Connect to EEG Stream") 
        self.connect_eeg_button.clicked.connect(self.start_eeg_detectors)
        layout.addWidget(self.connect_eeg_button)

        self.connect_bluetooth_button = QPushButton("Connect to Bluetooth Car") 
        self.connect_bluetooth_button.setEnabled(False)
        self.connect_bluetooth_button.clicked.connect(self.connect_bluetooth)
        layout.addWidget(self.connect_bluetooth_button)

        plots_layout = QVBoxLayout()
        left_right_layout = QHBoxLayout()

        # Blink Plot
        self.blink_figure = Figure(figsize=(5, 4)) # side-by-side size
        self.blink_canvas = FigureCanvas(self.blink_figure)
        self.blink_ax = self.blink_figure.add_subplot(111)

        # Empty lines for AF7, AF8
        self.blink_lines['AF7'], = self.blink_ax.plot([], [], lw=1, label='AF7')
        self.blink_lines['AF8'], = self.blink_ax.plot([], [], lw=1, label='AF8')
        self.blink_markers = self.blink_ax.vlines([], [], [], color='green', linewidth=1, label='Blinks', zorder=3) # vertical lines instead of stars
        self.blink_ax.set_ylim(-1500, 1500) 
        self.blink_ax.set_xlim(-C.BLINK_WINDOW_SECONDS, 0)
        self.blink_ax.set_title('Blink Detection (AF7/AF8)')
        self.blink_ax.set_xlabel('Time (s)')
        self.blink_ax.set_ylabel('Amplitude (uV)') 
        self.blink_ax.legend(loc='lower left')
        self.blink_figure.tight_layout()

        # Set size policy to allow expansion
        self.blink_canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_right_layout.addWidget(self.blink_canvas)
        

        # Focus plot
        self.focus_figure = Figure(figsize=(5, 4)) # side-by-side size
        self.focus_canvas = FigureCanvas(self.focus_figure)
        self.focus_ax = self.focus_figure.add_subplot(111)
        self.focus_beta_line, = self.focus_ax.plot([], [], label="Beta Power")
        self.focus_thresh_line, = self.focus_ax.plot([], [], 'r--', label="Threshold")
        self.focus_ax.set_ylim(-.5, 1) 
        self.focus_ax.set_title('Focus Metric Over Time')
        self.focus_ax.set_xlabel('Time (s)')
        self.focus_ax.set_ylabel('Metric Value')
        self.focus_ax.legend(loc='lower left')
        self.focus_figure.tight_layout()
        self.focus_canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        left_right_layout.addWidget(self.focus_canvas)
        

        # Add the horizontal layout to the main layout
        plots_layout.addLayout(left_right_layout)
        
        # Reset button for focus plot
        self.reset_focus_button = QPushButton("Reset Focus Threshold")
        self.reset_focus_button.setEnabled(False) # Enable when focus detector running
        self.reset_focus_button.clicked.connect(self.reset_focus_threshold_action)
        plots_layout.addWidget(self.reset_focus_button)
        
        layout.addLayout(plots_layout)

        self.setLayout(layout)
        self.setWindowTitle("EEG Car Monitor")
        self.resize(1200, 800) 


    def _perform_scan_and_emit(self):
        """Target function for the scan thread. Sets up loop, runs scan, emits result via signal."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        muses = None
        error_msg = None

        if not list_muses:
            error_msg = "muselsl library not available."
            print(f"Scan Thread: {error_msg}")
        else:
            try:
                print("Scan Thread: Starting list_muses scan...")
                muses = list_muses() 
                print(f"Scan Thread: Scan finished. Found: {muses}")
            except BleakError as be:
                error_msg = f"Bluetooth scan error (Bleak): {be}. Ensure Bluetooth is enabled."
                print(f"Scan Thread: {error_msg}")
            except Exception as e:
                error_msg = f"Unexpected error during scan: {e}"
                print(f"Scan Thread: {error_msg}")
                import traceback
                traceback.print_exc()

        # Emit the result back to the main GUI thread 
        self._muse_scan_result_signal.emit(muses)

        loop.close()
        print("Scan Thread: Exiting.")


    def scan_and_start_muse(self):
        """ Starts the Muse scan in a standard Python thread. """
        if self.scan_thread and self.scan_thread.is_alive():
            print("Muse scan already in progress.")
            return
        if self.streaming_thread and self.streaming_thread.is_alive():
            print("Muse streaming already active.")
            return

        self.scan_stream_button.setEnabled(False)
        self.scan_stream_button.setText("Scanning...")
        self.eeg_status_label.setText("Status: Scanning for Muse...") 

        # Start the scan in a standard thread
        self.scan_thread = threading.Thread(target=self._perform_scan_and_emit, daemon=True)
        self.scan_thread.start()


    # Connected to _muse_scan_result_signal and runs on the GUI thread
    def on_muse_scan_complete(self, muses):
        """ Handles the result of the Muse scan received via signal """
        print(f"GUI Thread: Scan complete signal received. Muses: {muses}")
        self.scan_stream_button.setText("Scan and Start Muse Stream") 

        if muses: 
            selected_muse = muses[0]
            muse_address = selected_muse.get('address')
            muse_name = selected_muse.get('name', 'Unknown Muse')
            if not muse_address:
                 print("Error: Found Muse but address is missing.")
                 self.eeg_status_label.setText("Status: Found Muse, but address missing.")
                 self.scan_stream_button.setEnabled(True)
                 return

            self.eeg_status_label.setText(f"Status: Found {muse_name}. Starting LSL stream...")
            print(f"Attempting to start stream for {muse_name} ({muse_address})")
            if stream:
                 self.streaming_thread = threading.Thread(
                     target=run_stream_in_thread, args=(muse_address,), daemon=True)
                 self.streaming_thread.start()
                 QTimer.singleShot(1000, self.check_stream_started)
                 self.scan_stream_button.setEnabled(False)
                 self.scan_stream_button.setText("Muse Streaming...")
            else:
                 self.eeg_status_label.setText("Status: Found Muse, but 'stream' function missing.")
                 self.scan_stream_button.setEnabled(True)
        else: # Allow user to try again and show error message
            error_text = "Muse scan failed: Check if Muse is ON or connected elsewhere."
            self.eeg_status_label.setText(f"Status: {error_text}")
            self.scan_stream_button.setEnabled(True) 

        self.scan_thread = None # Clear thread reference once done

    def check_stream_started(self):
        if self.streaming_thread and not self.streaming_thread.is_alive():
             print("Error: Muse streaming thread exited unexpectedly.")
             self.eeg_status_label.setText("Status: Failed to start Muse LSL stream.")
             self.scan_stream_button.setText("Scan and Start Muse Stream")
             self.scan_stream_button.setEnabled(True)
             self.streaming_thread = None
        elif self.streaming_thread and self.streaming_thread.is_alive():
             print("Muse streaming thread appears active.")
             self.eeg_status_label.setText("Status: Muse LSL Stream Active.")


    def start_eeg_detectors(self):
        if self.blink_thread or self.focus_thread:
            return
        self.connect_eeg_button.setEnabled(False)
        self.connect_eeg_button.setText("Connecting LSL...")
        self.eeg_status_label.setText("Status: Initializing Detectors...")
        self.blink_detector_connected = False
        self.focus_detector_connected = False
        self.update_bluetooth_button_state()

        # Blink Detector 
        self.blink_detector = BlinkDetector()
        self.blink_thread = QThread(self)
        self.blink_detector.moveToThread(self.blink_thread)
        self.blink_detector.statusUpdate.connect(self.update_detector_status)
        self.blink_detector.blinkDetected.connect(self.update_movement)
        # Connect plot signal
        self.blink_detector.plotDataReady.connect(self.update_blink_plot)
        self.blink_detector.finished.connect(self.on_detector_finished)
        self.blink_thread.started.connect(self.blink_detector.run)
        self.blink_thread.finished.connect(self.on_detector_thread_finished)
        self.blink_thread.start()

        # Focus Detector 
        self.focus_detector = FocusDetector()
        self.focus_thread = QThread(self)
        self.focus_detector.moveToThread(self.focus_thread)
        self.focus_detector.statusUpdate.connect(self.update_detector_status)
        self.focus_detector.focusStateChanged.connect(self.update_focus)
        # Connect plot signal 
        self.focus_detector.plotDataReady.connect(self.update_focus_plot)
        self.focus_detector.finished.connect(self.on_detector_finished)
        self.focus_thread.started.connect(self.focus_detector.run)
        self.focus_thread.finished.connect(self.on_detector_thread_finished)
        # Enable reset button when focus detector starts
        self.focus_thread.finished.connect(lambda: self.reset_focus_button.setEnabled(False))
        self.focus_thread.start()


    # Plot Update 
    def update_blink_plot(self, timestamps, channel_data_dict, new_blink_markers):
        if not timestamps.any() or not self.blink_canvas: return 

        # Add newly received markers to the persistent list
        self.gui_blink_markers_list.extend(new_blink_markers)

        # Filter the persistent list based on the current timestamp window
        min_plot_time = timestamps[0]
        max_plot_time = timestamps[-1]
        # Keep markers within the received timestamp range
        self.gui_blink_markers_list = [m for m in self.gui_blink_markers_list 
                                     if min_plot_time <= m[0] <= max_plot_time]

        # Make time relative to the end
        relative_times = timestamps - timestamps[-1]

        # Update EEG lines
        for name, line in self.blink_lines.items():
            if name in channel_data_dict:
                subsample = C.SUBSAMPLE # Adjust subsampling factor
                line.set_data(relative_times[::subsample], channel_data_dict[name][::subsample])

        # Update blink markers using the filtered persistent list - vertical lines version
        if self.gui_blink_markers_list:
            # Extract marker times and convert to relative times
            marker_times = np.array([m[0] for m in self.gui_blink_markers_list])
            relative_marker_times = marker_times - timestamps[-1]
            
            # Get corresponding y values for vertical lines (from bottom to top of plot)
            ymin = np.full(len(relative_marker_times), -1500)  # Bottom of plot
            ymax = np.full(len(relative_marker_times), 1500)   # Top of plot
            
            # Update vertical lines
            self.blink_markers.set_segments([np.array([[x, y1], [x, y2]]) 
                                           for x, y1, y2 in zip(relative_marker_times, ymin, ymax)])
        else:
            # Clear markers by setting empty segments
            self.blink_markers.set_segments([])

        # Adjust axes and redraw
        self.blink_ax.set_xlim(-C.BLINK_WINDOW_SECONDS, 0)
        self.blink_canvas.draw_idle() 

    def update_focus_plot(self, timestamp, beta_metric, threshold_value):
         if not self.focus_canvas: return

         # Append data to deques
         self.focus_times.append(timestamp)
         self.focus_beta_values.append(beta_metric)
         self.focus_threshold_values.append(threshold_value)

         # Update plot lines
         self.focus_beta_line.set_data(list(self.focus_times), list(self.focus_beta_values))
         self.focus_thresh_line.set_data(list(self.focus_times), list(self.focus_threshold_values))

         # Adjust axes to keep a fixed window size of 100 seconds
         current_max_time = self.focus_times[-1] if self.focus_times else 0
         self.focus_ax.set_xlim(max(0, current_max_time - 100), current_max_time + 1)

         self.focus_canvas.draw_idle()

    

    # Action for the reset button
    def reset_focus_threshold_action(self):
        if self.focus_detector:
            print("GUI: Resetting focus calculation history in detector.")
            self.focus_detector.reset_threshold()

            print("GUI: Focus threshold reset signal sent to detector.")
        else:
             print("GUI: Cannot reset focus threshold - detector not available.")

    def update_detector_status(self, status):
        print(f"Detector Status: {status}")
        # Update the status label based on the detector status if detectors connected to muse
        if "Blink Detector: Connected" in status: self.blink_detector_connected = True
        elif "Focus Detector: Connected" in status: self.focus_detector_connected = True
        elif "stream not found" in status.lower() and ("Blink Detector" in status or "Focus Detector" in status):
            self.eeg_status_label.setText(f"Status: LSL Stream Not Found for Detectors!")
            self.connect_eeg_button.setText("LSL Failed")
            self.stop_eeg_detectors()
            return
        elif "Stopped." in status:
            if "Blink Detector" in status: self.blink_detector_connected = False
            if "Focus Detector" in status: self.focus_detector_connected = False

        if "Focus Detector" in status:
            if "Connected" in status:
                self.reset_focus_button.setEnabled(True)
                print("Reset Focus Button Enabled.") 
            elif "Stopped" in status or "stream not found" in status.lower() or "Error" in status:
                self.reset_focus_button.setEnabled(False)
                print("Reset Focus Button Disabled.") 
            
        blink_status = "OK" if self.blink_detector_connected else "Wait"
        focus_status = "OK" if self.focus_detector_connected else "Wait"
        if not self.blink_thread and not self.focus_thread:
            blink_status = "Off"
            focus_status = "Off"
        muse_stream_status_str = "" 
        if self.streaming_thread and self.streaming_thread.is_alive(): muse_stream_status_str = "Muse: Streaming | "
        elif self.scan_stream_button.text() == "Scanning...": muse_stream_status_str = "Muse: Scanning | "
        elif self.scan_stream_button.isEnabled(): muse_stream_status_str = "Muse: Off | "
        elif not self.scan_stream_button.isEnabled() and "Streaming" in self.scan_stream_button.text(): muse_stream_status_str = "Muse: Streaming | "
        self.eeg_status_label.setText(f"Status: {muse_stream_status_str}Detectors (Blink:{blink_status}, Focus:{focus_status})")
        
        if not self.focus_thread or not self.focus_thread.isRunning():
            self.reset_focus_button.setEnabled(False)

        self.update_bluetooth_button_state()
        if self.blink_detector_connected and self.focus_detector_connected: self.connect_eeg_button.setText("Detectors Connected")
        elif not self.blink_thread and not self.focus_thread: self.connect_eeg_button.setText("Connect to EEG Stream"); self.connect_eeg_button.setEnabled(True)


    def update_bluetooth_button_state(self):
        enable_bt = self.blink_detector_connected and self.focus_detector_connected
        current_bt_status = self.bluetooth_status_label.text().lower()
        already_connected = "connected!" in current_bt_status
        already_connecting = "connecting..." in current_bt_status or "connecting bt..." in current_bt_status
        self.connect_bluetooth_button.setEnabled(enable_bt and not already_connected and not already_connecting)

    def connect_bluetooth(self):
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
            return
        self.connect_bluetooth_button.setEnabled(False)
        self.connect_bluetooth_button.setText("Connecting BT...")
        self.bluetooth_status_label.setText("Bluetooth Status: Connecting...")
        if self.bluetooth_thread:
            self.bluetooth_thread.stop_thread()
            self.bluetooth_thread = None
        self.bluetooth_thread = BluetoothThread()
        self.bluetooth_thread.bluetooth_status_signal.connect(self.update_bluetooth_status)
        self.bluetooth_thread.connection_lost_signal.connect(self.handle_connection_lost)
        self.bluetooth_thread.start()

    def update_bluetooth_status(self, status):
        self.bluetooth_status_label.setText(f"Bluetooth Status: {status}")
        if "Connected!" in status:
            self.connect_bluetooth_button.setEnabled(False)
            self.connect_bluetooth_button.setText("Connected to BT")
            self.bluetooth_status_label.setStyleSheet("color: green;")
            QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))
        elif "Disconnected" in status or "Failed" in status or "Error" in status or "LOST" in status:
            self.connect_bluetooth_button.setText("Connect to Bluetooth Car")
            self.update_bluetooth_button_state()
            self.movement_display.setText("Stop")
            if "Failed" in status or "Error" in status or "LOST" in status:
                self.bluetooth_status_label.setStyleSheet("color: red;")
                QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))

    def handle_connection_lost(self):
        print("GUI: Bluetooth connection lost signal received.")
        self.bluetooth_status_label.setText("Bluetooth Status: CONNECTION LOST")
        self.bluetooth_status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_bluetooth_button.setText("Connect to Bluetooth Car")
        self.update_bluetooth_button_state()
        self.movement_display.setText("Stop")
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
            print("GUI: Stopping BT thread after connection loss signal.")
            self.bluetooth_thread.stop_thread()
        self.bluetooth_thread = None
    '''
    def update_movement(self, direction):
        print(f"GUI: Blink received: {direction}")
        if not self._check_bluetooth_connection():
            print("GUI: Bluetooth not connected, ignoring blink.")
            return
        command = None
        display_text = ""
        if direction == "left":
            display_text = "Moving Left"
            command = "MOVE_LEFT"
        elif direction == "right":
            display_text = "Moving Right"
            command = "MOVE_RIGHT"
        if command:
            self.movement_display.setText(display_text)
            self.direction_timer.start(500)
            self.bluetooth_thread.send_command(command)
    '''
    def update_movement(self, direction):
        print(f"GUI: Blink received: {direction}")
        if not self._check_bluetooth_connection():
            print("GUI: Bluetooth not connected, ignoring blink.")
            return

        if direction == "toggle_reverse":
            self.reverse_mode = not self.reverse_mode
            mode_text = "REVERSE" if self.reverse_mode else "FORWARD"
            self.direction_mode_label.setText(f"Direction: {mode_text}")
            print(f"GUI: Reverse mode toggled -> Now {mode_text}")
            # If moving, update move command to reflect direction
            if self.is_moving_forward:
                move_cmd = "MOVE_REVERSE" if self.reverse_mode else "MOVE_FORWARD"
                self.bluetooth_thread.send_command(move_cmd)
                self.movement_display.setText(f"Moving {mode_text}")
            return

        # Normal left/right blink control
        command = None
        display_text = ""
        if direction == "left":
            display_text = "Moving Left"
            command = "MOVE_LEFT"
        elif direction == "right":
            display_text = "Moving Right"
            command = "MOVE_RIGHT"

        if command:
            self.movement_display.setText(display_text)
            self.direction_timer.start(500)
            self.bluetooth_thread.send_command(command)

    def reset_direction_display(self):
        print("GUI: Resetting direction display.")
        if self._check_bluetooth_connection():
            command = None
            if self.is_moving_forward:
                self.movement_display.setText("Moving Forward")
                command = "MOVE_FORWARD"
            else:
                self.movement_display.setText("Stop")
                command = "STOP"
            if command:
                self.bluetooth_thread.send_command(command)
        else:
            self.movement_display.setText("Stop")

    def update_focus(self, focused):
        print(f"GUI: Focus state received: {focused}")
        self.is_moving_forward = focused
        if not self._check_bluetooth_connection():
            print("GUI: Bluetooth not connected, ignoring focus change.")
            self.movement_display.setText("Stop")
            return
        command = None
        #MODIFIED FOR REVERSE
        if not self.direction_timer.isActive():
            if focused:
                mode = "REVERSE" if self.reverse_mode else "FORWARD"
                self.movement_display.setText(f"Moving {mode}")
                command = "MOVE_REVERSE" if self.reverse_mode else "MOVE_FORWARD"
            else:
                self.movement_display.setText("Stop")
                command = "STOP"
            if command:
                self.bluetooth_thread.send_command(command)

    def _check_bluetooth_connection(self):
        return (self.bluetooth_thread is not None and self.bluetooth_thread.isRunning() and
                hasattr(self.bluetooth_thread, 'controller') and self.bluetooth_thread.controller.connected and
                self.bluetooth_thread.was_connected)

    def stop_eeg_detectors(self):
        print("GUI: Stopping EEG detector threads...")
        if self.blink_thread and self.blink_thread.isRunning():
            if self.blink_detector:
                self.blink_detector.stop()
            self.blink_thread.quit()
            self.blink_thread.wait(1000)
        if self.focus_thread and self.focus_thread.isRunning():
            if self.focus_detector:
                self.focus_detector.stop()
            self.focus_thread.quit()
            self.focus_thread.wait(1000)

    def on_detector_finished(self):
        sender_obj = self.sender()
        print(f"GUI: Detector object finished signal: {type(sender_obj).__name__}")

    def on_detector_thread_finished(self):
         sender_thread = self.sender()
         print(f"GUI: Detector thread finished: {sender_thread}")
         if sender_thread == self.blink_thread:
              print("Cleaning up blink detector resources.")
              self.blink_detector = None
              self.blink_thread = None
              self.blink_detector_connected = False
         elif sender_thread == self.focus_thread:
              print("Cleaning up focus detector resources.")
              self.focus_detector = None
              self.focus_thread = None
              self.focus_detector_connected = False
         self.update_detector_status("Detector Stopped.")
         self.update_bluetooth_button_state()
         if not self.blink_thread and not self.focus_thread:
              self.connect_eeg_button.setEnabled(True)
              self.connect_eeg_button.setText("Connect to EEG Stream")

    def closeEvent(self, event):
        print("GUI: Close event triggered. Cleaning up...")
        self.stop_eeg_detectors()
        QApplication.processEvents()
        self.msleep(100)
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
            self.bluetooth_thread.stop_thread()
        print("GUI: Cleanup finished.")
        event.accept()

    def msleep(self, msecs):
        loop = QEventLoop()
        QTimer.singleShot(msecs, loop.quit)
        loop.exec()


# Run Stream Function  
def run_stream_in_thread(address):
    if not stream: 
        print("muselsl stream function not available.")
        return
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    try:
        print(f"Starting Muse LSL stream for address: {address}...")
        stream(address)
        print("Muse LSL stream stopped.")
    except Exception as e:
        print(f"Error in streaming thread, details: {e}")
        import traceback 
        traceback.print_exc()


# Main Application Execution 
if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    window.show()

    exit_code = app.exec()
    sys.exit(exit_code)