import sys
import asyncio
import threading
import numpy as np
import os
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QFrame, QSizePolicy, QSlider, QCheckBox
from PyQt6.QtCore import QThread, pyqtSignal, QTimer, QEventLoop, Qt
from time import time

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
    from muselsl.muse import Muse
    from bleak.exc import BleakError
except ImportError:
    print("Warning: muselsl/BleakError not found.")
    stream, list_muses, Muse, BleakError = None, None, None, Exception


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

class BatteryCheckThread(QThread):
    """
    Thread to connect briefly to Muse, get battery level, and disconnect.
    """
    battery_level_signal = pyqtSignal(float)

    def __init__(self, muse_address, muse_name=None):
        super().__init__()
        self.muse_address = muse_address
        self.muse_name = muse_name
        self._battery_level = -1.0 # Default to error state
        self._received_telemetry = False
        self._lock = threading.Lock() # To safely update shared flags/vars if needed

    def _handle_telemetry_for_battery(self, timestamp, battery, fuel_gauge, adc_volt, temperature):
        """Callback for Muse telemetry, stores battery level and sets flag."""
        with self._lock:
            if not self._received_telemetry: # Process only the first telemetry packet
                print(f"Battery Check Thread: Received telemetry - Battery: {battery:.1f}%")
                self._battery_level = float(battery)
                self._received_telemetry = True

    def run(self):
        if not Muse:
            print("Battery Check Thread: muselsl.muse.Muse class not available.")
            self.battery_level_signal.emit(-1.0)
            return

        # --- Set up asyncio event loop for this thread ---
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        # --- End asyncio setup ---

        muse_instance = None
        connected = False
        try:
            print(f"Battery Check Thread: Connecting to {self.muse_name or self.muse_address} for battery...")
            # Instantiate Muse 
            muse_instance = Muse(address=self.muse_address,
                                 callback_telemetry=self._handle_telemetry_for_battery,
                                 # Disable other callbacks for efficiency
                                 callback_eeg=None, callback_acc=None, callback_gyro=None, callback_ppg=None,
                                 # Ensure backend selection doesn't override bleak if it's the default
                                 backend='bleak') # Explicitly use bleak if needed, or 'auto'

            connected = muse_instance.connect(retries=0) # Attempt connection once

            if connected:
                print("Battery Check Thread: Connected. Starting telemetry...")
                muse_instance.start() # Sends 'd' command, hopefully triggering telemetry

                # Wait for the callback to set the flag, with a timeout
                start_wait = time()
                timeout_seconds = 10 # Slightly longer timeout
                while True:
                    with self._lock:
                        if self._received_telemetry:
                            break # Exit loop once telemetry is received
                    if (time() - start_wait) > timeout_seconds:
                        print("Battery Check Thread: Timeout waiting for telemetry packet.")
                        self._battery_level = -1.0 # Ensure error state on timeout
                        break # Exit loop on timeout
                    self.msleep(200) # Short sleep within the thread

                if self._received_telemetry:
                    print(f"Battery Check Thread: Successfully received battery level: {self._battery_level:.1f}%")

                print("Battery Check Thread: Stopping stream...")
                muse_instance.stop()
                self.msleep(100) # Brief pause after stop

            else:
                print("Battery Check Thread: Failed to connect for battery check.")
                self._battery_level = -1.0

        except Exception as e:
            print(f"Battery Check Thread: Error during battery check: {e}")
            import traceback
            traceback.print_exc()
            self._battery_level = -1.0 # Set error state

        finally:
            # --- Cleanly disconnect ---
            if connected and muse_instance:
                 print("Battery Check Thread: Disconnecting...")
                 try:
                     muse_instance.disconnect()
                     print("Battery Check Thread: Disconnected.")
                 except Exception as e_disconnect:
                      print(f"Battery Check Thread: Error during disconnect: {e_disconnect}")
            elif muse_instance:
                # Attempt disconnect even if connect failed partially, Muse might hold adapter resources
                try:
                    muse_instance.disconnect() # Muse.disconnect should handle adapter cleanup
                    print("Battery Check Thread: Disconnected (after failed connect).")
                except Exception as e_disconnect_fail:
                    print(f"Battery Check Thread: Error during cleanup disconnect (after fail): {e_disconnect_fail}")


            # --- Close the asyncio loop for this thread ---
            if loop and not loop.is_closed():
                loop.close()
                print("Battery Check Thread: Closed asyncio loop.")
            # --- End asyncio cleanup ---

            print(f"Battery Check Thread: Emitting battery level: {self._battery_level}")
            self.battery_level_signal.emit(self._battery_level)
            print("Battery Check Thread: Finished.")

class EEGMonitorGUI(QWidget):
    _muse_scan_result_signal = pyqtSignal(object)
    # Signal from BatteryCheckThread
    _battery_level_signal = pyqtSignal(float)
    # Signal to communicate manual threshold to focus detector
    _manual_threshold_signal = pyqtSignal(float, bool)

    def __init__(self):
        super().__init__()
        # Detectors/Threads
        self.blink_detector = None
        self.focus_detector = None
        self.blink_thread = None
        self.focus_thread = None
        # Bluetooth
        self.bluetooth_thread = None
        # Muse Streaming and Battery
        self.scan_thread = None
        self.streaming_thread = None
        self.battery_thread = None
        self.current_muse_address = None # Store address for LSL stream start
        self.current_muse_name = None    # Store name
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
        # Connect battery signal to its handler
        self._battery_level_signal.connect(self.on_battery_level_received)

        # Manual threshold settings
        self.manual_threshold_enabled = False
        self.manual_threshold_value = 0.25

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
        status_layout = QHBoxLayout() # Layout for EEG and Battery status
        self.eeg_status_label = QLabel("Status: Ready")
        status_layout.addWidget(self.eeg_status_label)
        # --- Add Battery Status Label ---
        self.battery_status_label = QLabel("Battery: Unknown")
        self.battery_status_label.setStyleSheet("margin-left: 20px;") # Add some spacing
        status_layout.addWidget(self.battery_status_label)
        status_layout.addStretch() 
        layout.addLayout(status_layout)

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
        self.scan_stream_button.setEnabled(list_muses is not None and Muse is not None) # Also check Muse class
        if list_muses is None or Muse is None: self.scan_stream_button.setToolTip("muselsl library or Muse class not found")
        layout.addWidget(self.scan_stream_button)

        self.connect_eeg_button = QPushButton("Connect to EEG Stream")
        self.connect_eeg_button.clicked.connect(self.start_eeg_detectors)
        layout.addWidget(self.connect_eeg_button)

        self.connect_bluetooth_button = QPushButton("Connect to Bluetooth Car")
        self.connect_bluetooth_button.setEnabled(False)
        self.connect_bluetooth_button.clicked.connect(self.connect_bluetooth)
        layout.addWidget(self.connect_bluetooth_button)

        plots_layout = QVBoxLayout()
        left_right_layout = QHBoxLayout() #sss

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

        # Focus plot with threshold controls
        focus_section_layout = QHBoxLayout()
        
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
        focus_section_layout.addWidget(self.focus_canvas)
        
        # Add vertical threshold controls
        threshold_controls = QVBoxLayout()
        
        # Manual threshold checkbox at the top
        self.manual_threshold_checkbox = QCheckBox("Manual\nThreshold")
        self.manual_threshold_checkbox.setChecked(False)
        self.manual_threshold_checkbox.toggled.connect(self.toggle_manual_threshold)
        threshold_controls.addWidget(self.manual_threshold_checkbox)
        

        
        # Threshold slider in the middle - with fixed height
        self.threshold_slider = QSlider(Qt.Orientation.Vertical)
        self.threshold_slider.setMinimum(-50)
        self.threshold_slider.setMaximum(100)
        self.threshold_slider.setValue(int(self.manual_threshold_value*100)) 
        self.threshold_slider.setTickPosition(QSlider.TickPosition.TicksRight)
        self.threshold_slider.setTickInterval(10)
        self.threshold_slider.valueChanged.connect(self.update_manual_threshold)
        self.threshold_slider.setEnabled(False)  # Disabled until manual mode checked
        self.threshold_slider.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        threshold_controls.addWidget(self.threshold_slider)
        
        threshold_controls.addSpacing(50)
        
        # Threshold value label at the bottom
        self.threshold_value_label = QLabel(f"{self.manual_threshold_value:.2f}")
        self.threshold_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        threshold_controls.addWidget(self.threshold_value_label)
        
        focus_section_layout.addLayout(threshold_controls)
        left_right_layout.addLayout(focus_section_layout)

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

    
    def toggle_manual_threshold(self, checked):
        """Toggle between automatic and manual threshold modes"""
        # Store previous state to detect transitions
        was_manual = self.manual_threshold_enabled
        
        # Update the current state
        self.manual_threshold_enabled = checked
        self.threshold_slider.setEnabled(checked)
        
        # If we have a focus detector running, update its threshold mode
        if self.focus_detector and hasattr(self.focus_detector, 'set_manual_threshold'):
            self.focus_detector.set_manual_threshold(
                self.manual_threshold_value, self.manual_threshold_enabled)
            
            # If switching from manual to automatic, reset the threshold history
            if was_manual and not checked:
                print("GUI: Switching from manual to auto mode - resetting threshold history")
                self.focus_detector.reset_threshold()
            
        print(f"Manual threshold mode {'enabled' if checked else 'disabled'}, " 
              f"value: {self.manual_threshold_value:.2f}")

    def update_manual_threshold(self, value):
        """Update the manual threshold value based on slider position"""
        # Convert slider value (-50,-100) to threshold (-0.5,-1.0)
        self.manual_threshold_value = (value) / 100.0
        self.threshold_value_label.setText(f"{self.manual_threshold_value:.2f}")
        
        # Update the focus detector if it exists and is in manual mode
        if self.focus_detector and self.manual_threshold_enabled and hasattr(self.focus_detector, 'set_manual_threshold'):
            self.focus_detector.set_manual_threshold(
                self.manual_threshold_value, self.manual_threshold_enabled)
            
        print(f"Manual threshold updated to {self.manual_threshold_value:.2f}")

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
        if self.battery_thread and self.battery_thread.isRunning():
            print("Battery check already in progress.")
            return

        self.scan_stream_button.setEnabled(False)
        self.scan_stream_button.setText("Scanning...")
        self.eeg_status_label.setText("Status: Scanning for Muse...")
        # Reset battery label
        self.battery_status_label.setText("Battery: Unknown")

        # Start the scan in a standard thread
        self.scan_thread = threading.Thread(target=self._perform_scan_and_emit, daemon=True)
        self.scan_thread.start()


    # Connected to _muse_scan_result_signal and runs on the GUI thread
    def on_muse_scan_complete(self, muses):
        """ Handles the result of the Muse scan received via signal """
        print(f"GUI Thread: Scan complete signal received. Muses: {muses}")
        # Button text will be updated after battery check or if scan fails

        if muses:
            selected_muse = muses[0]
            muse_address = selected_muse.get('address')
            muse_name = selected_muse.get('name', 'Unknown Muse')
            if not muse_address:
                 print("Error: Found Muse but address is missing.")
                 self.eeg_status_label.setText("Status: Found Muse, but address missing.")
                 self.scan_stream_button.setText("Scan and Start Muse Stream")
                 self.scan_stream_button.setEnabled(True)
                 return

            # Store address and name for later use by battery check handler
            self.current_muse_address = muse_address
            self.current_muse_name = muse_name

            # --- Start Battery Check ---
            self.eeg_status_label.setText(f"Status: Found {muse_name}. Checking battery...")
            print(f"Attempting battery check for {muse_name} ({muse_address})")
            self.battery_status_label.setText("Battery: Checking...")

            if self.battery_thread and self.battery_thread.isRunning():
                print("Warning: Battery thread already running? Stopping previous.")
                self.battery_thread.quit() # Request quit
                self.battery_thread.wait(1000)

            self.battery_thread = BatteryCheckThread(muse_address, muse_name)
            # Connect the thread's signal to the GUI's slot
            self.battery_thread.battery_level_signal.connect(self._battery_level_signal)
            self.battery_thread.start()
            # --- Do NOT start LSL stream here yet ---

        else: # Scan failed
            error_text = "Muse scan failed: Check if Muse is ON or connected elsewhere."
            self.eeg_status_label.setText(f"Status: {error_text}")
            self.battery_status_label.setText("Battery: N/A")
            self.scan_stream_button.setText("Scan and Start Muse Stream")
            self.scan_stream_button.setEnabled(True)

        self.scan_thread = None # Clear scan thread reference

    # --- New Slot to Handle Battery Level Result ---
    def on_battery_level_received(self, battery_level):
        """Handles the battery level received from the BatteryCheckThread."""
        print(f"GUI Thread: Battery level signal received: {battery_level}")

        if battery_level >= 0:
            self.battery_status_label.setText(f"Battery: {battery_level:.0f}%")
        else:
            self.battery_status_label.setText("Battery: N/A")

        # Clean up battery thread reference
        self.battery_thread = None

        # --- Now Proceed to Start LSL Stream ---
        if self.current_muse_address:
            self.eeg_status_label.setText(f"Status: Starting LSL stream for {self.current_muse_name}...")
            print(f"Attempting to start LSL stream for {self.current_muse_name} ({self.current_muse_address})")
            if stream:
                 # Ensure previous streaming thread is cleaned up if necessary
                 if self.streaming_thread and self.streaming_thread.is_alive():
                     print("Warning: LSL Streaming thread was still alive? May cause issues.")
                     # Attempt to join? Or just overwrite? Overwriting is simpler for now.

                 self.streaming_thread = threading.Thread(
                     target=run_stream_in_thread, args=(self.current_muse_address,), daemon=True)
                 self.streaming_thread.start()
                 QTimer.singleShot(1000, self.check_stream_started) # Check if it actually started
                 self.scan_stream_button.setText("Muse Streaming...") # Keep button disabled
                 self.scan_stream_button.setEnabled(False)
            else:
                 self.eeg_status_label.setText("Status: Muse Found, Battery Checked, but 'stream' function missing.")
                 self.scan_stream_button.setText("Scan and Start Muse Stream")
                 self.scan_stream_button.setEnabled(True)
                 # Reset stored address if stream can't start
                 self.current_muse_address = None
                 self.current_muse_name = None

        else:
            # This case should ideally not happen if scan succeeded, but handle it
            print("Error: Battery level received, but no Muse address stored.")
            self.eeg_status_label.setText("Status: Error after battery check (no address).")
            self.scan_stream_button.setText("Scan and Start Muse Stream")
            self.scan_stream_button.setEnabled(True)


    def check_stream_started(self):
        if self.streaming_thread and not self.streaming_thread.is_alive():
             print("Error: Muse streaming thread exited unexpectedly after start attempt.")
             self.eeg_status_label.setText("Status: Failed to start Muse LSL stream.")
             self.scan_stream_button.setText("Scan and Start Muse Stream")
             self.scan_stream_button.setEnabled(True)
             self.streaming_thread = None
             self.current_muse_address = None # Reset address if stream failed
             self.current_muse_name = None
        elif self.streaming_thread and self.streaming_thread.is_alive():
             print("Muse streaming thread appears active.")
             self.eeg_status_label.setText("Status: Muse LSL Stream Active.")
             # Button remains disabled and text "Muse Streaming..."


    def start_eeg_detectors(self):
        if self.blink_thread or self.focus_thread:
            print("GUI: Detectors already running or starting.")
            return
        self.connect_eeg_button.setEnabled(False)
        self.connect_eeg_button.setText("Connecting LSL...")
        self.eeg_status_label.setText("Status: Initializing Detectors...") # Keep existing status logic mostly
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
        
        # Set manual threshold if enabled before starting detector
        if self.manual_threshold_enabled and hasattr(self.focus_detector, 'set_manual_threshold'):
            self.focus_detector.set_manual_threshold(
                self.manual_threshold_value, self.manual_threshold_enabled)
            
        self.focus_thread.start()

    # Plot Update
    def update_blink_plot(self, timestamps, channel_data_dict, new_blink_markers):
        if not self.blink_canvas or not hasattr(timestamps, 'any') or not timestamps.any(): return # Added canvas check and better timestamp check

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
            ymin, ymax = self.blink_ax.get_ylim() # Use current ylim

            # Update vertical lines
            segments = [np.array([[x, ymin], [x, ymax]]) for x in relative_marker_times if -C.BLINK_WINDOW_SECONDS <= x <= 0]
            self.blink_markers.set_segments(segments)

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
            # Modify EEG status only if LSL stream is *supposed* to be running
            if self.streaming_thread and self.streaming_thread.is_alive():
                 self.eeg_status_label.setText(f"Status: LSL Stream Found, but Detectors Failed to Connect!")
            else:
                 self.eeg_status_label.setText(f"Status: LSL Stream Not Found for Detectors!")
            self.connect_eeg_button.setText("LSL Failed")
            self.stop_eeg_detectors() # Stop detectors if stream fails
            return # Early exit
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

        # --- Simplified EEG Status Logic ---
        current_eeg_status = ""
        if self.streaming_thread and self.streaming_thread.is_alive():
            current_eeg_status = "Muse: Streaming | "
            # Update button state if stream is running but button text is wrong
            if self.scan_stream_button.isEnabled():
                self.scan_stream_button.setEnabled(False)
                self.scan_stream_button.setText("Muse Streaming...")
        elif self.scan_stream_button.text() == "Scanning..." or self.battery_status_label.text() == "Battery: Checking...":
             current_eeg_status = "Muse: Initializing | "
             if self.scan_stream_button.isEnabled(): self.scan_stream_button.setEnabled(False)
        elif self.scan_stream_button.isEnabled(): # Stream off, button enabled
             current_eeg_status = "Muse: Off | "
        elif not self.scan_stream_button.isEnabled() and "Streaming" in self.scan_stream_button.text():
             current_eeg_status = "Muse: Streaming | " # Catch state during init
        elif not self.scan_stream_button.isEnabled() and "Scan" in self.scan_stream_button.text():
             current_eeg_status = "Muse: Failed/Off | " # Catch failed state
             self.scan_stream_button.setEnabled(True) # Re-enable if failed

        # Detector Status Part
        blink_status = "OK" if self.blink_detector_connected else ("Wait" if self.blink_thread and self.blink_thread.isRunning() else "Off")
        focus_status = "OK" if self.focus_detector_connected else ("Wait" if self.focus_thread and self.focus_thread.isRunning() else "Off")

        self.eeg_status_label.setText(f"Status: {current_eeg_status}Detectors (Blink:{blink_status}, Focus:{focus_status})")

        # Update buttons based on combined state
        if self.blink_detector_connected and self.focus_detector_connected:
            self.connect_eeg_button.setText("Detectors Connected")
            self.connect_eeg_button.setEnabled(False) # Keep disabled when connected
        elif not self.blink_thread and not self.focus_thread: # Both threads stopped/off
            self.connect_eeg_button.setText("Connect to EEG Stream")
            # Enable ONLY if LSL stream is running
            self.connect_eeg_button.setEnabled(self.streaming_thread is not None and self.streaming_thread.is_alive())
        else: # One or both starting/running but not fully connected yet
             self.connect_eeg_button.setText("Connecting LSL...")
             self.connect_eeg_button.setEnabled(False)

        if not self.focus_thread or not self.focus_thread.isRunning():
            self.reset_focus_button.setEnabled(False)

        self.update_bluetooth_button_state()


    def update_bluetooth_button_state(self):
        enable_bt = self.blink_detector_connected and self.focus_detector_connected
        current_bt_status = self.bluetooth_status_label.text().lower()
        already_connected = "connected!" in current_bt_status or "connected to bt" in current_bt_status
        already_connecting = "connecting..." in current_bt_status or "connecting bt..." in current_bt_status

        # Only enable if detectors are connected AND BT is not already connected/connecting
        self.connect_bluetooth_button.setEnabled(enable_bt and not already_connected and not already_connecting)


    def connect_bluetooth(self):
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
            print("GUI: Bluetooth thread already running.")
            return
        self.connect_bluetooth_button.setEnabled(False)
        self.connect_bluetooth_button.setText("Connecting BT...")
        self.bluetooth_status_label.setText("Bluetooth Status: Connecting...")
        if self.bluetooth_thread: # Clean up previous thread if exists
            print("GUI: Stopping previous Bluetooth thread.")
            self.bluetooth_thread.stop_thread()
            self.bluetooth_thread = None
            self.msleep(100) 

        self.bluetooth_thread = BluetoothThread()
        self.bluetooth_thread.bluetooth_status_signal.connect(self.update_bluetooth_status)
        self.bluetooth_thread.connection_lost_signal.connect(self.handle_connection_lost)
        # Make sure the thread cleans itself up if it finishes unexpectedly
        self.bluetooth_thread.finished.connect(self.on_bluetooth_thread_finished)
        self.bluetooth_thread.start()

    def on_bluetooth_thread_finished(self):
        print("GUI: Bluetooth thread finished signal received.")
        # Check if the status isn't already reflecting a disconnect/error
        current_status = self.bluetooth_status_label.text()
        if "Connected" in current_status and not "Error" in current_status and not "Disconnected" in current_status:
             # If thread finished but status was 'Connected', assume it lost connection
             self.handle_connection_lost()
        self.bluetooth_thread = None # Clear the reference

    def update_bluetooth_status(self, status):
        self.bluetooth_status_label.setText(f"Bluetooth Status: {status}")
        if "Connected!" in status:
            self.connect_bluetooth_button.setEnabled(False)
            self.connect_bluetooth_button.setText("Connected to BT")
            self.bluetooth_status_label.setStyleSheet("color: green;")
            QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))
        elif "Disconnected" in status or "Failed" in status or "Error" in status or "LOST" in status:
            self.connect_bluetooth_button.setText("Connect to Bluetooth Car")
            # Re-check if detectors are still running before enabling
            self.update_bluetooth_button_state()
            self.movement_display.setText("Stop")
            if "Failed" in status or "Error" in status or "LOST" in status:
                self.bluetooth_status_label.setStyleSheet("color: red;")
                QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))
        # Update state based on the new status immediately
        self.update_bluetooth_button_state()


    def handle_connection_lost(self):
        print("GUI: Bluetooth connection lost signal received.")
        self.bluetooth_status_label.setText("Bluetooth Status: CONNECTION LOST")
        self.bluetooth_status_label.setStyleSheet("color: red; font-weight: bold;")
        self.connect_bluetooth_button.setText("Connect to Bluetooth Car")
        self.update_bluetooth_button_state() # Re-evaluate enabling BT button
        self.movement_display.setText("Stop")
        self.is_moving_forward = False # Ensure movement state is reset

        # Ensure the thread object is cleaned up
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
             print("GUI: Stopping BT thread after connection loss signal.")
             # Disconnect might fail if already lost, but try stopping the loop/thread
             self.bluetooth_thread.stop_thread()
        self.bluetooth_thread = None


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
            if self.is_moving_forward: # Check if supposed to be moving based on focus
                move_cmd = "MOVE_REVERSE" if self.reverse_mode else "MOVE_FORWARD"
                self.bluetooth_thread.send_command(move_cmd)
                # Update display only if not currently turning (timer inactive)
                if not self.direction_timer.isActive():
                     self.movement_display.setText(f"Moving {mode_text}")
            return

        # Normal left/right blink control
        command = None
        display_text = ""
        if direction == "left":
            display_text = "Turning Left"
            command = "MOVE_LEFT"
        elif direction == "right":
            display_text = "Turning Right"
            command = "MOVE_RIGHT"

        if command:
            self.movement_display.setText(display_text)
            self.direction_timer.start(500)
            self.bluetooth_thread.send_command(command)

    def reset_direction_display(self):
        print("GUI: Resetting direction display after turn.")
        # Display should reflect focus state and direction mode
        if self._check_bluetooth_connection():
            command = None
            if self.is_moving_forward:
                 mode_text = "REVERSE" if self.reverse_mode else "FORWARD"
                 self.movement_display.setText(f"Moving {mode_text}")
                 command = "MOVE_REVERSE" if self.reverse_mode else "MOVE_FORWARD"
            else:
                self.movement_display.setText("Stop")
                command = "STOP"

            if command:
                # Send the command to ensure car state matches display
                self.bluetooth_thread.send_command(command)
        else:
            # If BT disconnected while turning, just show stop
            self.movement_display.setText("Stop")


    def update_focus(self, focused):
        print(f"GUI: Focus state received: {focused}")
        focus_changed = self.is_moving_forward != focused
        self.is_moving_forward = focused

        if not self._check_bluetooth_connection():
            print("GUI: Bluetooth not connected, ignoring focus change.")
            # Update display even if not connected
            if not self.direction_timer.isActive(): # Don't override turning display
                 self.movement_display.setText("Stop")
            return

        command = None
        # Only send command if focus state actually changed OR if the turn timer just expired
        # The direction_timer.isActive() check prevents sending move/stop while turning display is active
        if focus_changed and not self.direction_timer.isActive():
            if focused:
                mode = "REVERSE" if self.reverse_mode else "FORWARD"
                self.movement_display.setText(f"Moving {mode}")
                command = "MOVE_REVERSE" if self.reverse_mode else "MOVE_FORWARD"
            else:
                self.movement_display.setText("Stop")
                command = "STOP"

            if command:
                print(f"GUI: Sending focus-based command: {command}")
                self.bluetooth_thread.send_command(command)
        elif not self.direction_timer.isActive():
             # If focus didn't change, but maybe timer expired, ensure display matches state
             if self.is_moving_forward:
                  mode = "REVERSE" if self.reverse_mode else "FORWARD"
                  self.movement_display.setText(f"Moving {mode}")
             else:
                  self.movement_display.setText("Stop")



    def _check_bluetooth_connection(self):
        # More robust check
        return (self.bluetooth_thread is not None and
                self.bluetooth_thread.isRunning() and
                hasattr(self.bluetooth_thread, 'controller') and
                self.bluetooth_thread.controller is not None and # Check controller exists
                self.bluetooth_thread.controller.connected and
                self.bluetooth_thread.was_connected)


    def stop_eeg_detectors(self):
        print("GUI: Stopping EEG detector threads...")
        if self.blink_thread and self.blink_thread.isRunning():
            if self.blink_detector:
                self.blink_detector.stop() # Signal detector loop to stop
            self.blink_thread.quit()
            if not self.blink_thread.wait(1500): # Increased wait time
                print("Warning: Blink thread did not finish quitting gracefully.")
                # self.blink_thread.terminate() # Use terminate as last resort if needed

        if self.focus_thread and self.focus_thread.isRunning():
            if self.focus_detector:
                self.focus_detector.stop() # Signal detector loop to stop
            self.focus_thread.quit()
            if not self.focus_thread.wait(1500): # Increased wait time
                print("Warning: Focus thread did not finish quitting gracefully.")



    def on_detector_finished(self):
        sender_obj = self.sender()
        if sender_obj: # Check if sender exists
            print(f"GUI: Detector object finished signal: {type(sender_obj).__name__}")

    def on_detector_thread_finished(self):
         sender_thread = self.sender()
         if not sender_thread: return # Should not happen, but check

         print(f"GUI: Detector thread finished: {sender_thread}")
         if sender_thread == self.blink_thread:
              print("Cleaning up blink detector resources.")
              self.blink_detector = None # Allow garbage collection
              self.blink_thread = None
              self.blink_detector_connected = False
              status_update = "Blink Detector Stopped."
         elif sender_thread == self.focus_thread:
              print("Cleaning up focus detector resources.")
              self.focus_detector = None # Allow garbage collection
              self.focus_thread = None
              self.focus_detector_connected = False
              self.reset_focus_button.setEnabled(False) # Ensure reset button is disabled
              status_update = "Focus Detector Stopped."
         else:
              status_update = "Unknown Detector Stopped."

         # Update status only after cleaning up references
         self.update_detector_status(status_update)
         # Re-evaluate button states now that a thread has finished
         self.update_bluetooth_button_state()
         # If *both* threads are now None, re-enable the connect button (if LSL stream active)
         if not self.blink_thread and not self.focus_thread:
              print("GUI: Both detector threads finished.")
              self.connect_eeg_button.setText("Connect to EEG Stream")
              self.connect_eeg_button.setEnabled(self.streaming_thread is not None and self.streaming_thread.is_alive())


    def closeEvent(self, event):
        print("GUI: Close event triggered. Cleaning up...")
        self.stop_eeg_detectors() # Stop detectors first

        # Stop battery thread if running
        if self.battery_thread and self.battery_thread.isRunning():
             print("GUI: Stopping battery check thread...")
             self.battery_thread.quit()
             self.battery_thread.wait(1000)

        # Stop Bluetooth thread
        if self.bluetooth_thread and self.bluetooth_thread.isRunning():
            print("GUI: Stopping Bluetooth thread...")
            self.bluetooth_thread.stop_thread() # Already waits internally

        if self.streaming_thread and self.streaming_thread.is_alive():
             print("GUI: LSL streaming thread is running (daemon). Will exit with app.")
             # We can't cleanly stop it from here without modifying muselsl or using multiprocessing

        QApplication.processEvents() # Process any pending events
        self.msleep(100)

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

    # --- Set up asyncio event loop for this thread ---
    loop = None
    try:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        print(f"Streaming Thread: Set up asyncio loop for thread {threading.current_thread().name}")
    except Exception as e:
        print(f"Streaming Thread: Failed to set up asyncio loop: {e}")
        # If loop setup fails, bleak backend won't work, so exit.
        return
    # --- End asyncio setup ---

    try:
        print(f"Streaming Thread: Starting Muse LSL stream for address: {address}...")
        # This call blocks until the stream stops or errors.
        # It will use the asyncio loop we just set for Bleak operations.
        stream(address, backend='bleak') # Explicitly use bleak or 'auto'
        print("Streaming Thread: Muse LSL stream stopped normally.")
    except RuntimeError as e:
        # Catch the specific "no event loop" error here if setup failed, though unlikely now
        print(f"Streaming Thread: Runtime error during stream: {e}")
        import traceback
        traceback.print_exc()
    except Exception as e:
        print(f"Streaming Thread: Error in LSL stream function, details: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- Clean up the asyncio loop for this thread ---
        if loop and not loop.is_closed():
            try:
                # Stop the loop if it's running (might not be if stream errored out early)
                if loop.is_running():
                    loop.call_soon_threadsafe(loop.stop)
                    pass # Avoid sleep here, just close
                loop.close()
                print(f"Streaming Thread: Closed asyncio loop for thread {threading.current_thread().name}")
            except Exception as loop_close_e:
                print(f"Streaming Thread: Error closing asyncio loop: {loop_close_e}")
        # --- End asyncio cleanup ---
        print(f"Streaming Thread: Exiting thread {threading.current_thread().name}.")


# Main Application Execution
if __name__ == "__main__":

    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    window.show()

    exit_code = app.exec()
    print(f"Application exiting with code: {exit_code}")
    sys.exit(exit_code)