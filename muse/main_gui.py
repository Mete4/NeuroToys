import sys
import asyncio
import subprocess
import os
import threading
import argparse  # Added import for command-line argument parsing
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QFrame
from PyQt6.QtCore import QThread, pyqtSignal, QTimer  # Added QTimer import

sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Bluetooth/bluetooth_setup'))
from bluetooth import BluetoothController
from muselsl import stream, list_muses

from signals import PlotSignals

class BluetoothThread(QThread):
    """Handles asynchronous Bluetooth connection and command sending."""
    bluetooth_status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.controller = BluetoothController()

    def run(self):
        asyncio.run(self.connect_bluetooth())

    async def connect_bluetooth(self):
        """Connects to the Bluetooth device."""
        self.bluetooth_status_signal.emit("Connecting to Bluetooth...")
        success = await self.controller.connect("ESP_CAR")
        if success:
            self.bluetooth_status_signal.emit("Bluetooth Connected!")
        else:
            self.bluetooth_status_signal.emit("Bluetooth Connection Failed.")

    def send_command(self, command):
        """Send movement commands via Bluetooth."""
        if self.controller.connected:
            asyncio.run(self.controller.send_command(command))

    def disconnect_bluetooth(self):
        """Disconnect from the Bluetooth device."""
        if self.controller.connected:
            asyncio.run(self.controller.disconnect())
            self.bluetooth_status_signal.emit("Bluetooth Disconnected.")

class SignalListenerThread(QThread):
    """Listens for signals from plot scripts and emits them to the main GUI."""
    blink_signal = pyqtSignal(str)
    focus_signal = pyqtSignal(bool)
    status_signal = pyqtSignal(str)  # Add status signal
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.server = PlotSignals(is_server=True)
        
    def run(self):
        self.running = True
        
        # Register callbacks for different message types
        self.server.register_callback("blink", lambda direction: self.blink_signal.emit(direction))
        self.server.register_callback("focus", lambda is_focused: self.focus_signal.emit(is_focused))
        self.server.register_callback("status", lambda status: self.status_signal.emit(status))
        
        # Start the server
        self.server.start_server()
        
        # Keep the thread running until stopped
        while self.running:
            # Small sleep to prevent CPU hogging
            self.msleep(100)
        
    def stop(self):
        """Stop the signal listener thread"""
        self.running = False
        self.server.stop()

class EEGThread(QThread):
    """Handles EEG stream processing and launches visualization plots."""
    blink_signal = pyqtSignal(str)
    focus_signal = pyqtSignal(bool)
    eeg_status_signal = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.processes = []
        
    def run(self):
        """Start EEG monitoring and launch visualization plots."""
        self.eeg_status_signal.emit("Launching EEG plots...")
        
        # Get the current script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        python_exe = sys.executable
        
        try:
            # Create creation flags based on platform
            if sys.platform == 'win32':
                creation_flags = subprocess.CREATE_NEW_CONSOLE
            else:
                creation_flags = 0
                
            # Launch blink_plot.py
            blink_process = subprocess.Popen([
                python_exe, 
                os.path.join(script_dir, "blink_plot.py")
            ], creationflags=creation_flags if sys.platform == 'win32' else 0)
            self.processes.append(blink_process)
            
            # Launch focus_plot.py
            focus_process = subprocess.Popen([
                python_exe, 
                os.path.join(script_dir, "focus_plot.py")
            ], creationflags=creation_flags if sys.platform == 'win32' else 0)
            self.processes.append(focus_process)
            
            self.eeg_status_signal.emit("Waiting for EEG headset connection...")
            
            # Keep the thread running to monitor subprocess status
            while all(p.poll() is None for p in self.processes):
                # Check if any process has terminated unexpectedly
                self.msleep(1000)
            
            # If any process died, report it
            for proc in self.processes:
                if proc.poll() is not None:
                    self.eeg_status_signal.emit(f"EEG plot process exited with code {proc.returncode}")
                    
        except Exception as e:
            self.eeg_status_signal.emit(f"Error launching plots: {str(e)}")
        finally:
            self.eeg_status_signal.emit("EEG connected and plots running")
    
    def __del__(self):
        """Cleanup plot processes on exit."""
        self.terminate_processes()
    
    def terminate_processes(self):
        """Terminate all child processes."""
        for process in self.processes:
            try:
                if process.poll() is None:  # If process is still running
                    process.terminate()
                    process.wait(timeout=1)
            except:
                pass

class EEGMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.bluetooth_thread = BluetoothThread()  # Bluetooth handler
        self.signal_thread = None
        self.eeg_thread = None
        self.direction_timer = QTimer()  # Create a timer for direction display
        self.direction_timer.timeout.connect(self.reset_direction_display)
        self.direction_timer.setSingleShot(True)  # Timer will only fire once

    def initUI(self):
        layout = QVBoxLayout()

        self.eeg_status_label = QLabel("EEG Status: Not Connected")
        layout.addWidget(self.eeg_status_label)

        self.bluetooth_status_label = QLabel("Bluetooth Status: Not Connected")
        layout.addWidget(self.bluetooth_status_label)

        self.movement_display = QLabel("")  # Starts blank
        self.movement_display.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.movement_display.setStyleSheet("font-size: 32px; text-align: center;")
        layout.addWidget(self.movement_display)

        self.connect_eeg_button = QPushButton("Connect to EEG Stream")
        self.connect_eeg_button.clicked.connect(self.start_eeg_stream)
        layout.addWidget(self.connect_eeg_button)

        self.connect_bluetooth_button = QPushButton("Connect to Bluetooth")
        self.connect_bluetooth_button.setEnabled(False)  # Disabled until EEG connects
        self.connect_bluetooth_button.clicked.connect(self.connect_bluetooth)
        layout.addWidget(self.connect_bluetooth_button)

        self.setLayout(layout)
        self.setWindowTitle("EEG Car Monitor")
        self.resize(400, 300)

    def start_eeg_stream(self):
        """Starts the EEG stream and initializes monitoring."""
        self.connect_eeg_button.setEnabled(False)
        self.eeg_status_label.setText("EEG Status: Starting...")
        
        # Start the signal listener thread first
        self.signal_thread = SignalListenerThread()
        self.signal_thread.blink_signal.connect(self.update_movement)
        self.signal_thread.focus_signal.connect(self.update_focus)
        self.signal_thread.status_signal.connect(self.update_eeg_status)  # Connect status signal
        self.signal_thread.start()
        
        # Then start the EEG thread to launch the plot scripts
        self.eeg_thread = EEGThread()
        self.eeg_thread.eeg_status_signal.connect(self.update_eeg_status)
        self.eeg_thread.start()

    def connect_bluetooth(self):
        """Starts the Bluetooth connection."""
        self.bluetooth_thread.bluetooth_status_signal.connect(self.update_bluetooth_status)
        self.bluetooth_thread.start()

    def update_movement(self, direction):
        """Updates movement display based on detected blink direction and sends Bluetooth command."""
        current_display = self.movement_display.text()
        print("Blink Direction:", direction)
        
        # Store whether we're moving forward
        is_moving_forward = "Moving Forward" in current_display
        
        if direction == "left":
            self.movement_display.setText("Moving Left")
            self.bluetooth_thread.send_command("MOVE_LEFT")
            # Start the timer to reset the display after 0.5 seconds
            self.direction_timer.start(500)  # 500 ms = 0.5 seconds
        elif direction == "right":
            self.movement_display.setText("Moving Right")
            self.bluetooth_thread.send_command("MOVE_RIGHT")
            # Start the timer to reset the display after 0.5 seconds
            self.direction_timer.start(500)  # 500 ms = 0.5 seconds
    
    def reset_direction_display(self):
        """Reset the direction display after the timer expires"""
        # If we were previously moving forward, restore that state
        if "Moving Left" in self.movement_display.text() or "Moving Right" in self.movement_display.text():
            if self.direction_timer.property("was_moving_forward"):
                self.movement_display.setText("Moving Forward")
                self.bluetooth_thread.send_command("MOVE_FORWARD")
            else:
                self.movement_display.setText("Stop")  # Clear the display

    def update_focus(self, focused):
        """Updates movement display based on focus detection and sends Bluetooth command."""
        print("Focused:", focused)
        current_display = self.movement_display.text()
        
        # If showing a temporary direction message, store whether we should 
        # return to "Moving Forward" when the timer expires
        if "Moving Left" in current_display or "Moving Right" in current_display:
            self.direction_timer.setProperty("was_moving_forward", focused)
        
        if focused:
            if "Moving Forward" not in current_display and "Moving Left" not in current_display and "Moving Right" not in current_display:
                self.movement_display.setText("Moving Forward")
            self.bluetooth_thread.send_command("MOVE_FORWARD")
        else:
            current_display = self.movement_display.text()
            if "Moving Left" not in current_display and "Moving Right" not in current_display:
                self.movement_display.setText(current_display.replace("Moving Forward", "Stop").strip())
            self.bluetooth_thread.send_command("STOP")

    def update_eeg_status(self, status):
        """Updates EEG status and enables Bluetooth + plot buttons when connected."""
        self.eeg_status_label.setText(f"EEG Status: {status}")
        
        # Check if EEG stream was not found and exit program if so
        if "EEG stream not found!" in status:
            print("EEG stream not found. Closing application...")
            QTimer.singleShot(2000, lambda: QApplication.quit())  # Wait 2 seconds before quitting
            return
        
        # Enable Bluetooth button if both plots are connected
        if ("Focus plot: Connected" in status or "Blink plot: Connected" in status):
            self.connect_bluetooth_button.setEnabled(True)

    def update_bluetooth_status(self, status):
        """Updates Bluetooth connection status."""
        self.bluetooth_status_label.setText(f"Bluetooth Status: {status}")
        
    def closeEvent(self, event):
        """Clean up resources when the application is closed."""
        # Stop all threads before closing
        if self.signal_thread:
            self.signal_thread.stop()
            self.signal_thread.wait()
        
        if self.eeg_thread:
            self.eeg_thread.terminate_processes()
            self.eeg_thread.wait()
            
        if self.bluetooth_thread:
            self.bluetooth_thread.disconnect_bluetooth()
            
        event.accept()

def run_stream_in_thread(address):
    """Run stream in a thread with its own event loop"""
    import asyncio
    from muselsl import stream
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        stream(address)
    except Exception as e:
        print(f"Error in streaming thread: {e}")

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='EEG Car Monitor')
    parser.add_argument('--stream', action='store_true', help='Start streaming EEG data from Muse')
    args = parser.parse_args()
    
    if args.stream:
        # Find available Muse devices
        muses = list_muses()
        
        if not muses:
            print("No Muse devices found.")
            print("Continuing without streaming...")
        else:
            # Connect to the first available device
            print(f"Connecting to {muses[0]['name']} ({muses[0]['address']})...")
            print("Press Ctrl+C to disconnect.")
            
            # Start streaming in a thread with proper event loop setup
            streaming_thread = threading.Thread(target=run_stream_in_thread, args=(muses[0]['address'],))
            streaming_thread.daemon = True
            streaming_thread.start()
    else:
        print("Running without direct connection to Muse headset.")
        print("Use --stream parameter to connect and stream from Muse.")
    
    # Continue with the GUI application
    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    window.show()
    sys.exit(app.exec())