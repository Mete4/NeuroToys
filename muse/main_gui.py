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
    connection_lost_signal = pyqtSignal()  

    def __init__(self):
        super().__init__()
        self.controller = BluetoothController()
        self.connection_check_timer = None
        self.loop = None  # Store the event loop to handle it properly
        
    def run(self):
        try:
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)
            
            # Run the connection logic
            self.loop.run_until_complete(self.connect_bluetooth())
            
            # Start a timer to check connection status every 2 seconds
            self.connection_check_timer = QTimer()
            self.connection_check_timer.timeout.connect(self.check_connection_status)
            self.connection_check_timer.start(2000)  
            
            # Keep the thread running without blocking the event loop
            while self.connection_check_timer and self.connection_check_timer.isActive():
                try:
                    self.loop.run_until_complete(asyncio.sleep(0.1))
                except (RuntimeError, asyncio.CancelledError):
                    break
                self.msleep(200)  
                
        except Exception as e:
            print(f"Error in Bluetooth thread: {e}")
            self.bluetooth_status_signal.emit(f"Bluetooth Error: {str(e)}")
        finally:
            # Clean up the event loop
            if self.loop and not self.loop.is_closed():
                self.loop.close()
            
    def check_connection_status(self):
        """Check if we're still connected and update status if not"""
        try:
            if not self.controller.connected and hasattr(self, 'was_connected') and self.was_connected:
                self.was_connected = False
                self.bluetooth_status_signal.emit("Bluetooth Disconnected")
                self.connection_lost_signal.emit()
        except Exception as e:
            print(f"Error checking connection status: {e}")

    async def connect_bluetooth(self):
        print("BluetoothThread: Coroutine started. Attempting connection...") 
        try:
            print("BluetoothThread: Calling controller.connect...") 
            connected = await self.controller.connect("ESP_CAR")
            print(f"BluetoothThread: controller.connect returned: {connected}") 

            if connected:
                print("BluetoothThread: Connection successful. Setting was_connected=True.") 
                self.was_connected = True 
                print("BluetoothThread: Emitting 'Bluetooth Connected!' signal...") 
                self.bluetooth_status_signal.emit("Bluetooth Connected!")
                print("BluetoothThread: 'Bluetooth Connected!' signal emitted.") 
                print("BluetoothThread: Starting connection check timer...") 
                self.connection_check_timer = self.loop.call_later(1, self.check_connection_status)
                print("BluetoothThread: Connection check timer started.") 
            else:
                print("BluetoothThread: Connection failed (controller.connect returned False).") 
                self.was_connected = False
                print("BluetoothThread: Emitting 'Bluetooth Failed!' signal (from False return)...") 
                self.bluetooth_status_signal.emit("Bluetooth Failed!")
                print("BluetoothThread: 'Bluetooth Failed!' signal emitted.") 


        except Exception as e:
            print(f"BluetoothThread: Exception during connection: {e}") 
            if not self.was_connected: # Only emit Failed if it wasn't previously connected and then lost
                 self.bluetooth_status_signal.emit("Bluetooth Failed!")
                 print("BluetoothThread: 'Bluetooth Failed!' signal emitted.") 
            else:
                 print(f"BluetoothThread: Exception occurred but was_connected is True. Error: {e}. Might indicate disconnection.") 


    def send_command(self, command):
        """Send movement commands via Bluetooth with improved error handling."""
        # First check if connected 
        if not self.controller.connected:
            print(f"Not connected to Bluetooth device. Cannot send command: {command}")
            if hasattr(self, 'was_connected') and self.was_connected:
                self.was_connected = False
                print("Connection lost detected during command send")
                self.bluetooth_status_signal.emit("Bluetooth Disconnected (Not Connected)")
                self.connection_lost_signal.emit()
            return False
            
        if not self.loop or self.loop.is_closed():
            print("Event loop is closed. Cannot send command.")
            self.controller.connected = False
            self.was_connected = False
            self.bluetooth_status_signal.emit("Bluetooth Disconnected (Loop Closed)")
            self.connection_lost_signal.emit()
            return False
            
        try:
            future = asyncio.run_coroutine_threadsafe(
                self.controller.send_command(command), 
                self.loop
            )
            result = future.result(timeout=1.0) 
            return result
        except (asyncio.TimeoutError, asyncio.CancelledError):
            print(f"Command timed out: {command}")
            return False
        except Exception as e:
            print(f"Failed to send command: {e}")
            self.controller.connected = False
            self.was_connected = False
            self.bluetooth_status_signal.emit(f"Bluetooth Disconnected (Error: {str(e)})")
            self.connection_lost_signal.emit()
            
            QTimer.singleShot(0, lambda: self.connection_lost_signal.emit())
            return False

    def disconnect_bluetooth(self):
        """Disconnect from the Bluetooth device."""
        try:
            # Stop the connection check timer first
            if self.connection_check_timer and self.connection_check_timer.isActive():
                self.connection_check_timer.stop()
            
            # Only try to disconnect if connected
            if self.controller and self.controller.connected:
                loop = asyncio.new_event_loop()
                asyncio.set_event_loop(loop)
                
                try:
                    loop.run_until_complete(self.controller.disconnect())
                    self.was_connected = False
                    self.bluetooth_status_signal.emit("Bluetooth Disconnected.")
                except Exception as e:
                    print(f"Error during disconnect: {e}")
                    self.controller.connected = False
                    self.was_connected = False
                    self.bluetooth_status_signal.emit("Bluetooth Disconnected (Error).")
                finally:
                    loop.close()
        except Exception as e:
            print(f"Error in disconnect_bluetooth: {e}")

class SignalListenerThread(QThread):
    """Listens for signals from plot scripts and emits them to the main GUI."""
    blink_signal = pyqtSignal(str)
    focus_signal = pyqtSignal(bool)
    status_signal = pyqtSignal(str)  
    
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
        self.bluetooth_thread = None  
        self.signal_thread = None
        self.eeg_thread = None
        self.direction_timer = QTimer()  
        self.direction_timer.timeout.connect(self.reset_direction_display)
        self.direction_timer.setSingleShot(True)  

    def initUI(self):
        layout = QVBoxLayout()

        self.eeg_status_label = QLabel("EEG Status: Not Connected")
        layout.addWidget(self.eeg_status_label)

        self.bluetooth_status_label = QLabel("Bluetooth Status: Not Connected")
        layout.addWidget(self.bluetooth_status_label)

        self.movement_display = QLabel("")  
        self.movement_display.setFrameStyle(QFrame.Shape.Box | QFrame.Shadow.Raised)
        self.movement_display.setStyleSheet("font-size: 32px; text-align: center;")
        layout.addWidget(self.movement_display)

        self.connect_eeg_button = QPushButton("Connect to EEG Stream")
        self.connect_eeg_button.clicked.connect(self.start_eeg_stream)
        layout.addWidget(self.connect_eeg_button)

        self.connect_bluetooth_button = QPushButton("Connect to Bluetooth")
        self.connect_bluetooth_button.setEnabled(False)  
        self.connect_bluetooth_button.clicked.connect(self.connect_bluetooth)
        layout.addWidget(self.connect_bluetooth_button)

        self.setLayout(layout)
        self.setWindowTitle("EEG Car Monitor")
        self.resize(400, 300)

    def start_eeg_stream(self):
        """Starts the EEG stream and initializes monitoring."""
        self.connect_eeg_button.setEnabled(False)
        self.eeg_status_label.setText("EEG Status: Starting...")
        
        # Start the signal listener thread 
        self.signal_thread = SignalListenerThread()
        self.signal_thread.blink_signal.connect(self.update_movement)
        self.signal_thread.focus_signal.connect(self.update_focus)
        self.signal_thread.status_signal.connect(self.update_eeg_status)  
        self.signal_thread.start()
        
        # Start the EEG thread 
        self.eeg_thread = EEGThread()
        self.eeg_thread.eeg_status_signal.connect(self.update_eeg_status)
        self.eeg_thread.start()

    def connect_bluetooth(self):
        """Starts the Bluetooth connection."""
        self.connect_bluetooth_button.setEnabled(False)
        self.connect_bluetooth_button.setText("Connecting...")
        
        # Properly clean up the previous thread if it exists
        if self.bluetooth_thread is not None:
            try:
                try:
                    self.bluetooth_thread.bluetooth_status_signal.disconnect()
                except (TypeError, RuntimeError):
                    pass
                
                # Check if thread is running and terminate it properly
                if self.bluetooth_thread.isRunning():
                    self.bluetooth_thread.terminate()  
                    self.bluetooth_thread.wait(1000)   
                    
                self.bluetooth_thread = None
            except Exception as e:
                print(f"Error cleaning up Bluetooth thread: {e}")
        
        # Create a new BluetoothThread instance after a short delay
        QTimer.singleShot(100, self._create_new_bluetooth_thread)

    def _create_new_bluetooth_thread(self):
        """Creates a new Bluetooth thread after proper cleanup of the previous one."""
        try:
            self.bluetooth_thread = BluetoothThread()
            # Connect both signals with named methods for clarity
            self.bluetooth_thread.bluetooth_status_signal.connect(self.update_bluetooth_status)
            self.bluetooth_thread.connection_lost_signal.connect(self.handle_connection_lost)
            self.bluetooth_thread.connection_lost_signal.connect(
                lambda: QTimer.singleShot(0, self.immediate_connection_lost_update))
            self.bluetooth_thread.start()
        except Exception as e:
            print(f"Error creating new Bluetooth thread: {e}")
            self.bluetooth_status_label.setText(f"Bluetooth Status: Error - {str(e)}")
            self.connect_bluetooth_button.setEnabled(True)
            self.connect_bluetooth_button.setText("Connect to Bluetooth")
            
    def handle_connection_lost(self):
        """Handle Bluetooth connection being lost."""
        # Only update UI if we actually had a connection that was lost
        if self.bluetooth_thread and hasattr(self.bluetooth_thread, 'was_connected') and self.bluetooth_thread.was_connected:
            print("Bluetooth connection lost")
            # Re-enable the connect button
            self.connect_bluetooth_button.setEnabled(True)
            self.connect_bluetooth_button.setText("Connect to Bluetooth")
            
            self.movement_display.setText("Stop")
            
            # Update the status label with a more noticeable message
            self.bluetooth_status_label.setText("Bluetooth Status: CONNECTION LOST")
            
                

    def immediate_connection_lost_update(self):
        """Immediate UI update when connection is lost - called on main thread"""
        print("IMMEDIATE UI UPDATE: Bluetooth connection lost")
        # Force button to be re-enabled
        self.connect_bluetooth_button.setEnabled(True)
        self.connect_bluetooth_button.setText("Connect to Bluetooth")
        
        # Force status update
        self.bluetooth_status_label.setText("Bluetooth Status: CONNECTION LOST")
        self.bluetooth_status_label.setStyleSheet("color: red; font-weight: bold;")
        
        # Update movement display
        self.movement_display.setText("Stop")

    def update_movement(self, direction):
        """Updates movement display based on detected blink direction and sends Bluetooth command."""
        current_display = self.movement_display.text()
        print("Blink Direction:", direction)
        
        # Update the display regardless of bluetooth connection
        if direction == "left":
            self.movement_display.setText("Moving Left")
            self.direction_timer.start(500)
        elif direction == "right":
            self.movement_display.setText("Moving Right")
            self.direction_timer.start(500)
        
        # Only send commands if bluetooth is working
        bluetooth_working = self._check_bluetooth_connection()
        if bluetooth_working:
            if direction == "left":
                result = self.bluetooth_thread.send_command("MOVE_LEFT")
                if not result and self.bluetooth_thread.was_connected:
                    self.handle_connection_lost()
            elif direction == "right":
                result = self.bluetooth_thread.send_command("MOVE_RIGHT")
                if not result and self.bluetooth_thread.was_connected:
                    self.handle_connection_lost()
        else:
            print("Not connected to Bluetooth device. Movement command not sent.")

    def reset_direction_display(self):
        """Reset the direction display after the timer expires"""
        # If we were previously moving forward, restore that state
        if "Moving Left" in self.movement_display.text() or "Moving Right" in self.movement_display.text():
            was_moving_forward = self.direction_timer.property("was_moving_forward")
            if was_moving_forward:
                self.movement_display.setText("Moving Forward")
                bluetooth_working = self._check_bluetooth_connection()
                if bluetooth_working:
                    result = self.bluetooth_thread.send_command("MOVE_FORWARD")
                    if not result and self.bluetooth_thread.was_connected:
                        self.handle_connection_lost()
            else:
                self.movement_display.setText("Stop")  

    def update_focus(self, focused):
        """Updates movement display based on focus detection and sends Bluetooth command."""
        print("Focused:", focused)
        current_display = self.movement_display.text()
        
        if "Moving Left" in current_display or "Moving Right" in current_display:
            self.direction_timer.setProperty("was_moving_forward", focused)
        
        if focused:
            if "Moving Forward" not in current_display and "Moving Left" not in current_display and "Moving Right" not in current_display:
                self.movement_display.setText("Moving Forward")
        else:
            if "Moving Left" not in current_display and "Moving Right" not in current_display:
                self.movement_display.setText(current_display.replace("Moving Forward", "Stop").strip())
        
        # Only send commands if bluetooth thread exists and is connected
        bluetooth_working = self._check_bluetooth_connection()
        if bluetooth_working:
            command = "MOVE_FORWARD" if focused else "STOP"
            result = self.bluetooth_thread.send_command(command)
            if not result and self.bluetooth_thread.was_connected:
                self.handle_connection_lost()
        else:
            print("Not connected to Bluetooth device. Focus command not sent.")

    def _check_bluetooth_connection(self):
        """Helper method to check Bluetooth connection status."""
        return (self.bluetooth_thread is not None and 
                hasattr(self.bluetooth_thread, 'controller') and 
                self.bluetooth_thread.controller.connected)

    def update_eeg_status(self, status):
        """Updates EEG status and enables Bluetooth + plot buttons when connected."""
        self.eeg_status_label.setText(f"EEG Status: {status}")
        
        # Check if EEG stream was not found and exit program if so
        if "EEG stream not found!" in status:
            print("EEG stream not found. Closing application...")
            QTimer.singleShot(2000, lambda: QApplication.quit())  
            return
        
        # Enable Bluetooth button if both plots are connected
        if ("Focus plot: Connected" in status or "Blink plot: Connected" in status):
            self.connect_bluetooth_button.setEnabled(True)

    def update_bluetooth_status(self, status):
        """Updates Bluetooth connection status."""
        self.bluetooth_status_label.setText(f"Bluetooth Status: {status}")
        
        # Update the button 
        if "Connected!" in status:
            self.connect_bluetooth_button.setEnabled(False)
            self.connect_bluetooth_button.setText("Connected to Bluetooth")
            self.bluetooth_status_label.setStyleSheet("color: green;")
            
            # Reset style after a few seconds
            QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))
            
        elif "Disconnected" in status or "Failed" in status or "Error" in status:
            self.connect_bluetooth_button.setEnabled(True)
            self.connect_bluetooth_button.setText("Connect to Bluetooth")
            
            # Set the status label color to indicate a problem
            if "Failed" in status or "Error" in status:
                self.bluetooth_status_label.setStyleSheet("color: red;")
                QTimer.singleShot(3000, lambda: self.bluetooth_status_label.setStyleSheet(""))
            
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
    
    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    
    if args.stream:
        # Find available Muse devices
        muses = list_muses()
        
        if not muses:
            print("No Muse devices found.")
            print("Continuing without streaming...")
            window.eeg_status_label.setText("EEG Status: No Muse devices found. Run: `muselsl stream` in another window.")
        else:
            # Connect to the first available device
            print(f"Connecting to {muses[0]['name']} ({muses[0]['address']})...")
            print("Press Ctrl+C to disconnect.")
            
            # Start streaming in a thread with proper event loop setup
            streaming_thread = threading.Thread(target=run_stream_in_thread, args=(muses[0]['address'],))
            streaming_thread.daemon = True
            streaming_thread.start()
    else:
        # Display message in GUI that --stream parameter is not used
        instruction_text = (
            "Running without direct connection to Muse headset.\n"
            "Use --stream parameter next time to connect and stream from Muse.\n"
            "You must run: `muselsl stream` in another command window to start streaming."
        )
        window.eeg_status_label.setText(f"EEG Status: {instruction_text}")
        print("Running without direct connection to Muse headset.")
        print("Use --stream parameter next time to connect and stream from Muse.")
        print("You must run: `muselsl stream` in another command window to start streaming.")
    
    window.show()
    sys.exit(app.exec())