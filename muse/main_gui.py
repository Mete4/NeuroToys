import sys
import asyncio
import subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QFrame
from PyQt6.QtCore import QThread, pyqtSignal
from eeg_processor import EEGProcessor
from bluetooth import BluetoothController

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
        success = await self.controller.connect()
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

class EEGThread(QThread):
    """Handles EEG stream processing and signals for GUI updates."""
    blink_signal = pyqtSignal(str)
    focus_signal = pyqtSignal(bool)
    eeg_status_signal = pyqtSignal(str)

    def run(self):
        processor = EEGProcessor(self.blink_signal, self.focus_signal, self.eeg_status_signal)
        processor.monitor_eeg()

class EEGMonitorGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.bluetooth_thread = BluetoothThread()  # Bluetooth handler

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

        self.connect_eeg_button = QPushButton("Start EEG Stream")
        self.connect_eeg_button.clicked.connect(self.start_eeg_stream)
        layout.addWidget(self.connect_eeg_button)

        self.connect_bluetooth_button = QPushButton("Connect to Bluetooth")
        self.connect_bluetooth_button.setEnabled(False)  # Disabled until EEG connects
        self.connect_bluetooth_button.clicked.connect(self.connect_bluetooth)
        layout.addWidget(self.connect_bluetooth_button)

        self.launch_blink_plot_button = QPushButton("Open Blink Detection Plot")
        self.launch_blink_plot_button.setEnabled(False)
        self.launch_blink_plot_button.clicked.connect(self.launch_blink_plot)
        layout.addWidget(self.launch_blink_plot_button)

        self.launch_focus_plot_button = QPushButton("Open Focus Detection Plot")
        self.launch_focus_plot_button.setEnabled(False)
        self.launch_focus_plot_button.clicked.connect(self.launch_focus_plot)
        layout.addWidget(self.launch_focus_plot_button)

        self.setLayout(layout)
        self.setWindowTitle("EEG Car Monitor")
        self.resize(400, 300)

    def start_eeg_stream(self):
        """Starts the EEG stream using muselsl and initializes monitoring."""

        self.connect_to_eeg()

    def connect_to_eeg(self):
        """Starts monitoring EEG data."""
        self.eeg_thread = EEGThread()
        self.eeg_thread.blink_signal.connect(self.update_movement)
        self.eeg_thread.focus_signal.connect(self.update_focus)
        self.eeg_thread.eeg_status_signal.connect(self.update_eeg_status)
        self.eeg_thread.start()

    def connect_bluetooth(self):
        """Starts the Bluetooth connection."""
        self.bluetooth_thread.bluetooth_status_signal.connect(self.update_bluetooth_status)
        self.bluetooth_thread.start()

    def update_movement(self, direction):
        """Updates movement display based on detected blink direction and sends Bluetooth command."""
        current_display = self.movement_display.text()
        if direction == "left":
            self.movement_display.setText("⬅️" if "⬆️" not in current_display else "⬅️ ⬆️")
            self.bluetooth_thread.send_command("MOVE_LEFT")
        elif direction == "right":
            self.movement_display.setText("➡️" if "⬆️" not in current_display else "➡️ ⬆️")
            self.bluetooth_thread.send_command("MOVE_RIGHT")

    def update_focus(self, focused):
        """Updates movement display based on focus detection and sends Bluetooth command."""
        current_display = self.movement_display.text()
        if focused:
            if "⬆️" not in current_display:
                self.movement_display.setText(current_display + " ⬆️")
            self.bluetooth_thread.send_command("MOVE_FORWARD")
        else:
            self.movement_display.setText(current_display.replace("⬆️", "").strip())
            self.bluetooth_thread.send_command("STOP")

    def update_eeg_status(self, status):
        """Updates EEG status and enables Bluetooth + plot buttons when connected."""
        self.eeg_status_label.setText(f"EEG Status: {status}")
        if "connected" in status.lower():
            self.connect_bluetooth_button.setEnabled(True)
            self.launch_blink_plot_button.setEnabled(True)
            self.launch_focus_plot_button.setEnabled(True)

    def update_bluetooth_status(self, status):
        """Updates Bluetooth connection status."""
        self.bluetooth_status_label.setText(f"Bluetooth Status: {status}")

    def launch_blink_plot(self):
        subprocess.Popen(["python", "blink_plot.py"])

    def launch_focus_plot(self):
        subprocess.Popen(["python", "focus_plot.py"])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    window.show()
    sys.exit(app.exec())
