import sys
import subprocess
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QFrame
from PyQt6.QtCore import QThread, pyqtSignal
from eeg_processor import EEGProcessor

class EEGThread(QThread):
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
        self.connect_bluetooth_button.setEnabled(False)  # Initially disabled
        layout.addWidget(self.connect_bluetooth_button)

        self.launch_blink_plot_button = QPushButton("Open Blink Detection Plot")
        self.launch_blink_plot_button.setEnabled(False)  # Initially disabled
        self.launch_blink_plot_button.clicked.connect(self.launch_blink_plot)
        layout.addWidget(self.launch_blink_plot_button)

        self.launch_focus_plot_button = QPushButton("Open Focus Detection Plot")
        self.launch_focus_plot_button.setEnabled(False)  # Initially disabled
        self.launch_focus_plot_button.clicked.connect(self.launch_focus_plot)
        layout.addWidget(self.launch_focus_plot_button)

        self.setLayout(layout)
        self.setWindowTitle("NeuroToys EEG Monitor")
        self.resize(400, 300)

    def start_eeg_stream(self):
        """Starts the EEG stream using muselsl and initializes monitoring."""
        self.eeg_status_label.setText("Starting EEG stream...")

        self.connect_to_eeg()

    def connect_to_eeg(self):
        """Starts monitoring EEG data."""
        self.eeg_thread = EEGThread()
        self.eeg_thread.blink_signal.connect(self.update_movement)
        self.eeg_thread.focus_signal.connect(self.update_focus)
        self.eeg_thread.eeg_status_signal.connect(self.update_eeg_status)
        self.eeg_thread.start()

    def update_movement(self, direction):
        """Updates movement display based on detected blink direction."""
        current_display = self.movement_display.text()
        if direction == "left":
            self.movement_display.setText("⬅️" if "⬆️" not in current_display else "⬅️ ⬆️")
        elif direction == "right":
            self.movement_display.setText("➡️" if "⬆️" not in current_display else "➡️ ⬆️")

    def update_focus(self, focused):
        """Updates movement display based on focus threshold."""
        current_display = self.movement_display.text()
        if focused:
            if "⬆️" not in current_display:
                self.movement_display.setText(current_display + " ⬆️")
        else:
            self.movement_display.setText(current_display.replace("⬆️", "").strip())

    def update_eeg_status(self, status):
        """Updates the EEG connection status and enables buttons when connected."""
        self.eeg_status_label.setText(f"EEG Status: {status}")
        if "connected" in status.lower():
            self.connect_bluetooth_button.setEnabled(True)
            self.launch_blink_plot_button.setEnabled(True)
            self.launch_focus_plot_button.setEnabled(True)

    def launch_blink_plot(self):
        subprocess.Popen(["python", "blink_plot.py"])

    def launch_focus_plot(self):
        subprocess.Popen(["python", "focus_plot.py"])

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EEGMonitorGUI()
    window.show()
    sys.exit(app.exec())
