import sys
import json
import time
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QVBoxLayout, QWidget, QPushButton, QLineEdit, QLabel, QGridLayout, QCheckBox
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from pyqtgraph import PlotWidget, mkPen
from bleak import BleakClient, BleakScanner
from telnetlib import Telnet
from scipy.signal import butter, lfilter, welch

# Bluetooth UUIDs
SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"

def bandpass_filter(data, low_cut=12.0, high_cut=30.0, fs=512, order=5):
    nyquist = 0.5 * fs
    low = low_cut / nyquist
    high = high_cut / nyquist
    b, a = butter(order, [low, high], btype="band")
    y = lfilter(b, a, data)
    return y

def compute_beta_power(data, fs=512):
    nperseg = min(len(data), 1024)  # Set nperseg to the smaller of the data length or 1024
    freqs, psd = welch(data, fs, nperseg=nperseg)
    beta_power = np.sum(psd[(freqs >= 12) & (freqs <= 30)])
    return beta_power


async def find_device(device_name="ESP_GATTS_DEMO"):
    print(f"Scanning for {device_name}...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: (d.name or ad.local_name or "").lower() == device_name.lower()
    )
    return device

class NeuroskyThread(QThread):
    data_updated = pyqtSignal(float, float, float, float)  # time, raw_eeg, focus_level, threshold
    status_updated = pyqtSignal(str)  # Update status messages

    def __init__(self, rms_constant, run_time, parent=None):
        super().__init__(parent)
        self.rms_constant = rms_constant
        self.run_time = run_time
        self.running = False
        self.client = None  # Bluetooth client
    async def setup_bluetooth(self):
        device = await find_device()
        if not device:
            self.status_updated.emit("Device not found.")
            return None
        self.client = BleakClient(device)
        await self.client.connect()
        if self.client.is_connected:
            self.status_updated.emit("Connected to Bluetooth device.")
            return self.client
        else:
            self.status_updated.emit("Failed to connect to Bluetooth device.")
            return None

    async def send_message(self, message):
        if self.client:
            try:
                await self.client.write_gatt_char(CHARACTERISTIC_UUID, message.encode())
                self.status_updated.emit(f"Sent: {message}")
            except Exception as e:
                self.status_updated.emit(f"Error sending message: {e}")

    def run(self):
        self.running = True
        tn = Telnet("localhost", 13854)
        tn.write(b'{"enableRawOutput": true, "format": "Json"}\n')

        start_time = time.time()
        raw_eeg = np.array([])
        focus_levels = np.array([])
        prev_focus_above_threshold = False

        while self.running and (time.time() - start_time < self.run_time):
            current_time = time.time() - start_time
            line = tn.read_until(b"\r").decode("utf-8").strip()
            if not line:
                continue
            try:
                data_dict = json.loads(line)
            except json.JSONDecodeError:
                continue
            if "rawEeg" in data_dict:
                raw_value = data_dict["rawEeg"]
                raw_eeg = np.append(raw_eeg, [raw_value])

            if len(raw_eeg) > 512:
                filtered_beta = bandpass_filter(raw_eeg[-512:])
                focus_level = compute_beta_power(filtered_beta)
                focus_levels = np.append(focus_levels, [focus_level])
                threshold = np.sqrt(np.mean(np.square(focus_levels))) * self.rms_constant
                self.data_updated.emit(current_time, raw_value, focus_level, threshold)

                if focus_level > threshold and not prev_focus_above_threshold:
                    self.status_updated.emit("Focus above threshold: Turning ON.")
                    if not self.use_dummy_data:
                        asyncio.run(self.send_message("ON"))
                elif focus_level <= threshold and prev_focus_above_threshold:
                    self.status_updated.emit("Focus below threshold: Turning OFF.")
                    if not self.use_dummy_data:
                        asyncio.run(self.send_message("OFF"))

                prev_focus_above_threshold = focus_level > threshold

        tn.close()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("NeuroToys Control Panel v1.0")
        self.thread = None

        #GUI Layout
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QGridLayout()
        self.central_widget.setLayout(self.layout)

        #input fields
        self.test_name_label = QLabel("Test Name:")
        self.test_name_input = QLineEdit()
        self.rms_label = QLabel("RMS Threshold Constant:")
        self.rms_input = QLineEdit("1.2")
        self.run_time_label = QLabel("Run Time (seconds):")
        self.run_time_input = QLineEdit("60")

        #start button
        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.start_thread)

        #live plots
        self.raw_eeg_plot = PlotWidget()
        self.raw_eeg_plot.setTitle("Raw EEG vs. Time")
        self.raw_eeg_plot.setLabel("left", "Raw EEG (uV)")
        self.raw_eeg_plot.setLabel("bottom", "Time (s)")

        self.focus_plot = PlotWidget()
        self.focus_plot.setTitle("Focus Level vs. Time")
        self.focus_plot.setLabel("left", "Focus Level (Beta Power in dB)")
        self.focus_plot.setLabel("bottom", "Time (s)")

        #status label
        self.status_label = QLabel("Status: Waiting to start...")
        self.layout.addWidget(self.status_label, 0, 0, 1, 2)

        #add widgets
        self.layout.addWidget(self.test_name_label, 1, 0)
        self.layout.addWidget(self.test_name_input, 1, 1)
        self.layout.addWidget(self.rms_label, 2, 0)
        self.layout.addWidget(self.rms_input, 2, 1)
        self.layout.addWidget(self.run_time_label, 3, 0)
        self.layout.addWidget(self.run_time_input, 3, 1)
        self.layout.addWidget(self.start_button, 5, 0, 1, 2)
        self.layout.addWidget(self.raw_eeg_plot, 6, 0, 1, 2)
        self.layout.addWidget(self.focus_plot, 7, 0, 1, 2)

        #plot data storage
        self.raw_eeg_data = []
        self.time_data = []
        self.focus_data = []
        self.threshold_data = []

        self.raw_curve = self.raw_eeg_plot.plot(pen=mkPen("b"))
        self.focus_curve = self.focus_plot.plot(pen=mkPen("g"))
        self.threshold_curve = self.focus_plot.plot(pen=mkPen("r", style=Qt.DashLine))

    def start_thread(self):
        test_name = self.test_name_input.text()
        rms_constant = float(self.rms_input.text())
        run_time = int(self.run_time_input.text())

        self.thread = NeuroskyThread(rms_constant, run_time)
        self.thread.data_updated.connect(self.update_plots)
        self.thread.status_updated.connect(self.update_status)
        self.thread.start()

    def update_plots(self, time, raw_eeg, focus_level, threshold):
        self.raw_eeg_data.append(raw_eeg)
        self.time_data.append(time)
        self.focus_data.append(focus_level)

        self.raw_curve.setData(self.time_data, self.raw_eeg_data)
        self.focus_curve.setData(self.time_data, self.focus_data)
        if self.time_data:
            self.threshold_curve.setData([self.time_data[0], self.time_data[-1]], [threshold, threshold])

    def update_status(self, status):
        self.status_label.setText(f"Status: {status}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
