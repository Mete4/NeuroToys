import sys
import asyncio
import numpy as np
import matplotlib.pyplot as plt
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLineEdit, QLabel
from PyQt6.QtCore import QThread, pyqtSignal
import pyqtgraph as pg
from bleak import BleakClient, BleakScanner
from pylsl import StreamInlet, resolve_byprop
import utils
from collections import deque
import time

SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"

BUFFER_LENGTH = 5  # EEG buffer length in seconds
EPOCH_LENGTH = 1
OVERLAP_LENGTH = 0.8
SHIFT_LENGTH = EPOCH_LENGTH - OVERLAP_LENGTH
INDEX_CHANNEL = [1]

class BluetoothThread(QThread):
    connection_status = pyqtSignal(str)
    
    def run(self):
        asyncio.run(self.connect_to_car())
    
    async def connect_to_car(self):
        self.connection_status.emit("Scanning for car...")
        device = await BleakScanner.find_device_by_filter(lambda d, ad: (d.name or "").lower() == "esp_gatts_demo")
        if not device:
            self.connection_status.emit("Car not found")
            return
        
        self.connection_status.emit(f"Found: {device.address}, connecting...")
        try:
            client = BleakClient(device)
            await client.connect()
            self.connection_status.emit("Connected to car!")
            await client.start_notify(CHARACTERISTIC_UUID, self.handle_notification)
        except Exception as e:
            self.connection_status.emit(f"Connection failed: {e}")
    
    async def handle_notification(self, sender, data):
        print(f"Received from {sender}: {data}")

class EEGThread(QThread):
    headset_status = pyqtSignal(str)
    movement_signal = pyqtSignal(str)
    
    def run(self):
        self.monitor_eeg()
    
    def monitor_eeg(self):
        self.headset_status.emit("Looking for EEG stream...")
        streams = resolve_byprop('type', 'EEG', timeout=2)
        if not streams:
            self.headset_status.emit("EEG headset not found")
            return
        
        inlet = StreamInlet(streams[0])
        self.headset_status.emit("EEG headset connected!")
        fs = int(inlet.info().nominal_srate())
        eeg_buffer = np.zeros((int(fs * BUFFER_LENGTH), 1))
        filter_state = None
        n_win_test = int(np.floor((BUFFER_LENGTH - EPOCH_LENGTH) / SHIFT_LENGTH + 1))
        band_buffer = np.zeros((n_win_test, 4))

        while True:
            eeg_data, timestamp = inlet.pull_chunk(timeout=1, max_samples=int(SHIFT_LENGTH * fs))
            if eeg_data:
                ch_data = np.array(eeg_data)[:, INDEX_CHANNEL]
                eeg_buffer, filter_state = utils.update_buffer(eeg_buffer, ch_data, notch=True, filter_state=filter_state)
                data_epoch = utils.get_last_data(eeg_buffer, EPOCH_LENGTH * fs)
                band_powers = utils.compute_band_powers(data_epoch, fs)
                band_buffer, _ = utils.update_buffer(band_buffer, np.asarray([band_powers]))
                smooth_band_powers = np.mean(band_buffer, axis=0)
                beta_metric = smooth_band_powers[3]
                if beta_metric > 0.5:
                    self.movement_signal.emit("Moving Forward")
                elif beta_metric < -0.5:
                    self.movement_signal.emit("Moving Backward")
                else:
                    self.movement_signal.emit("Stopped")

class EEGCarGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()
    
    def initUI(self):
        layout = QVBoxLayout()
        
        self.test_name_label = QLabel("Test Name:")
        self.test_name_input = QLineEdit()
        layout.addWidget(self.test_name_label)
        layout.addWidget(self.test_name_input)
        
        self.connect_button = QPushButton("Connect to Car")
        self.connect_button.clicked.connect(self.connect_to_car)
        layout.addWidget(self.connect_button)
        
        self.car_status_label = QLabel("Car Connection Status: Not Connected")
        layout.addWidget(self.car_status_label)
        
        self.headset_status_label = QLabel("EEG Headset Status: Not Connected")
        layout.addWidget(self.headset_status_label)
        
        self.movement_status_label = QLabel("Car Movement: Stopped")
        layout.addWidget(self.movement_status_label)
        
        self.setLayout(layout)
        self.setWindowTitle("EEG Car Control")
        self.resize(400, 300) #new commitd
    
    def connect_to_car(self):
        self.bluetooth_thread = BluetoothThread()
        self.bluetooth_thread.connection_status.connect(self.update_car_status)
        self.bluetooth_thread.start()
        
        self.eeg_thread = EEGThread()
        self.eeg_thread.headset_status.connect(self.update_headset_status)
        self.eeg_thread.movement_signal.connect(self.update_movement_status)
        self.eeg_thread.start()
    
    def update_car_status(self, status):
        self.car_status_label.setText(f"Car Connection Status: {status}")
    
    def update_headset_status(self, status):
        self.headset_status_label.setText(f"EEG Headset Status: {status}")
    
    def update_movement_status(self, status):
        self.movement_status_label.setText(f"Car Movement: {status}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = EEGCarGUI()
    window.show()
    sys.exit(app.exec())
