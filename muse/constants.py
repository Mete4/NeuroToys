import logging

MUSE_NB_EEG_CHANNELS = 5
MUSE_SAMPLING_EEG_RATE = 256
LSL_EEG_CHUNK = 12

MUSE_NB_PPG_CHANNELS = 3
MUSE_SAMPLING_PPG_RATE = 64
LSL_PPG_CHUNK = 6

MUSE_NB_ACC_CHANNELS = 3
MUSE_SAMPLING_ACC_RATE = 52
LSL_ACC_CHUNK = 1

MUSE_NB_GYRO_CHANNELS = 3
MUSE_SAMPLING_GYRO_RATE = 52
LSL_GYRO_CHUNK = 1

# 00001800-0000-1000-8000-00805f9b34fb Generic Access 0x05-0x0b
# 00001801-0000-1000-8000-00805f9b34fb Generic Attribute 0x01-0x04
MUSE_GATT_ATTR_SERVICECHANGED = '00002a05-0000-1000-8000-00805f9b34fb' # ble std 0x02-0x04
# 0000fe8d-0000-1000-8000-00805f9b34fb Interaxon Inc. 0x0c-0x42
MUSE_GATT_ATTR_STREAM_TOGGLE = '273e0001-4c4d-454d-96be-f03bac821358' # serial 0x0d-0x0f
MUSE_GATT_ATTR_LEFTAUX = '273e0002-4c4d-454d-96be-f03bac821358' # not implemented yet 0x1c-0x1e
MUSE_GATT_ATTR_TP9 = '273e0003-4c4d-454d-96be-f03bac821358' # 0x1f-0x21
MUSE_GATT_ATTR_AF7 = '273e0004-4c4d-454d-96be-f03bac821358' # fp1 0x22-0x24
MUSE_GATT_ATTR_AF8 = '273e0005-4c4d-454d-96be-f03bac821358' # fp2 0x25-0x27
MUSE_GATT_ATTR_TP10 = '273e0006-4c4d-454d-96be-f03bac821358' # 0x28-0x2a
MUSE_GATT_ATTR_RIGHTAUX = '273e0007-4c4d-454d-96be-f03bac821358' #0x2b-0x2d
MUSE_GATT_ATTR_REFDRL = '273e0008-4c4d-454d-96be-f03bac821358' # not implemented yet 0x10-0x12
MUSE_GATT_ATTR_GYRO = '273e0009-4c4d-454d-96be-f03bac821358' # 0x13-0x15
MUSE_GATT_ATTR_ACCELEROMETER = '273e000a-4c4d-454d-96be-f03bac821358' # 0x16-0x18
MUSE_GATT_ATTR_TELEMETRY = '273e000b-4c4d-454d-96be-f03bac821358' # 0x19-0x1b
#MUSE_GATT_ATTR_MAGNETOMETER = '273e000c-4c4d-454d-96be-f03bac821358' # 0x2e-0x30
#MUSE_GATT_ATTR_PRESSURE = '273e000d-4c4d-454d-96be-f03bac821358' # 0x31-0x33
#MUSE_GATT_ATTR_ULTRAVIOLET = '273e000e-4c4d-454d-96be-f03bac821358' # 0x34-0x36
MUSE_GATT_ATTR_PPG1 = "273e000f-4c4d-454d-96be-f03bac821358" # ambient 0x37-0x39
MUSE_GATT_ATTR_PPG2 = "273e0010-4c4d-454d-96be-f03bac821358" # infrared 0x3a-0x3c
MUSE_GATT_ATTR_PPG3 = "273e0011-4c4d-454d-96be-f03bac821358" # red 0x3d-0x3f
MUSE_GATT_ATTR_THERMISTOR = "273e0012-4c4d-454d-96be-f03bac821358" # muse S only, not implemented yet 0x40-0x42

MUSE_ACCELEROMETER_SCALE_FACTOR = 0.0000610352
MUSE_GYRO_SCALE_FACTOR = 0.0074768

# How long to wait while scanning for devices
LIST_SCAN_TIMEOUT = 10.5
# How long to wait after device stops sending data before ending the stream
AUTO_DISCONNECT_DELAY = 3
# How long to wait in between connection attempts
RETRY_SLEEP_TIMEOUT = 1

LSL_SCAN_TIMEOUT = 5
LSL_BUFFER = 360

VIEW_SUBSAMPLE = 1
VIEW_BUFFER = 12

LOG_LEVELS = {
    'debug': logging.DEBUG,
    'info': logging.INFO,
    'warning': logging.WARNING,
    'error': logging.ERROR,
    'critical': logging.CRITICAL
}

# --- Constants Added/Modified for direct use ---
# Buffer window size for blink detection (similar to VIEW_BUFFER)
BLINK_WINDOW_SECONDS = 5
# Subsampling factor (can be kept low as no plotting)
SUBSAMPLE = 1
# Blink detection parameters (from blink_plot.py)
BLINK_SPIKE_THRESHOLD = 200
BLINK_SPIKE_MIN_DISTANCE = 50 # Samples
BLINK_SPIKE_PROMINENCE = 30
BLINK_MAX_INTERVAL = 0.2 # Seconds
BLINK_COOLDOWN = 0.5 # Seconds

# Focus detection parameters (from focus_plot.py)
FOCUS_BUFFER_LENGTH = 5            # seconds of EEG data buffer
FOCUS_EPOCH_LENGTH = 1             # seconds for FFT
FOCUS_OVERLAP_LENGTH = 0.8         # seconds overlap between epochs
FOCUS_SHIFT_LENGTH = FOCUS_EPOCH_LENGTH - FOCUS_OVERLAP_LENGTH # seconds
FOCUS_INDEX_CHANNEL = [1]          # Use channel index 1 (e.g., TP9)
FOCUS_RMS_CONSTANT = 1.0           # Thresholding constant
