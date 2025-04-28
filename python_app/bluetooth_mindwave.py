import asyncio
from bleak import BleakClient, BleakScanner
import numpy as np
import pandas as pd
import sys
import json
import time
from telnetlib import Telnet
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter, welch

#install bluez with sudo apt install bluez
#install dbus

# UUIDs from the ESP32 GATT server code (full 128-bit UUIDs)
SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"  # GATTS_SERVICE_UUID_TEST_A
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # GATTS_CHAR_UUID_TEST_A


def bandpass_filter(data, low_cut=12.0, high_cut=30.0, fs=512, order=5):
    nyquist = 0.5*fs
    low = low_cut/nyquist
    high = high_cut/nyquist
    b, a = butter(order, [low, high], btype='band')
    y = lfilter(b, a, data)
    return y

def compute_beta_power(data, fs=512):
    freqs, psd = welch(data, fs, nperseg=1024)
    beta_power = np.sum(psd[(freqs >= 12) & (freqs <= 30)])
    return beta_power

async def scan_for_device(device_name="ESP_GATTS_DEMO"):
    print(f"Scanning for {device_name}...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, ad: (d.name or ad.local_name or "").lower() == device_name.lower()
    )
    return device

async def scan_all_devices(timeout=5.0):
    """Scan for all BLE devices and print their information."""
    print(f"Scanning for BLE devices for {timeout} seconds...")
    
    devices = await BleakScanner.discover(timeout=timeout)
    
    if not devices:
        print("No devices found!")
        return
    
    print(f"\nFound {len(devices)} devices:")
    print("-" * 50)
    
    for device in devices:
        device_name = device.name or "Unknown"
        print(f"\nDevice: {device_name}")
        print(f"Address: {device.address}")
        # print(f"Details: {device.details}")
        print(f"RSSI: {device.rssi} dBm")
        print("-" * 50)

async def connect_to_device(device):
    """Connect to the device and return the client."""
    try:
        client = BleakClient(device)
        await client.connect()
        print("Connected to device")
        return client
    except Exception as e:
        print(f"Connection error: {str(e)}")
        return None

async def handle_notification(sender, data):
    """Handle incoming notifications from the ESP32 device."""
    print(f"Received notification:")
    print(f"  From: {sender}")
    print(f"  Data (hex): {' '.join([f'{b:02x}' for b in data])}")
    print(f"  Data (ASCII): {data.decode(errors='replace')}")

async def list_services(client):
    """List all services and characteristics of the connected device."""
    print("\nAvailable Services and Characteristics:")
    for service in client.services:
        print(f"Service: {service.uuid}")
        for char in service.characteristics:
            properties = ', '.join(char.properties)
            print(f"  Characteristic: {char.uuid} | Properties: {properties}")
    print("-" * 50)

async def send_message(client, message=b"Hello ESP32!"):
    """Send a message to the connected device and handle notifications."""
    try:
        await client.write_gatt_char(CHARACTERISTIC_UUID, message)
        print(f"Sent message: {message.decode()}")
    except Exception as e:
        print(f"Failed to send message: {e}")
        exit(1)

async def run_neurosky(client):
    test_name = input("test name: ")
    rms_constant = 1.2
    run_time = 65
    # test_name = str(input("input name of test (for naming graphs at end): "))
    # rms_constant = int(input("input rms constant: "))
    # run_time = int(input("input time to take in data (sec): "))
    # Initializing the arrays required to store the data.
    attention_values = np.array([])
    meditation_values = np.array([])
    delta_values = np.array([])
    theta_values = np.array([])
    lowAlpha_values = np.array([])
    highAlpha_values = np.array([])
    lowBeta_values = np.array([])
    highBeta_values = np.array([])
    lowGamma_values = np.array([])
    highGamma_values = np.array([])
    blinkStrength_values = np.array([])
    time_array = np.array([])
    time_array_eeg = np.array([])
    raw_eeg = np.array([])
    focus_levels = np.array([])

    tn = Telnet('localhost', 13854)

    start = time.time()

    i = 0
    tn.write(b'{"enableRawOutput": true, "format": "Json"}\n')

    outfile = "null"
    if len(sys.argv) > 1:
        outfile = sys.argv[len(sys.argv) - 1]
        outfptr = open(outfile, 'w')

    eSenseDict = {'attention': 0, 'meditation': 0}
    waveDict = {'lowGamma': 0, 'highGamma': 0, 'highAlpha': 0, 'delta': 0, 'highBeta': 0, 'lowAlpha': 0, 'lowBeta': 0, 'theta': 0}
    signalLevel = 0
    prev_focus_above_threshold = False
    while time.time() - start < run_time:
        blinkStrength = 0
        line = tn.read_until(b'\r').decode('utf-8').strip()  # Read, decode, and strip the line
        #line = ser.readline().decode('utf-8').strip()  # Read, decode, and strip the line
        if line:  # Check if line is not empty
            try:
                data_dict = json.loads(line)  # Attempt to parse JSON
            except json.JSONDecodeError:
                print("Invalid JSON received:", line)  # Print for debugging
                continue  # Skip this iteration if JSON is invalid

            # Continue processing if JSON is valid
            timediff = time.time() - start
            if "poorSignalLevel" in data_dict:
                signalLevel = data_dict['poorSignalLevel']
            if "blinkStrength" in data_dict:
                blinkStrength = data_dict['blinkStrength']
            if "eegPower" in data_dict:
                waveDict = data_dict['eegPower']
                eSenseDict = data_dict['eSense']
            
            outputstr = (f"{timediff}, {signalLevel}, {blinkStrength}, "
                        f"{eSenseDict['attention']}, {eSenseDict['meditation']}, "
                        f"{waveDict['lowGamma']}, {waveDict['highGamma']}, "
                        f"{waveDict['highAlpha']}, {waveDict['delta']}, "
                        f"{waveDict['highBeta']}, {waveDict['lowAlpha']}, "
                        f"{waveDict['lowBeta']}, {waveDict['theta']}")

            # Append values to arrays as before
            time_array = np.append(time_array, [timediff])
            blinkStrength_values = np.append(blinkStrength_values, [blinkStrength])
            lowGamma_values = np.append(lowGamma_values, [waveDict['lowGamma']])
            highGamma_values = np.append(highGamma_values, [waveDict['highGamma']])
            highAlpha_values = np.append(highAlpha_values, [waveDict['highAlpha']])
            delta_values = np.append(delta_values, [waveDict['delta']])
            lowBeta_values = np.append(lowBeta_values, [waveDict['lowBeta']])
            highBeta_values = np.append(highBeta_values, [waveDict['highBeta']])
            theta_values = np.append(theta_values, [waveDict['theta']])
            lowAlpha_values = np.append(lowAlpha_values, [waveDict['lowAlpha']])
            attention_values = np.append(attention_values, [eSenseDict['attention']])
            meditation_values = np.append(meditation_values, [eSenseDict['meditation']])
            #print(outputstr)
            print(line)
            line_dict = json.loads(line)
            if("rawEeg" in line):
                time_array_eeg = np.append(time_array_eeg, [timediff])
                raw_eeg = np.append(raw_eeg, [line_dict['rawEeg']])
                if len(raw_eeg) > 512:
                    filtered_beta = bandpass_filter(raw_eeg[-512:])
                    focus_level = compute_beta_power(filtered_beta)
                    print(focus_level)
                    focus_levels = np.append(focus_levels, [focus_level])
                    threshold = np.sqrt(np.mean(np.square(focus_levels)))*rms_constant
                    #print(threshold)
                    
                    if focus_level > threshold and not prev_focus_above_threshold:
                        await send_message(client, b"ON")
                    elif focus_level <= threshold and prev_focus_above_threshold:
                        curr_time = time.time()
                        await send_message(client, b"OFF")
                    prev_focus_above_threshold = focus_level > threshold
                    
                    #print(f"Focus level (Beta power): {focus_level}")
                    #delay after on
                    #choose sample rate
            if outfile != "null":
                outfptr.write(outputstr + "\n")
    plt.figure()
    plt.xlabel("Time (s)")
    plt.ylabel("Raw EEG (uV)")
    plt.title("Raw EEG Over Time")
    plt.plot(time_array_eeg, raw_eeg)
    print(time_array_eeg)
    print(raw_eeg)
    raw_eeg_plot_title = "raw_eeg_plot_" + test_name + ".jpg"
    plt.savefig(raw_eeg_plot_title, format="jpg", dpi=300)  # dpi=300 for higher quality
    plt.figure()
    adjusted_time_array = time_array[len(time_array) - len(focus_levels):]
    plt.plot(adjusted_time_array, focus_levels)
    plt.axhline(y=threshold, color='r', linestyle="--")
    plt.xlabel("Time (s)")
    plt.ylabel("Focus Level (Beta Power, dB)")
    plt.title("Focus Level Over Time")
    #plt.ylim(0, 100000)  # Set upper limit to 100,000
    #plt.yticks(np.arange(0, 100001, 5000))  # Set ticks at intervals of 5,000 up to 100,000
    plt.grid(True, which='both', axis='y') 
    focus_plot_title = "focus_plot_" + test_name + ".jpg"
    plt.savefig(focus_plot_title, format="jpg", dpi=300)
    np.savetxt(('time_full_' + test_name + '.csv'), time_array, delimiter=',') 
    np.savetxt(('time_adjusted_focus_' + test_name + '.csv'), adjusted_time_array, delimiter=',') 
    np.savetxt(('focus_levels_' + test_name + '.csv'), focus_levels, delimiter=',') 
    np.savetxt(('raw_eeg_' + test_name + '.csv'), raw_eeg, delimiter=',') 

    tn.close()
    if outfile != "null":
        outfptr.close()

async def main():
    # Scan for all devices
    # await scan_all_devices()

    # Find the ESP32 device
    device = await scan_for_device()
    if not device:
        print("ESP32 device not found. Exiting.")
        return

    print(f"Found device: {device.name} ({device.address})")

    # Connect to the device
    client = await connect_to_device(device)
    if client and client.is_connected:
        print("Successfully connected to the device.")
        await list_services(client)

        # Subscribe to notifications
        await client.start_notify(CHARACTERISTIC_UUID, handle_notification)

        await run_neurosky(client)
        '''
        # Send "ON" message
        await send_message(client, b"ON")

        # Wait for 2 seconds
        await asyncio.sleep(2)

        # Send "OFF" message
        await send_message(client, b"OFF")
        '''
        # Disconnect after sending messages
        await client.disconnect()
        print("Disconnected from the device.")

    else:
        print("Failed to connect to the device.")
        exit(1)

    

if __name__ == "__main__":
    asyncio.run(main())
