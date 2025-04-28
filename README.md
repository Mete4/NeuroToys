# NeuroToys: Non-Invasive Brain-Computer Interface for Real-Time Robot Control

## Project Overview
NeuroToys is a senior capstone project aimed at developing a non-invasive brain-computer interface (BCI) that allows users to control a robotic device in real-time using EEG signals. The system interprets brainwave patterns to translate them into commands for robot movement, improving accessibility for individuals with physical disabilities. This project combines expertise in mechanical design, electrical engineering, and machine learning.

## Project Structure

### `/Bluetooth`
Contains components related to Bluetooth Low Energy (BLE) communication and ESP32 control.

#### `/Bluetooth/bluetooth_setup`
Python scripts for BLE client-side operations:
- `bluetooth.py`: Core `BluetoothController` class managing BLE interactions. Handles scanning (specific device `ESP_CAR` or all), connection, service/characteristic discovery, command sending (strings/bytes), notification handling, and disconnection.
- `example.py`: Demonstrates basic usage of `BluetoothController` for connecting, listing services, subscribing to notifications, sending test commands (`MOVE_FORWARD`, `STOP`), and disconnecting.
- `input.py`: Provides an interactive command-line interface to manually send messages to the connected ESP32 car.
- `connect.py`: Implements a TCP server (`127.0.0.1:8888`) that maintains a persistent BLE connection to the car. It receives commands via TCP and forwards them to the car via BLE, allowing control from other applications or networked devices.
- `send.py`: A simple asynchronous TCP client to send commands to the `connect.py` server.

#### `/Bluetooth/bluetooth_ino`
Arduino code for the ESP32 microcontroller:
- `bluetooth.ino`: Implements a BLE GATT server (`ESP_CAR`) using the Arduino BLE library. Defines the service (`000000ff-...`) and characteristic (`0000ff01-...`) UUIDs. Receives commands via BLE writes and controls the car's motors (connected to GPIOs 15, 2, 4, 16) and a reverse LED (GPIO 17). Includes functions for `Forward`, `Reverse`, `Left`, `Right`, `Stop`, `Reverse_LED_ON`, `Reverse_LED_OFF`. Performs a motor test sequence on setup.

#### `/Bluetooth/gatt_server`
ESP-IDF based GATT server implementation for the ESP32:
- `main/gatts_demo.c`: A more detailed BLE GATT server implementation using the ESP-IDF framework. Defines services and characteristics (matching the Arduino version), handles BLE events (connection, disconnection, writes, reads, notifications), and integrates motor control functions (`Forward`, `Reverse`, `Left`, `Right`, `Stop`, `Reverse_LED_ON`, `Reverse_LED_OFF`) triggered by received BLE commands. Also performs a motor test sequence on startup.

## Key Features
- Real-time BLE communication between client and car
- Motor control with commands: Forward, Reverse, Left, Right, Stop, Reverse LED On/Off
- Robust BLE connection handling and command processing
- Support for manual control via interactive command interface (`input.py`)
- Persistent BLE connection management through TCP server (`connect.py`)
- Device scanning and service discovery capabilities
- Example script (`example.py`) for basic BLE interaction demonstration
- TCP client (`send.py`) for sending commands to the persistent connection server

## Technical Stack
- ESP32 microcontroller for car control
- Bluetooth Low Energy (BLE) for wireless communication
- Python-based client implementation using `Bleak` library
- ESP-IDF and Arduino Core for ESP32 for GATT server implementations
- Motor driver (L298N assumed) for movement control
- Asynchronous I/O (`asyncio`) for communication handling
- TCP/IP for persistent connection option
