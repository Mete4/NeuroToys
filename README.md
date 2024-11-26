# NeuroToys: Non-Invasive Brain-Computer Interface for Real-Time Robot Control

## Project Overview
NeuroToys is a senior capstone project aimed at developing a non-invasive brain-computer interface (BCI) that allows users to control a robotic device in real-time using EEG signals. The system interprets brainwave patterns to translate them into commands for robot movement, improving accessibility for individuals with physical disabilities. This project combines expertise in mechanical design, electrical engineering, and machine learning.

## Project Structure

### Bluetooth Components

#### `/Bluetooth/bluetooth_ino`
Contains the Arduino-compatible code for the ESP32 microcontroller:
- `bluetooth.ino`: Implements BLE server functionality and motor control
  - Sets up GATT server for BLE communication
  - Handles motor control pins and movement functions (Forward, Backward, Left, Right, Stop)
  - Processes incoming BLE commands to control car movement

#### `/Bluetooth/bluetooth_setup`
Python-based Bluetooth client implementation with modular components:

- `bluetooth.py`: Core BLE functionality
  - Device scanning and management
  - Connection handling
  - Service discovery
  - Message handling and notifications
  - Key functions:
    - `scan_for_device`: Finds specific ESP32 device
    - `scan_all_devices`: Discovers all nearby BLE devices
    - `connect_to_device`: Establishes device connection
    - `handle_notification`: Processes device notifications
    - `list_services`: Enumerates available services
    - `send_message`: Transmits commands to device

- `example.py`: Demonstration implementation
  - Shows basic device interaction flow
  - Implements device scanning and connection
  - Demonstrates service discovery
  - Sends test commands ("ON"/"OFF") with delays
  - Handles notifications and disconnection

- `input.py`: Interactive control interface
  - Manual message sending capability
  - Continuous user input handling
  - Command processing and transmission
  - Disconnection handling

- `connect.py`: Persistent connection manager
  - TCP server implementation for continuous connection
  - Handles incoming TCP messages
  - Routes commands to BLE device
  - Maintains persistent device connection
  - Functions:
    - `handle_client`: TCP message processor
    - `maintain_connection`: Keep connection open

- `send.py`: Command sending tool
  - Asynchronous TCP client implementation
  - Simple command sending interface
  - Connection handling

#### `/Bluetooth/gatt_server`
ESP-IDF based GATT server implementation:
- Complete BLE server setup using ESP-IDF 
- Implements service and characteristic definitions
- Handles client connections and data exchange
- Includes motor control logic integrated with BLE commands

## Key Features
- Real-time BLE communication between client and car
- Motor control with 5 movement modes (Forward, Backward, Left, Right, Stop)
- Robust connection handling and command processing
- Support for both manual control and automated movement patterns
- Persistent connection management through TCP server
- Interactive command interface for manual control
- Device scanning and service discovery

## Technical Stack
- ESP32 microcontroller for car control
- Bluetooth Low Energy (BLE) for wireless communication
- Python-based client implementation using Bleak library
- ESP-IDF/ Arduino for GATT server
- Motor driver  for movement control
- Asynchronous I/O for communication handling
