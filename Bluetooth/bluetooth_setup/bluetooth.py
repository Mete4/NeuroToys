import asyncio
from bleak import BleakClient, BleakScanner

# UUIDs from the ESP32 GATT server code (full 128-bit UUIDs)
SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"  # GATTS_SERVICE_UUID_TEST_A
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # GATTS_CHAR_UUID_TEST_A

async def scan_for_device(device_name="ESP_GATTS"):
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

async def main():
    # Scan for all devices
    # await scan_all_devices()

    # Find the ESP32 device
    device = await scan_for_device()
    if not device:
        print("Device not found. Exiting.")
        return

    print(f"Found device: {device.name} ({device.address})")

    # Connect to the device
    client = await connect_to_device(device)
    if client and client.is_connected:
        print("Successfully connected to the device.")
        await list_services(client)

        # Subscribe to notifications
        await client.start_notify(CHARACTERISTIC_UUID, handle_notification)

        # Send "ON" message
        await send_message(client, b"ON")

        # Wait for 2 seconds
        await asyncio.sleep(2)

        # Send "OFF" message
        await send_message(client, b"OFF")

        # Disconnect after sending messages
        await client.disconnect()
        print("Disconnected from the device.")

    else:
        print("Failed to connect to the device.")
        exit(1)

if __name__ == "__main__":
    asyncio.run(main())
