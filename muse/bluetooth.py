import asyncio
from bleak import BleakClient, BleakScanner

# UUIDs from the ESP32 GATT server code (full 128-bit UUIDs)
SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"

class BluetoothController:
    def __init__(self):
        self.client = None
        self.connected = False

    async def scan_for_device(self, device_name="ESP_GATTS_DEMO"):
        """Scan and connect to the ESP32 device."""
        print(f"Scanning for {device_name}...")
        device = await BleakScanner.find_device_by_filter(
            lambda d, ad: (d.name or ad.local_name or "").lower() == device_name.lower()
        )
        return device

    async def connect(self, device_name="ESP_GATTS_DEMO"):
        """Connect to the ESP32 Bluetooth device."""
        device = await self.scan_for_device(device_name)
        if not device:
            print("ESP32 device not found.")
            return False

        print(f"Found device: {device.name} ({device.address})")

        try:
            self.client = BleakClient(device)
            await self.client.connect()
            self.connected = True
            print("Successfully connected to the device.")
            return True
        except Exception as e:
            print(f"Connection error: {str(e)}")
            return False

    async def send_command(self, command):
        """Send a movement command to the ESP32."""
        if not self.connected or not self.client:
            print("Not connected to Bluetooth device.")
            return

        try:
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, command.encode())
            print(f"Sent command: {command}")
        except Exception as e:
            print(f"Failed to send command: {e}")

    async def disconnect(self):
        """Disconnect from the Bluetooth device."""
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from the Bluetooth device.")
