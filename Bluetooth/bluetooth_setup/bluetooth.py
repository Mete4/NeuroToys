import asyncio
from bleak import BleakClient, BleakScanner

# UUIDs from the ESP32 GATT server code (full 128-bit UUIDs)
SERVICE_UUID = "000000ff-0000-1000-8000-00805f9b34fb"  # GATTS_SERVICE_UUID_TEST_A
CHARACTERISTIC_UUID = "0000ff01-0000-1000-8000-00805f9b34fb"  # GATTS_CHAR_UUID_TEST_A

class BluetoothController:
    """Controller class for BLE communication with the ESP32."""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.notification_callback = None
    
    async def scan_for_device(self, device_name="ESP_CAR"):
        """Scan and find the specified BLE device.
        
        If the device_name ends with '*', it will match any device starting with the name.
        """
        print(f"Scanning for {device_name}...")
        
        # Check if we're matching by prefix (starts with)
        if device_name.endswith('*'):
            prefix = device_name[:-1]  # Remove the * at the end
            print(f"Scanning for devices starting with '{prefix}'")
            
            # Find all devices and filter manually for the prefix
            all_devices = await BleakScanner.discover(timeout=5.0)
            matching_devices = [d for d in all_devices 
                               if (d.name and d.name.startswith(prefix))]
            
            if matching_devices:
                print(f"Found {len(matching_devices)} devices matching '{prefix}'")
                return matching_devices
            return None
        else:
            # Exact name match (original behavior)
            device = await BleakScanner.find_device_by_filter(
                lambda d, ad: (d.name or ad.local_name or "").lower() == device_name.lower()
            )
            return device
    
    async def scan_all_devices(self, timeout=5.0):
        """Scan for all BLE devices and print their information."""
        print(f"Scanning for all BLE devices for {timeout} seconds...")
        devices = await BleakScanner.discover(timeout=timeout)
        
        if not devices:
            print("No devices found!")
            return []
        
        print(f"\nFound {len(devices)} devices:")
        print("-" * 50)
        
        for device in devices:
            device_name = device.name or "Unknown"
            print(f"\nDevice: {device_name}")
            print(f"Address: {device.address}")
            print(f"RSSI: {device.rssi} dBm")
            print("-" * 50)
        
        return devices
    
    async def connect(self, device_name="ESP_CAR"):
        """Connect to the ESP32 Bluetooth device."""
        device = await self.scan_for_device(device_name)
        if not device:
            print(f"{device_name} not found.")
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
        
    async def connect_to_device(self, device):
        """Connect to a specific Bluetooth device."""
        if not device:
            print("No device provided for connection.")
            return False

        print(f"Connecting to device: {device.name} ({device.address})")

        try:
            self.client = BleakClient(device)
            await self.client.connect()
            self.connected = True
            print("Successfully connected to the device.")
            return True
        except Exception as e:
            print(f"Connection error: {str(e)}")
            return False
        
    async def list_services(self):
        """List all services and characteristics of the connected device."""
        if not self.connected or not self.client:
            print("Not connected to Bluetooth device.")
            return
            
        print("\nAvailable Services and Characteristics:")
        for service in self.client.services:
            print(f"Service: {service.uuid}")
            for char in service.characteristics:
                properties = ', '.join(char.properties)
                print(f"  Characteristic: {char.uuid} | Properties: {properties}")
        print("-" * 50)
    
    def handle_notification(self, sender, data):
        """Handle incoming notifications from the ESP32 device."""
        print(f"Received notification:")
        print(f"  From: {sender}")
        print(f"  Data (hex): {' '.join([f'{b:02x}' for b in data])}")
        print(f"  Data (ASCII): {data.decode(errors='replace')}")
        
        # If a custom callback has been registered, call it
        if self.notification_callback:
            self.notification_callback(sender, data)
    
    async def subscribe_to_notifications(self, characteristic_uuid=None):
        """Subscribe to notifications from the specified characteristic."""
        if not self.connected or not self.client:
            print("Not connected to Bluetooth device.")
            return False
            
        try:
            await self.client.start_notify(
                characteristic_uuid or CHARACTERISTIC_UUID, 
                self.handle_notification
            )
            print(f"Subscribed to notifications on {characteristic_uuid or CHARACTERISTIC_UUID}")
            return True
        except Exception as e:
            print(f"Failed to subscribe to notifications: {e}")
            return False
    
    async def send_command(self, command):
        """Send a movement command to the ESP32."""
        if not self.connected or not self.client:
            print("Not connected to Bluetooth device.")
            return False
        
        try:
            # Accept both string and bytes inputs
            if isinstance(command, str):
                command_bytes = command.encode()
            else:
                command_bytes = command
                
            await self.client.write_gatt_char(CHARACTERISTIC_UUID, command_bytes)
            print(f"Sent command: {command}")
            return True
        except Exception as e:
            print(f"Failed to send command: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the Bluetooth device."""
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from the Bluetooth device.")
            return True
        return False
    
    def set_notification_callback(self, callback):
        """Set a custom callback function to handle notifications."""
        self.notification_callback = callback

    async def is_connected(self):
        """Check if the connection is active by attempting a simple operation."""
        if not self.client or not self.connected:
            print("is_connected: Not connected according to flags")
            return False
        
        try:
            # Try to read a characteristic to verify connection
            services = self.client.services
            if not services:
                print("is_connected: Client has no services")
                return False
            print("is_connected: Connection verified via services access")
            return True
        except Exception as e:
            print(f"is_connected: Error verifying connection: {e}")
            # Update the connected flag since we detected a disconnection
            self.connected = False
            return False


async def main():
    """Example usage of the BluetoothController class."""
    controller = BluetoothController()
    
    # Scan for all devices
    await controller.scan_all_devices()
    
    # Connect to the ESP32 device
    connected = await controller.connect()
    if connected:
        # List services
        await controller.list_services()
        
        # Subscribe to notifications
        await controller.subscribe_to_notifications()
        
        # Send a command
        await controller.send_command("MOVE_FORWARD")
        await asyncio.sleep(2)  # Wait 2 seconds
        await controller.send_command("STOP")
        await asyncio.sleep(1)  # Wait 1 second
        
        # Disconnect
        await controller.disconnect()
    else:
        print("Failed to connect to the device.")


if __name__ == "__main__":
    # If run directly, execute the main example
    asyncio.run(main())
