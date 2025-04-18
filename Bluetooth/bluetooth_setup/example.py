import asyncio
from bluetooth import BluetoothController

async def main():
    controller = BluetoothController()
    
    # Scan for all devices
    # await controller.scan_all_devices()

    # Find the ESP32 device
    device = await controller.scan_for_device()
    if not device:
        print("ESP32 device not found. Exiting.")
        return

    print(f"Found device: {device.name} ({device.address})")

    # Connect to the device
    connected = await controller.connect()
    if connected:
        print("Successfully connected to the device.")
        await controller.list_services()

        # Subscribe to notifications
        await controller.subscribe_to_notifications()

        # Send "ON" message
        await controller.send_command("MOVE_FORWARD")

        # Wait for 2 seconds
        await asyncio.sleep(2)

        # Send "OFF" message
        await controller.send_command("STOP")

        # Disconnect after sending messages
        await controller.disconnect()
        print("Disconnected from the device.")

    else:
        print("Failed to connect to the device.")
        exit(1)

if __name__ == "__main__":
    asyncio.run(main())