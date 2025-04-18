from bluetooth import BluetoothController

import asyncio

async def main():
    controller = BluetoothController()
    
    # Scan for the ESP32 device
    device = await controller.scan_for_device()
    if not device:
        print("Device not found!")
        return

    print(f"Found device: {device.name} ({device.address})")

    # Connect to the device
    connected = await controller.connect()
    if connected:
        await send_user_input(controller)

async def send_user_input(controller):
    # Start handling notifications
    await controller.subscribe_to_notifications()
    try:
        while True:
            message = input("Enter message to send (or 'exit' to quit): ")
            if message.lower() == 'exit':
                break
            await controller.send_command(message)
    except KeyboardInterrupt:
        pass
    finally:
        await controller.disconnect()
        print("Disconnected from device.")

if __name__ == "__main__":
    asyncio.run(main())