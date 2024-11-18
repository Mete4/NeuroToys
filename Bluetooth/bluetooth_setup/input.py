from bluetooth import scan_for_device, connect_to_device, send_message, handle_notification, CHARACTERISTIC_UUID

import asyncio

async def main():
    # Scan for the ESP32 device
    device = await scan_for_device()
    if not device:
        print("Device not found!")
        return

    print(f"Found device: {device.name} ({device.address})")

    # Connect to the device
    client = await connect_to_device(device)
    if client:
        await send_user_input(client)

async def send_user_input(client):
    # Start handling notifications
    await client.start_notify(CHARACTERISTIC_UUID, handle_notification)
    try:
        while True:
            message = input("Enter message to send (or 'exit' to quit): ")
            if message.lower() == 'exit':
                break
            await send_message(client, message.encode())
    except KeyboardInterrupt:
        pass
    finally:
        await client.disconnect()
        print("Disconnected from device.")

if __name__ == "__main__":
    asyncio.run(main())