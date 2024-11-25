import asyncio
from bluetooth import (
    scan_all_devices,
    scan_for_device,
    connect_to_device,
    list_services,
    handle_notification,
    send_message,
    CHARACTERISTIC_UUID
)

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