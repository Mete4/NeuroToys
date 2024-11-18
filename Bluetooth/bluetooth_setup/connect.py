import asyncio
from bleak import BleakClient
from bluetooth import scan_for_device, connect_to_device, handle_notification, CHARACTERISTIC_UUID
import json

async def handle_client(reader, writer):
    while True:
        data = await reader.readline()
        if not data:
            break
        message = data.decode().strip()
        print(f"Received command: {message}")
        if message is not None:
            await client.write_gatt_char(CHARACTERISTIC_UUID, message.encode())
            print(f"Sent message: {message}")
        elif message == "exit":
            print("Exiting...")
            writer.close()
            await writer.wait_closed()
            await client.disconnect()
            break

async def maintain_connection():
    # Find and connect to device
    device = await scan_for_device()
    if not device:
        print("Device not found!")
        return

    print(f"Found device: {device.name} ({device.address})")
    global client
    client = await connect_to_device(device)
    
    if not client:
        print("Failed to connect")
        return

    # Start notification handler
    await client.start_notify(CHARACTERISTIC_UUID, handle_notification)
    print("Connected and ready to receive commands!")

    # Start TCP server
    server = await asyncio.start_server(handle_client, '127.0.0.1', 8888)
    addr = server.sockets[0].getsockname()
    print(f'TCP server running on {addr}')

    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(maintain_connection())