import asyncio
from bluetooth import BluetoothController

controller = None

async def handle_client(reader, writer):
    while True:
        data = await reader.readline()
        if not data:
            break
        message = data.decode().strip()
        print(f"Received command: {message}")
        if message and message != "exit":
            await controller.send_command(message)
            print(f"Sent message: {message}")
        elif message == "exit":
            print("Exiting...")
            writer.close()
            await writer.wait_closed()
            await controller.disconnect()
            break

async def maintain_connection():
    global controller
    controller = BluetoothController()
    
    # Find and connect to device
    device = await controller.scan_for_device()
    if not device:
        print("Device not found!")
        return

    print(f"Found device: {device.name} ({device.address})")
    connected = await controller.connect()
    
    if not connected:
        print("Failed to connect")
        return

    # Start notification handler
    await controller.subscribe_to_notifications()
    print("Connected and ready to receive commands!")

    # Start TCP server
    server = await asyncio.start_server(handle_client, '127.0.0.1', 8888)
    addr = server.sockets[0].getsockname()
    print(f'TCP server running on {addr}')

    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(maintain_connection())