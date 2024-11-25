import asyncio

async def send_command(message):
    reader, writer = await asyncio.open_connection('127.0.0.1', 8888)
    writer.write(f"{message}\n".encode())
    await writer.drain()
    # print(f"Sent: {message}")
    writer.close()
    await writer.wait_closed()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python send.py send <command>")
        sys.exit(1)
    command = ' '.join(sys.argv[1:])
    asyncio.run(send_command(command))