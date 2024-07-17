import asyncio
from bleak import BleakClient, BleakScanner

def notification_handler(sender, data):
    """Notification handler which prints the data received."""
    print(f"Notification from {sender}: {data.decode('utf-8')}")

async def run():
    print("Starting BLE scan...")
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Found device: {device}")

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
