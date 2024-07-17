import asyncio
from bleak import BleakClient
import nest_asyncio
nest_asyncio.apply()

# Address of the Bluetooth device
address = "48:87:2D:6F:D4:B4"

async def discover_services(address):
    print(f"Connecting to device {address}...")
    async with BleakClient(address) as client:
        print("Connected")
        if client.is_connected:
            print("Client connected successfully")
            try:
                # Discover services
                services = await client.get_services()
                for service in services:
                    print(f"Service: {service.uuid}")
                    for characteristic in service.characteristics:
                        print(f"  Characteristic: {characteristic.uuid}, Properties: {characteristic.properties}")
            except Exception as e:
                print(f"Failed to discover services: {e}")
        else:
            print("Failed to connect")

if __name__ == "__main__":
    # Discover services and characteristics
    loop = asyncio.get_event_loop()
    loop.run_until_complete(discover_services(address))
