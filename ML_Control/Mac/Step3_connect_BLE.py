import asyncio
from bleak import BleakClient
import nest_asyncio
nest_asyncio.apply()

# Address of the Bluetooth device
address = "AA01BB8B-0DDB-AE53-E434-AD014EE5DAEC"
# UUID of the characteristic to write to
characteristic_uuid = "0000ffe2-0000-1000-8000-00805f9b34fb"

async def send_data(address, characteristic_uuid, data):
    print(f"Connecting to device {address}...")
    async with BleakClient(address) as client:
        print("Connected")
        if client.is_connected:
            print("Client connected successfully")
            try:
                # Write data to the characteristic
                await client.write_gatt_char(characteristic_uuid, data)
                print(f"Data {data} sent to {characteristic_uuid}")
            except Exception as e:
                print(f"Failed to send data: {e}")
        else:
            print("Failed to connect")

if __name__ == "__main__":

    # convert list to bytearray
    data = bytearray(b'2')

    # Connect to device and send data
    loop = asyncio.get_event_loop()
    loop.run_until_complete(send_data(address, characteristic_uuid, data))

