import time
import asyncio
from bleak import BleakClient
import nest_asyncio
nest_asyncio.apply()

# Address of the Bluetooth device
address = "48244CA4-D14D-C0FD-0995-69155F6546D0"

# UUID of the characteristic to write to
characteristic_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

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

def send_bluetooth_command(data):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(send_data(address, characteristic_uuid, data))

if __name__ == "__main__":

    # convert list to bytearray
    data = bytearray(b'1')
    send_bluetooth_command(data)
    time.sleep(1)

    data = bytearray(b'2')
    send_bluetooth_command(data)
    time.sleep(1)

    data = bytearray(b'3')
    send_bluetooth_command(data)
    