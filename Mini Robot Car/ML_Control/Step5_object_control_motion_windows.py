import time
import torch
import numpy as np
from torchvision import models, transforms
import cv2
import asyncio
from bleak import BleakClient
from PIL import Image
from threading import Thread

# Load ImageNet class labels
with open("imagenet1000_clsidx_to_labels.txt") as f:
    classes = [line.strip() for line in f.readlines()]

# Preprocess
preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406],
                         std=[0.229, 0.224, 0.225]),
])

# Load model
net = models.mobilenet_v2(weights=models.MobileNet_V2_Weights.DEFAULT)
net.eval()

# BLE config
# address = "48244CA4-D14D-C0FD-0995-69155F6546D0"
address = "48:87:2D:6F:D4:9C"
characteristic_uuid = "0000ffe1-0000-1000-8000-00805f9b34fb"

# Async BLE sender
async def ble_sender(queue: asyncio.Queue):
    while True:
        data = await queue.get()
        try:
            print(f"Connecting to BLE to send {data}")
            async with BleakClient(address) as client:
                if client.is_connected:
                    await client.write_gatt_char(characteristic_uuid, data)
                    print(f"Sent: {data}")
                else:
                    print("Failed to connect to BLE")
        except Exception as e:
            print(f"BLE error: {e}")
        queue.task_done()
        await asyncio.sleep(1)

# Frame loop in thread, gets event loop as arg
def frame_loop(ble_queue, main_loop):
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
    cap.set(cv2.CAP_PROP_FPS, 36)

    with torch.no_grad():
        while True:
            ret, image = cap.read()
            if not ret:
                continue

            image = image[:, :, [2, 1, 0]]
            input_tensor = preprocess(image).unsqueeze(0)
            output = net(input_tensor)

            top = list(enumerate(output[0].softmax(dim=0)))
            top.sort(key=lambda x: x[1], reverse=True)
            max_idx, max_val = top[0]

            print(f"{max_val.item() * 100:.2f}% {classes[max_idx]}")

            if max_idx == 589:  # hand blower
                asyncio.run_coroutine_threadsafe(ble_queue.put(bytearray(b'3')), main_loop)
            elif max_idx == 747:  # screen
                asyncio.run_coroutine_threadsafe(ble_queue.put(bytearray(b'4')), main_loop)

            time.sleep(0.1)

# Main async entry point
async def main():
    ble_queue = asyncio.Queue()
    loop = asyncio.get_running_loop()  # store main loop

    # Start BLE task
    asyncio.create_task(ble_sender(ble_queue))

    # Start OpenCV thread and pass main loop
    cam_thread = Thread(target=frame_loop, args=(ble_queue, loop), daemon=True)
    cam_thread.start()

    while True:
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())
