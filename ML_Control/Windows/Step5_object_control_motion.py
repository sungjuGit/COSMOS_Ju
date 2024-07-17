import time
import torch
import numpy as np
from torchvision import models, transforms
import cv2
from PIL import Image
import asyncio
from bleak import BleakClient
import nest_asyncio
from concurrent.futures import ThreadPoolExecutor

nest_asyncio.apply()

# Load the ImageNet class labels
with open("imagenet1000_clsidx_to_labels.txt") as f:
    classes = [line.strip() for line in f.readlines()]

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 224)
cap.set(cv2.CAP_PROP_FPS, 36)

preprocess = transforms.Compose([
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
])

# Load the non-quantized model with pretrained weights
net = models.mobilenet_v2(pretrained=True)
net.eval()  # Set the model to evaluation mode

address = "48:87:2D:6F:D4:B4"
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

def send_bluetooth_command(data):
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(send_data(address, characteristic_uuid, data))

if __name__ == "__main__":
    started = time.time()
    last_logged = time.time()
    frame_count = 0
    executor = ThreadPoolExecutor(max_workers=1)

    with torch.no_grad():
        while True:
            # Read frame
            ret, image = cap.read()
            if not ret:
                raise RuntimeError("Failed to read frame")

            # Convert OpenCV output from BGR to RGB
            image = image[:, :, [2, 1, 0]]

            # Preprocess the image
            input_tensor = preprocess(image)

            # Create a mini-batch as expected by the model
            input_batch = input_tensor.unsqueeze(0)

            # Run model on CPU
            output = net(input_batch)

            # Log model performance
            frame_count += 1
            now = time.time()
            if now - last_logged > 1:
                last_logged = now
                frame_count = 0

            top = list(enumerate(output[0].softmax(dim=0)))
            top.sort(key=lambda x: x[1], reverse=True)

            max_idx, max_val = top[0]

            print(f"{max_val.item() * 100:.2f}% {classes[max_idx]}")

            if max_idx == 898:  # Check for water bottle detection
                data = bytearray(b'1')  # Command to send
                executor.submit(send_bluetooth_command, data)
                time.sleep(1)  # Avoid spamming the command
            
            elif max_idx == 784:  # Check for water bottle detection
                data = bytearray(b'2')  # Command to send
                executor.submit(send_bluetooth_command, data)
                time.sleep(1)  # Avoid spamming the command

            else:
                continue
                # data = bytearray(b'0')  # Command to send
                # executor.submit(send_bluetooth_command, data)
                # time.sleep(1)  # Avoid spamming the command
