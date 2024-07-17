import time
import torch
import numpy as np
from torchvision import models, transforms
import cv2
from PIL import Image

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

started = time.time()
last_logged = time.time()
frame_count = 0

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
            # print(f"{frame_count / (now - last_logged)} fps")
            last_logged = now
            frame_count = 0

        top = list(enumerate(output[0].softmax(dim=0)))
        top.sort(key=lambda x: x[1], reverse=True)

        max_idx, max_val = top[0]
        
        print(f"{max_val.item() * 100:.2f}% {classes[max_idx]}")

