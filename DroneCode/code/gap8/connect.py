#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2021 Bitcraze AB
#
#  AI-deck demo
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License along with
#  this program; if not, write to the Free Software Foundation, Inc., 51
#  Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
#
#  Demo for showing streamed JPEG images from the AI-deck example.
#
#  By default this demo connects to the IP of the AI-deck example when in
#  Access point mode.
#
#  The demo works by opening a socket to the AI-deck, downloads a stream of
#  JPEG images and looks for start/end-of-frame for the streamed JPEG images.
#  Once an image has been fully downloaded it's rendered in the UI.
#
#  Note that the demo firmware is continously streaming JPEG files so a single
#  JPEG image is taken from the stream using the JPEG start-of-frame (0xFF 0xD8)
#  and the end-of-frame (0xFF 0xD9).

import argparse
import time
import socket
import os
import struct
import threading
import numpy as np
import cv2
from PathFinder import PathFinder
from inference import get_model
import supervision as sv

# Args for setting IP/port of AI-deck. Default settings are for when AI-deck is in AP mode.
parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
parser.add_argument("-n", default="192.168.4.1", metavar="ip", help="AI-deck IP")
parser.add_argument("-p", type=int, default=5000, metavar="port", help="AI-deck port")
parser.add_argument('--save', action='store_true', help="Save streamed images")
args = parser.parse_args()

deck_port = args.p
deck_ip = args.n

class Connect:
    def __init__(self, ip, port, save, line_coords, coords_lock):
        self.bottle_model = get_model(model_id="yolov8n-640")
        self.ip = ip
        self.port = port
        self.save = save
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((self.ip, self.port))
        self.model = PathFinder.init()
        self.model_path = './pathfinder3.pth'
        self.model.load(self.model_path)
        self.count = 0
        self.start = time.time()
        self.running = True
        self.line_coords = line_coords
        self.coords_lock = coords_lock
        self.bottle_count = 0
        print(f"Socket connected to {self.ip}:{self.port}")

    def rx_bytes(self, size):
        data = bytearray()
        while len(data) < size:
            data.extend(self.client_socket.recv(size - len(data)))
        return data

    def fetchImage(self):
        self.start = time.time()
        packetInfoRaw = self.rx_bytes(4)
        length, routing, function = struct.unpack('<HBB', packetInfoRaw)
        imgHeader = self.rx_bytes(length - 2)
        magic, width, height, depth, format, size = struct.unpack('<BHHBBI', imgHeader)

        if magic == 0xBC:
            imgStream = bytearray()
            while len(imgStream) < size:
                packetInfoRaw = self.rx_bytes(4)
                length, dst, src = struct.unpack('<HBB', packetInfoRaw)
                chunk = self.rx_bytes(length - 2)
                imgStream.extend(chunk)
            
            self.count += 1
            meanTimePerImage = (time.time() - self.start) / self.count
            #print(f"Mean time per image: {meanTimePerImage}")
            #print(f"FPS: {1 / meanTimePerImage}")

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8).reshape((244, 324))
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                if self.save:
                    os.makedirs("stream_out/raw", exist_ok=True)
                    os.makedirs("stream_out/debayer", exist_ok=True)
                    cv2.imwrite(f"stream_out/raw/img_{self.count:06d}.png", bayer_img)
                    cv2.imwrite(f"stream_out/debayer/img_{self.count:06d}.png", color_img)
                cv2.waitKey(1)
                return bayer_img
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr, cv2.IMREAD_UNCHANGED)
                cv2.waitKey(1)
                return decoded

    # Update steering vector for the motion_commander
    def update_line_coords(self, image):
        processed_image = self.model.preprocess(image)
        steeringVector = self.model.get_line_coords(processed_image)
        with self.coords_lock:
            self.line_coords[:] = steeringVector
        return processed_image, steeringVector
    
    def detect_bottle(self, image):
        # Ensure the image has 3 channels
        if image.ndim == 2:
            # If the image is grayscale, convert it to 3 channels by replicating the single channel
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 4:
            # If the image has an alpha channel, convert it to 3 channels
            image = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        # Verify the image type and shape
        if image.dtype != np.uint8:
        # If the image is in float32, convert it to uint8 by scaling
            image = (image * 255).astype(np.uint8)

        results = self.bottle_model.infer(image)[0]
        
        detections = sv.Detections.from_inference(results)
        for _, _, confidence, class_id, tracker_id, _ in detections:
            if class_id == 39:
                self.bottle_count += 1

        #print([f"{class_id} {confidence:0.2f}" for _, _, confidence, class_id, _, _ in detections])

        # create supervision annotators
        bounding_box_annotator = sv.BoundingBoxAnnotator()
        label_annotator = sv.LabelAnnotator()
        # annotate the image with our inference results
        annotated_image = bounding_box_annotator.annotate(scene=image, detections=detections)
        annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections)
        # display annotated image
        return annotated_image
    
    def get_bottle_count(self):
        return self.bottle_count

    def display_image(self, fetched_image, processed_image, steeringVector, annotated_image):
        img = np.copy(fetched_image)
        line = np.zeros((img.shape[0], img.shape[1]), dtype=np.float32)
        cv2.line(line, (int(steeringVector[0]), 244), (int(steeringVector[1]), int(steeringVector[2])), [255, 0, 0], 3)
        cv2.addWeighted(img, 0.8, line, 1.0, 0.0, img)
        cv2.imshow('Raw', img)
        cv2.imshow('Processed', processed_image.numpy()[0])
        cv2.imshow('Annotated Image', annotated_image)
        print("Bottle count is : ", self.bottle_count)

    def run(self):
        while self.running:
            fetched_image = np.array(self.fetchImage()).astype(np.float32)
            processed_image, steeringVector = self.update_line_coords(fetched_image)
            annotated_image = self.detect_bottle(fetched_image)
            self.display_image(fetched_image, processed_image, steeringVector, annotated_image)
            #print("Steering vector: ", steeringVector)
            #print("Self line coords: ", self.line_coords)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
                break

        self.client_socket.close()
        cv2.destroyAllWindows()




  

