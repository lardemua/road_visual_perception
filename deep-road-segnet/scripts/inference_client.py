#!/usr/bin/env python2

import zmq
import pyarrow as pa
import numpy as np
import cv2

context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")

A = cv2.imread('/home/bml/images_from_salinas/left0000.jpg')
A = np.array(A)
b = pa.serialize(A).to_buffer().to_pybytes()

print(len(b))

print(A)

socket.send(b)
message = socket.recv()

seg_img = pa.deserialize(message)

cv2.imwrite('sample_img.jpg', seg_img)