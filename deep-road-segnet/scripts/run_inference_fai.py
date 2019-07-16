#!/usr/bin/env python3

import time
import torch
import cv2
import numpy as np
import zmq
import pyarrow as pa

NORMALIZATION = ([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])

def preprocess(image):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image = image.astype('float32') / 255.0
    # image = cv2.resize(image, (480, 360))

    (w, h, c) = image.shape
    image = image.reshape(1, w, h, c)
    image = (image - NORMALIZATION[0]) / NORMALIZATION[1]
    image = torch.tensor(image, dtype=torch.float32)
    return image.permute(0, 3, 1, 2)

def predict(model, image):
    tensor = preprocess(image)
    y = model(tensor).detach().numpy()

    labels = np.argmax(y[0], axis=0).astype('uint8')

    # road_seg = (labels == 3).astype('uint8') * 255

    return labels


if __name__ == '__main__':

    model = torch.load('/home/tmr-algorithms/catkin_ws/src/deep-road-segnet/models/unet/model.pt', map_location='cpu')
    model.eval()
    

    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("ipc:///tmp/deep")

    while True:
        message = socket.recv()
        print(len(message))

        image = pa.deserialize(message)


        classified_img = predict(model, image)

        message = pa.serialize(classified_img).to_buffer().to_pybytes()

        socket.send(message)


    # X = cv2.imread('/home/bml/images_from_salinas/left0000.jpg')

    # start = time.time()

    # for i in range(1):
    #     y = predict(model, X)

    # end = time.time()

    # y = y.detach().numpy()

    # labels = np.argmax(y[0], axis=0).astype('uint8')
    # colorized = cv2.applyColorMap(labels * 20, cv2.COLORMAP_JET)
    # print(colorized.shape)

    # # road_seg = (labels == 3) * 200

    # cv2.imwrite('sample_img.jpg', colorized)

    # print(end - start)
