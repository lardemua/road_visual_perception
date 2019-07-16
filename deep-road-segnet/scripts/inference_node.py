#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from functools import partial
from cv_bridge import CvBridge
import numpy as np
import zmq
import pyarrow as pa
from matplotlib import cm

_cv_bridge = CvBridge()
msg2img = _cv_bridge.imgmsg_to_cv2
img2msg = _cv_bridge.cv2_to_imgmsg

def callback((seg_pub, road_pub), predictor, msg):
    print("received image")
    img = msg2img(msg)

    (width, height) = img.shape[:2]

    resized = cv2.resize(img, (480, 360))

    print(img.shape)

    labels = predictor(resized)
    road = (labels == 3).astype('uint8') * 255
    road = cv2.resize(road, (height, width), interpolation = cv2.INTER_CUBIC)
    omsg = img2msg(road, encoding="mono8")
    omsg.header = msg.header
    road_pub.publish(omsg)

    cmap = cm.get_cmap('Paired')
    seg = cmap(labels) * 255
    seg = cv2.resize(seg, (height, width), interpolation = cv2.INTER_CUBIC)

    omsg = img2msg((0.6 * img + 0.4 * seg[:,:,:3]).astype('uint8'), encoding="rgb8")
    omsg.header = msg.header
    seg_pub.publish(omsg)

def predictor(socket, image):
    image = np.array(image)

    b = pa.serialize(image).to_buffer().to_pybytes()
    print(len(b))

    socket.send(b)
    message = socket.recv()
    seg_img = pa.deserialize(message)



    return seg_img


if __name__ == '__main__':
    rospy.init_node("deep_road_segnet")

    seg_pub = rospy.Publisher("segmentation", Image, queue_size=1)
    road_pub = rospy.Publisher("road", Image, queue_size=1)

    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    socket.connect("ipc:///tmp/deep")

    predictor = partial(predictor, socket)
    callback = partial(callback, (seg_pub, road_pub), predictor)

    while True:
        img = rospy.wait_for_message("image", Image)
        callback(img)
