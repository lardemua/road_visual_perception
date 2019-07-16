#!/usr/bin/env python2

NUM_CLASSES = 19
INPUT_SHAPE = (2048, 1024)
PATH_TO_FROZEN_GRAPH = '/home/bml/catkin_ws/src/deep-road-segnet/models/frozen_inference_graph.pb'

import rospy
import cv2
from sensor_msgs.msg import Image
from functools import partial
from cv_bridge import CvBridge

_cv_bridge = CvBridge()
msg2img = _cv_bridge.imgmsg_to_cv2
img2msg = _cv_bridge.cv2_to_imgmsg

import tensorflow as tf

def run_inference_for_single_image(sess, graph, image):
    # Grab input and output tensors
    image_tensor = graph.get_tensor_by_name('inputs:0')
    output_tensor = graph.get_tensor_by_name('predictions:0')
        # Run inference
    predictions = sess.run(output_tensor,
                               feed_dict={'inputs:0': image})
    return predictions

def callback(output_pub, predictor, msg):
    print("received image")
    img = msg2img(msg)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, INPUT_SHAPE)

    logits = predictor(img)
    logits = logits.astype('uint8')
    logits = logits[0, :, :, 0]
    logits = cv2.resize(logits, INPUT_SHAPE)
    road = (logits == 0).astype('uint8') * 255

    omsg = img2msg(road, encoding="mono8")
    omsg.header = msg.header
    output_pub.publish(omsg)

if __name__ == '__main__':
    rospy.init_node("deep_road_segnet")

    # load graph
    segmentation_graph = tf.Graph()
    with segmentation_graph.as_default():
        segmentaion_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_FROZEN_GRAPH, 'rb') as fid:
            serialized_graph = fid.read()
            segmentaion_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(segmentaion_graph_def, name='')
    
    sess = tf.Session(graph=segmentation_graph)

    seg_pub = rospy.Publisher("segmentation", Image, queue_size=1)

    predictor = partial(run_inference_for_single_image, sess, segmentation_graph)
    callback = partial(callback, seg_pub, predictor)

    rospy.Subscriber("image", Image, callback=callback, queue_size=1)

    rospy.spin()