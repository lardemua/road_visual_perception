import rosbag
bag = rosbag.Bag('OAtlasVaiAFeiraSoCamera.bag')
for topic, msg, t in bag.read_messages(topics=['camera/image_rect_color']):
    print msg
bag.close()