#! /usr/bin/env python
from matplotlib import image
import rospy
from sensor_msgs.msg import Image
import cv2
import numpy as np
def image_callback(msg):
    print("Received an image!")
    try:
        pub = rospy.Publisher('/images', Image, queue_size=10)
        # im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
	# Save your OpenCV2 image as a jpeg
        # image_path = 'camera_image.jpeg'
        # cv2.imwrite(image_path, cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
        pub.publish(msg)
        print("Sent an image!")
    except:
        print('Err')
        	
	
rospy.init_node('scan_images')
sub = rospy.Subscriber('/raspicam_node/image',Image,image_callback)
rospy.spin()
