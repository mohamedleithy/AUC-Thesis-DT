import cv2
import rospy 
from matplotlib import image
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np


bridge = CvBridge()
rospy.init_node('scan_images')
rospy.spin()


def image_publisher():

    try:
        cap = cv2.VideoCapture(0)  # 0 represents the default webcam, change it if necessary
        
        # Check if the webcam is opened correctly
        if not cap.isOpened():
            print("Cannot open webcam")
            exit()

        while True:
            # Read a frame from the webcam
            ret, frame = cap.read()
            print(frame)
            # Check if the frame was read successfully

            pub = rospy.Publisher('/images', Image, queue_size=10)
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(ros_image)
            print("Sent an image!")


        # Release the video capture device and close the window
     
    except:
        print('Err')
        	
	
image_publisher()