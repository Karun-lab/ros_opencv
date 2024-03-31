#! /usr/bin python3

import rsopy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

subscriberNodeName='cv_camera'
topicName='image_raw'

def callbackFucntion(message):
	bridgeObject=CvBridge()
	rospy.loginfo("Received video")
	img=bridgeObject.imgmsg_to_cv2(message)
	
	cv2.imshow("Camera",img)
	cv2.waitkey(1)

rospy.init_node(subscriberNodeName  ,anonymous=True)	
rospy.Subscriber(topicName,Image,callbackFunction)
rospy.spin()
cv2.destroyAllWindow()
