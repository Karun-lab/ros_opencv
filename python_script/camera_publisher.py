#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge

publisherNodeName = 'camera_sensor_publisher'
topicName = 'video_topic'

rospy.init_node(publisherNodeName, anonymous=True)
publisher = rospy.Publisher(topicName, Image, queue_size=60)
rate = rospy.Rate(30)

videoCaptureObject = cv2.VideoCapture(4)  # 0 for laptop cam and 4 for webcam
bridgeObject = CvBridge()

while not rospy.is_shutdown():
    returnValue, capturedFrame = videoCaptureObject.read()

    # Compress the captured frame
    _, buffer = cv2.imencode(".jpg", capturedFrame, [int(cv2.IMWRITE_JPEG_QUALITY), 30])

    if returnValue:
        rospy.loginfo('Video frame captured and compressed')
        # Convert the compressed buffer to bytes
        compressed_data = buffer.tobytes()
        
        # Publish the compressed image
        #imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame)
        #imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="passthrough")
        imageToTransmit = bridgeObject.cv2_to_imgmsg(capturedFrame, encoding="bgr8")


        imageToTransmit.data = compressed_data  # Set the image data to the compressed bytes
        publisher.publish(imageToTransmit)

    rate.sleep()

videoCaptureObject.release()
cv2.destroyAllWindows()

