#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
body_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_fullbody.xml')

def callbackFunction(message):
    bridgeObject = CvBridge()

    try:
        # Extract compressed data from the received message
        compressed_data = message.data
        np_arr = np.frombuffer(compressed_data, np.uint8)

        # Decompress the data
        decompressed_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        rospy.loginfo("Received a video")

        # Convert the image to grayscale
        gray = cv2.cvtColor(decompressed_frame, cv2.COLOR_BGR2GRAY)

        # Check if the cascade classifiers are loaded successfully
        if face_cascade.empty() or body_cascade.empty():
            rospy.logerr("Cascade classifiers not loaded")
            return

        # Detect faces and full bodies in the grayscale image
        faces = face_cascade.detectMultiScale(gray, 1.2, 5)
        bodies = body_cascade.detectMultiScale(gray, 1.2, 5)

        # Draw rectangles around the detected faces
        for (x, y, w, h) in faces:
            cv2.rectangle(decompressed_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)

        # Draw rectangles around the detected bodies
        for (x, y, w, h) in bodies:
            cv2.rectangle(decompressed_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

        cv2.imshow("camera", decompressed_frame)
        cv2.waitKey(1)
        
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackFunction)
rospy.spin()
cv2.destroyAllWindows()

