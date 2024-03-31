import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

subscriberNodeName = 'camera_sensor_subscriber'
topicName = 'video_topic'

def callbackFunction(message):
    bridgeObject = CvBridge()

    try:
        # Extract compressed data from the received message
        compressed_data = message.data
        np_arr = np.frombuffer(compressed_data, np.uint8)

        # Decompress the data
        decompressed_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        rospy.loginfo("Received a video")
        cv2.imshow("camera", decompressed_frame)
        cv2.waitKey(1)
    except Exception as e:
        rospy.logerr("Error processing image: %s", str(e))

rospy.init_node(subscriberNodeName, anonymous=True)
rospy.Subscriber(topicName, Image, callbackFunction)
rospy.spin()
cv2.destroyAllWindows()

