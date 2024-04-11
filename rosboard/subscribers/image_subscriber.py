import rosboard.rospy2 as rospy2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Don't use this to receive videos, only images
# OpenCV streaming is slow!

class ImageSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.subscriber = None
        self.bridge = CvBridge()

    def subscribe(self, topic_name):
        self.subscriber = rospy2.Subscriber(
            topic_name,
            Image,
            self.image_callback,
            queue_size=10
        )

    def unregister(self):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.callback(cv_image)
        except Exception as e:
            rospy2.logerr("Error converting image: %s" % str(e))