import rosboard.rospy2 as rospy2
from sensor_msgs.msg import Joy 

class JoySubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.subscriber = None

    def subscribe(self, topic_name):
        self.subscriber = rospy2.Subscriber(
            topic_name,
            Joy,
            self.joy_callback,
            queue_size=10
        )
    
    def unregister(self):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None
    
    def joy_callback(self, msg):
        self.callback(msg)