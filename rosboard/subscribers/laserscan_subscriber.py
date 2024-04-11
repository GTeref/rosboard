import rosboard.rospy2 as rospy2
from sensor_msgs.msg import LaserScan

class LaserScanSubscriber(object):
    def __init__(self, callback):
        self.callback = callback
        self.subscription = None
    
    def subscribe(self, topic_name):
        self.subscriber = rospy2.Subscriber(
            topic_name,
            LaserScan,
            self.laserscan_callback,
            qos=10
        )
    
    def unregister(self):
        if self.subscriber:
            self.subscriber.unregister()
            self.subscriber = None
        
    def pointcloud2_callback(self, msg):
        self.callback(msg)