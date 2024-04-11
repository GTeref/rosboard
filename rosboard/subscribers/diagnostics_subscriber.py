import rosboard.rospy2 as rospy2
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray

class DiagnosticsSubscriber(Node):
    def __init__(self, callback):
        super().__init__('diagnostics_subscriber')
        self.callback = callback
        self.subscription = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )

    # def subscribe(self, topic_name):
    #     self.subscriber = rospy2.Subscriber(
    #         topic_name,
    #         DiagnosticArray,
    #         self.pointcloud2_callback,
    #         qos=10
    #     )
    
    # def unregister(self):
    #     if self.subscription:
    #         self.subscription.destroy()
    #         self.subscription = None
    
    def diagnostics_callback(self, msg):
        self.callback(msg)
