import rclpy
import traceback

from math import pi, sin, cos, atan2, sqrt

from rclpy.node         import Node
from geometry_msgs.msg  import PoseStamped

#
#   Example Node Class
#
class MoveNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)
        
        # Subscribers
        self.subexample = self.create_subscription(
            PoseStamped, '/example_msg', self.cb_examplemsg, 10)
        
        # Publishers
        self.pubexample = self.create_publisher(PoseStamped, 'cmd_example', 10)
        
        # Report and return.
        self.get_logger().info("Example running")
            
    # Shutdown
    def shutdown(self):
        # Nothing to do except shut down the node.
        self.destroy_node()
        
    def cb_examplemsg(self, msg):
        msg = msg

#
#   Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Instantiate the Move node.
    node = ExampleNode('example')

    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
