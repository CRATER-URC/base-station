import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('teleop')

        self.speed_setting = 2.0 # default to medium speed
        self.operating_mode = 0 # 0 for Stopped, 1 for Driving

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.on_joy,
            1)
        
        self.publisher_L = self.create_publisher(Float32, 'vel_L', 1)
        self.publisher_R = self.create_publisher(Float32, 'vel_R', 1)

        # self.publisher_ = self.create_publisher(String, 'topic', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    def on_joy(self, data):
        # Set speed ratio using d-pad
        if data.axes[7] == 1: # full speed (d-pad up)
            self.speed_setting = 1.0
        if data.axes[6] != 0: # medium speed (d-pad left or right)
            self.speed_setting = 2.0
        if data.axes[7] == -1: # low speed (d-pad down)
            self.speed_setting = 3.0

        # Drive sticks
        left_speed = -data.axes[1] / self.speed_setting # left stick
        right_speed = -data.axes[4] / self.speed_setting # right stick

        # Convert skid steering speeds to twist speeds
        # linear_vel  = (left_speed + right_speed) / 2.0 # (m/s)
        # angular_vel  = (right_speed - left_speed) / 2.0 # (rad/s)

        # self.servo1_pub.publish(servo1_msg)

        # Cancel move base goal
        # if data.buttons[2]: # X button
        #     self.operating_mode == 0
        #     self.get_logger().info('Stopping Movement')
            # rospy.loginfo('Cancelling move_base goal')
            # cancel_msg = GoalID()
            # self.goal_cancel_pub.publish(cancel_msg)

        # if data.buttons[3]: # Y button
        #     self.operating_mode == 0
        #     self.get_logger().info('Changed to Driving Mode')

        # Cancel move base goal
        if data.buttons[1]: # B button
            self.operating_mode = 0
            self.get_logger().info('Stopping Movement')
            # rospy.loginfo('Cancelling move_base goal')
            # cancel_msg = GoalID()
            # self.goal_cancel_pub.publish(cancel_msg)

        if data.buttons[7]: # Start button
            self.operating_mode = 1
            self.get_logger().info('Changed to Driving Mode')

        left = Float32()
        right = Float32()
        # Publish Speeds
        if self.operating_mode == 1:
            left.data = left_speed
            right.data = right_speed
        else:
            left.data = 0.0
            right.data = 0.0
        self.publisher_L.publish(left)
        self.publisher_R.publish(right)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()