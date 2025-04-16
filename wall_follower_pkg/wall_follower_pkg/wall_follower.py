import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.cmd_vel_pub = self.create_publisher(Twist, '/gabe_drive/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/gabe_drive/scan', self.scan_callback, 10)

    def scan_callback(self, msg):
        # Get the front distance from the single laser sensor (center of the scan)
        front_distance = msg.ranges[0]  # Only has a single sensor
        side_distance = msg.ranges[1] 

        print ("Front distance:", front_distance)
        print ("Side distance:", side_distance)

        self.get_logger().info(f"Front Distance: {front_distance} m")

        twist = Twist()

        if (front_distance < 3.0):  # Too close, turn
            twist.angular.z = -2.0  # Turn to avoid collision
        elif (front_distance > 3.0) and (side_distance < 2.5):
            twist.linear.x = 10.0  # Move forward
        elif (side_distance > 2.5):
            twist.linear.x = 1.0  # Move forward
            twist.angular.z = 0.3

        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
