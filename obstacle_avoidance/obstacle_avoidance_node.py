import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/ttb_lidar/out', self.listener_callback, 10)
        self.get_logger().info('Obstacle avoidance node started')

    def listener_callback(self, msg):
        twist = Twist()
        if min(msg.ranges) < 0.5:  # obstacle detected closer than 0.5m
            twist.angular.z = 0.5  # turn
            twist.linear.x = 0.0
        else:
            twist.linear.x = 0.2  # go forward
            twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
