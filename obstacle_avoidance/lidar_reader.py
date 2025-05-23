import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  #prevent unused variable warning

    def listener_callback(self, msg):
        # Print the distance straight ahead (at 0 degrees)
        center_index = len(msg.ranges) // 2
        distance_ahead = msg.ranges[center_index]
        self.get_logger().info(f'Distance ahead: {distance_ahead:.2f} meters')

def main(args=None):
    rclpy.init(args=args)
    node = LidarReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
