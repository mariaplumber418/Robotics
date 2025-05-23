import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import Header
from bresenham import bresenham
import math

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid_node')
        
        # Lidar subscription
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        
        # OccupancyGrid publisher
        self.publisher = self.create_publisher(OccupancyGrid, '/occupancy_grid', 10)

        # Grid settings
        self.grid_width = 100  # 100 cells
        self.grid_height = 100
        self.resolution = 0.1  # 10cm per cell

        # Initialize occupancy grid with unknowns
        self.grid_data = [-1] * (self.grid_width * self.grid_height)

    def lidar_callback(self, msg):
        # Create message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header = Header()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'map'

        occupancy_grid_msg.info.resolution = self.resolution
        occupancy_grid_msg.info.width = self.grid_width
        occupancy_grid_msg.info.height = self.grid_height
        occupancy_grid_msg.info.origin = Pose()  # default (0, 0)

        # Reset grid
        self.grid_data = [-1] * (self.grid_width * self.grid_height)

        # Robot position in the grid (center)
        robot_x = self.grid_width // 2
        robot_y = self.grid_height // 2

        for i, r in enumerate(msg.ranges):
            if r != float('inf') and r > 0:
                angle = msg.angle_min + i * msg.angle_increment
                x = int((r * math.cos(angle)) / self.resolution) + robot_x
                y = int((r * math.sin(angle)) / self.resolution) + robot_y

                # Skip if out of bounds
                if 0 <= x < self.grid_width and 0 <= y < self.grid_height:
                    # Free space: mark cells between robot and obstacle as 0
                    for px, py in bresenham(robot_x, robot_y, x, y):
                        if 0 <= px < self.grid_width and 0 <= py < self.grid_height:
                            index = py * self.grid_width + px
                            self.grid_data[index] = 0

                    # Mark obstacle cell as occupied (100)
                    index = y * self.grid_width + x
                    self.grid_data[index] = 100

        occupancy_grid_msg.data = self.grid_data
        self.publisher.publish(occupancy_grid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
