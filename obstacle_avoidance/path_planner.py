import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import heapq

class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/occupancy_grid',
            self.occupancy_grid_callback,
            10
        )
        self.publisher = self.create_publisher(Path, '/planned_path', 10)

        # Set start and goal
        self.start = (1, 1)   # in grid coordinates
        self.goal = (15, 15)

    def occupancy_grid_callback(self, msg):
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        grid = list(msg.data)

        def is_valid(x, y):
            return 0 <= x < width and 0 <= y < height and grid[y * width + x] != 100

        def heuristic(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])

        def a_star(start, goal):
            open_set = []
            heapq.heappush(open_set, (0, start))
            came_from = {}
            g_score = {start: 0}

            while open_set:
                _, current = heapq.heappop(open_set)
                if current == goal:
                    path = []
                    while current in came_from:
                        path.append(current)
                        current = came_from[current]
                    path.append(start)
                    path.reverse()
                    return path

                for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
                    neighbor = (current[0] + dx, current[1] + dy)
                    if not is_valid(*neighbor):
                        continue
                    tentative_g = g_score[current] + 1
                    if neighbor not in g_score or tentative_g < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g
                        f = tentative_g + heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f, neighbor))
            return []

        path = a_star(self.start, self.goal)

        if path:
            self.get_logger().info(f'Path found with {len(path)} steps.')
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            for x, y in path:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = x * resolution
                pose.pose.position.y = y * resolution
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
            self.publisher.publish(path_msg)
        else:
            self.get_logger().info('No path found.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
