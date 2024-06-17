#!/usr/bin/env python3

import rclpy
import numpy as np
from sklearn.cluster import DBSCAN
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
import math

GRID_CORNERS = [(-5, -5), (-5, 5), (5, 5), (5, -5), (-5, -5)]
GRID_SIZE_X = 50
GRID_SIZE_Y = 50

# Define the positions of obstacle centers for each environment
environment_centers = {
    'A': [(2, 2), (-2, 2), (2, -2)],
    'B': [(2, 2), (2, -2), (-2, -2), (-2, 2)],
    'C': [(0, 0)],
    'D': [(2, 2), (-2, 2), (2, -2), (0, 2), (2, 0)],
    'E': [(2, 2), (2, -2), (-2, -2), (-2, 2), (0, 0)]
}

class Map_cell:
    def __init__(self):
        self.log_odds = 0  # Initialize with a default log odds, which corresponds to a 0.5 probability

class Robot:
    def __init__(self):
        self.x = None
        self.y = None
        self.angle = None

class Map(Node):
    def __init__(self):
        super().__init__("localize_node")
        self.my_robot = Robot()
        self.obstacles = []
        self.my_map = [[Map_cell() for _ in range(GRID_SIZE_Y)] for _ in range(GRID_SIZE_X)]
        self.create_subscription(LaserScan,"/scan",self.scan_callback,100) #2 hz
        self.create_subscription(Pose2D,"/pose",self.pose_callback,100) #10 hz
        self.og_pub = self.create_publisher(OccupancyGrid, "/occupancy_grid",10)
        self.env_pub = self.create_publisher(String, "/environment", 10)
        self.create_timer(1,self.publish_map)
    
    def publish_map(self):
        og_publish = OccupancyGrid()
        og_publish.header.stamp = self.get_clock().now().to_msg()
        og_publish.header.frame_id = "world"
        og_publish.info.resolution = 0.2
        og_publish.info.width = GRID_SIZE_X
        og_publish.info.height = GRID_SIZE_Y
        og_publish.info.origin.position.x = -5.0
        og_publish.info.origin.position.y = -5.0
        og_publish.info.origin.position.z = 0.0
        og_publish.info.origin.orientation.x = 0.7071068
        og_publish.info.origin.orientation.y = 0.7071068
        og_publish.info.origin.orientation.z = 0.0
        og_publish.info.origin.orientation.w = 0.0

        og_array = []
        for i in range(GRID_SIZE_X):
            for j in range(GRID_SIZE_Y):
                log_odds = max(min(self.my_map[i][j].log_odds, 100), -100)
                prob = 1.0 / (1.0 + math.exp(-log_odds))  # Convert from log odds to probability
                og_array.append(int(prob * 100))

        og_publish.data = og_array
        self.og_pub.publish(og_publish)

    def pose_callback(self, msg: Pose2D):
        self.my_robot.x = msg.x
        self.my_robot.y = msg.y
        self.my_robot.angle = (msg.theta + math.pi) % (2 * math.pi) - math.pi

    def scan_callback(self, msg: LaserScan):
        self.obstacles.clear()
        min_angle = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges
        robot_loc = [self.my_robot.x, self.my_robot.y]
        angle = min_angle
        for ray in ranges:
            if not math.isnan(ray):
                ray_angle = self.my_robot.angle + angle
                if math.isinf(ray):
                    # Infinite rays should update cells along the ray as free
                    end_x = robot_loc[0] + math.cos(ray_angle) * 5
                    end_y = robot_loc[1] + math.sin(ray_angle) * 5
                else:
                    end_x = robot_loc[0] + math.cos(ray_angle) * ray
                    end_y = robot_loc[1] + math.sin(ray_angle) * ray

                start = self.pose_to_grid(robot_loc[0], robot_loc[1])
                end = self.pose_to_grid(end_x, end_y)
                grid_list = self.bresenham(start[0], start[1], end[0], end[1])
                self.update_grid(grid_list, math.isinf(ray))

            angle += angle_increment
        
        for i in range(4,GRID_SIZE_X-4):
            for j in range(4,GRID_SIZE_Y-4):
                temp = self.my_map[i][j].log_odds
                prob = 1.0 / (1.0 + math.exp(-temp))
                if prob>0.8:
                    x,y = self.grid_to_pose(i,j)
                    self.obstacles.append([x,y])
        
        ogroups, centroids = self.DBscan(self.obstacles)
        #print(ogroups,centroids,flush=True)
        if not centroids == None: 
            matching_environments = {env: centers for env, centers in environment_centers.items() if len(centers) == len(centroids)}
        
            if matching_environments:
                # Calculate average distance between centroids and closest obstacles for each environment
                average_distances = {}
                for env, centers in matching_environments.items():
                    total_distance = 0
                    for centroid in centroids:
                        min_distance = min(self.distance(centroid, center) for center in centers)
                        total_distance += min_distance
                    average_distances[env] = total_distance / len(centroids)

                best_match_environment = min(average_distances, key=average_distances.get)

                # Publish the best match environment
                env_msg = String()
                env_msg.data = f"Best match: {best_match_environment}"
                #print(env_msg.data,flush=True)
                if average_distances[best_match_environment]>1.5:
                    no_match_msg = String()
                    no_match_msg.data = "No matching environments found."
                    self.env_pub.publish(no_match_msg)
                else:
                    self.env_pub.publish(env_msg)
            else:
                no_match_msg = String()
                no_match_msg.data = "No matching environments found."
                self.env_pub.publish(no_match_msg)
        else:
                no_match_msg = String()
                no_match_msg.data = "No matching environments found."
                self.env_pub.publish(no_match_msg)

    
    #db scan
    def DBscan(self, obstacle_list):
        points = np.array([[p[0], p[1]] for p in obstacle_list])

        if len(points) < 2:
            print('Not enough points for clustering.')
            return [], None

        db_eps = 0.8
        db_min_samples = 7

        clustering = DBSCAN(eps=db_eps, min_samples=db_min_samples).fit(points)
        labels = clustering.labels_
        centroids = []
        groups = []
        for cluster_id in set(labels):
            if cluster_id == -1:
                continue

            cluster_points = points[labels == cluster_id]
            groups.append(cluster_points)

            centroid = np.mean(cluster_points, axis=0)
            centroids.append([centroid[0], centroid[1]])

        return groups, centroids

    def distance(self,point1, point2):
        return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

    def update_grid(self, grids, is_infinite):
        
        free_log_odds = math.log(0.25 / 0.75)  # Example values for free space
        occ_log_odds = math.log(0.92 / 0.08)   # Example values for occupied space
        for i, grid in enumerate(grids):
            if i == len(grids) - 1 and not is_infinite:
                self.my_map[grid[0]][grid[1]].log_odds += occ_log_odds
            else:
                self.my_map[grid[0]][grid[1]].log_odds += free_log_odds

    def pose_to_grid(self, x, y):
        grid_x = int((x - GRID_CORNERS[0][0]) / (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) * (GRID_SIZE_X - 1))
        grid_y = int((y - GRID_CORNERS[0][1]) / (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) * (GRID_SIZE_Y - 1))
        return max(0, min(grid_x, GRID_SIZE_X - 1)), max(0, min(grid_y, GRID_SIZE_Y - 1))
    
    def grid_to_pose(self, grid_x, grid_y):
        cell_width = (GRID_CORNERS[2][0] - GRID_CORNERS[0][0]) / (GRID_SIZE_X - 1)
        cell_height = (GRID_CORNERS[2][1] - GRID_CORNERS[0][1]) / (GRID_SIZE_Y - 1)

        x = GRID_CORNERS[0][0] + grid_x * cell_width
        y = GRID_CORNERS[0][1] + grid_y * cell_height

        return x, y

    def bresenham(self, x1, y1, x2, y2):
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy
        x, y = x1, y1

        line_points = []
        while True:
            line_points.append((x, y))
            if x == x2 and y == y2:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return line_points

def main(args=None):
    rclpy.init(args=args)
    node = Map()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()