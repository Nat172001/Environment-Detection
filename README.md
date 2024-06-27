# Environment-Detection
This project implements a ROS 2 node for map localization and environment identification using LaserScan data. The node processes laser scan data to detect obstacles, update an occupancy grid map, and identify the environment based on predefined obstacle configurations. The grid has a size of 50x50 and covers the area defined by corners (-5, -5) to (5, 5). 

The positions of obstacle centers for each environment are defined as follows: 
Environment A: [(2, 2), (-2, 2), (2, -2)]; 
Environment B: [(2, 2), (2, -2), (-2, -2), (-2, 2)]; 
Environment C: [(0, 0)]; 
Environment D: [(2, 2), (-2, 2), (2, -2), (0, 2), (2, 0)]; 
Environment E: [(2, 2), (2, -2), (-2, -2), (-2, 2), (0, 0)]. 

The node creates and publishes an occupancy grid map based on laser scan data, detects and clusters obstacles using the DBSCAN algorithm, and identifies the environment by comparing detected obstacle clusters to predefined configurations. It integrates with ROS 2 by subscribing to laser scan and robot pose topics, and publishing the occupancy grid and environment identification results.






