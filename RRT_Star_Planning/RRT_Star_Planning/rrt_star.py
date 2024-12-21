
#!/usr/bin/env python3

import sys
import os
import numpy as np
import heapq
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from PIL import Image, ImageOps
import yaml
import matplotlib.pyplot as plt
from copy import copy
import pandas as pd
from graphviz import Graph
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, pow, pi
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from geometry_msgs.msg import PointStamped
from scipy.spatial import KDTree
import time

class MapProcessor():
    def __init__(self,name):
        self.map = Map(name)
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.map_graph = Tree(name)
        self.obstacles = [] 
    
    def get_obstacles(self):
        return self.obstacles

    def __modify_map_pixel(self,map_array,i,j,value,absolute):
        if( (i >= 0) and
            (i < map_array.shape[0]) and
            (j >= 0) and
            (j < map_array.shape[1]) ):
            if absolute:
                map_array[i][j] = value
            else:
                map_array[i][j] += value

    def __inflate_obstacle(self,kernel,map_array,i,j,absolute):
        dx = int(kernel.shape[0]//2)
        dy = int(kernel.shape[1]//2)
        if (dx == 0) and (dy == 0):
            self.__modify_map_pixel(map_array,i,j,kernel[0][0],absolute)
        else:
            for k in range(i-dx,i+dx):
                for l in range(j-dy,j+dy):
                    self.__modify_map_pixel(map_array,k,l,kernel[k-i+dx][l-j+dy],absolute)

    def inflate_map(self,kernel,absolute=True):
        # Perform an operation like dilation, such that the small wall found during the mapping process
        # are increased in size, thus forcing a safer path.
        self.inf_map_img_array = np.zeros(self.map.image_array.shape)
        self.obstacles.clear()  # Clear the list before finding new obstacles
        for i in range(self.map.image_array.shape[0]):
            for j in range(self.map.image_array.shape[1]):
                if self.map.image_array[i][j] == 1:
                    self.obstacles.append((i, j))
                    self.__inflate_obstacle(kernel,self.inf_map_img_array,i,j,absolute)
        r = np.max(self.inf_map_img_array)-np.min(self.inf_map_img_array)
        if r == 0:
            r = 1
        
        self.inf_map_img_array = (self.inf_map_img_array - np.min(self.inf_map_img_array))/r

class Map:
    def __init__(self, map_name):
        self.map_im, self.map_df, self.limits = self.__open_map(map_name)
        if self.map_im is not None:
            self.image_array = self.__get_obstacle_map(self.map_im, self.map_df)
        else:
            self.image_array = None

    def __open_map(self, map_name):
        package_share_path = get_package_share_directory('turtlebot3_gazebo')
        yaml_path = os.path.join(package_share_path, 'maps', f"{map_name}.yaml")
        try:
            with open(yaml_path, 'r') as f:
                map_df = pd.json_normalize(yaml.safe_load(f))
        except FileNotFoundError:
            print(f"File {yaml_path} not found.")
            return None, None, None

        map_image_path = os.path.join(package_share_path, 'maps', map_df.image[0])
        try:
            im = Image.open(map_image_path)
        except FileNotFoundError:
            print(f"File {map_image_path} not found.")
            return None, None, None
        size = 200, 200
        im.thumbnail(size)
        im = ImageOps.grayscale(im)
        
        xmin = map_df.origin[0][0]
        xmax = map_df.origin[0][0] + im.size[0] * map_df.resolution[0]
        ymin = map_df.origin[0][1]
        ymax = map_df.origin[0][1] + im.size[1] * map_df.resolution[0]

        return im, map_df, [xmin, xmax, ymin, ymax]

    def __get_obstacle_map(self, map_im, map_df):
        img_array = np.array(map_im)
        obstacle_thresh = map_df.occupied_thresh[0] * 255
        binary_map = np.where(img_array <= obstacle_thresh, 1, 0) 

        return binary_map

    def world_to_pixel(self, x, y):
        scale_x = 200 / 14.7 
        scale_y = 139 / 10.36  
    
       
        pixel_x = int((x + 5.3) * scale_x)  
        pixel_y = int((4.2 - y) * scale_y)  
    
        # Clamping
        # pixel_x = max(0, min(pixel_x, self.image_array.shape[1] - 1))
        # pixel_y = max(0, min(pixel_y, self.image_array.shape[0] - 1))
    
        return pixel_x, pixel_y

    def pixel_to_world(self, x, y):
        scale_x = 200 / 14.7
        scale_y = 139 / 10.36
    
        # Convert pixel coordinates back to world coordinates
        world_x = (x / scale_x) - 5.3  
        world_y = 4.2 - (y / scale_y)  
    
        return world_x, world_y




class GraphNode():
    def __init__(self,name):
        self.name = name
        self.children = []
        self.weight = []

    def __repr__(self):
        return self.name

    def add_children(self,node,w=None):
        if w == None:
            w = [1]*len(node)
        self.children.extend(node)
        self.weight.extend(w)

class Tree():
    def __init__(self,name):
        self.name = name
        self.root = 0
        self.end = 0
        self.g = {}
        self.g_visual = Graph('G')

    def __call__(self):
        for name,node in self.g.items():
            if(self.root == name):
                self.g_visual.node(name,name,color='red')
            elif(self.end == name):
                self.g_visual.node(name,name,color='blue')
            else:
                self.g_visual.node(name,name)
            for i in range(len(node.children)):
                c = node.children[i]
                w = node.weight[i]
             
                if w == 0:
                    self.g_visual.edge(name,c.name)
                else:
                    self.g_visual.edge(name,c.name,label=str(w))
        return self.g_visual

    def add_node(self, node, start = False, end = False):
        self.g[node.name] = node
        if(start):
            self.root = node.name
        elif(end):
            self.end = node.name

    def set_as_root(self,node):
      
        self.root = True
        self.end = False

    def set_as_end(self,node):
       
        self.root = False
        self.end = True

class RRT:
    def __init__(self, map_processor, start, goal, step_size=100, max_iter=1000):
        self.map_processor = map_processor
        self.start = start
        self.goal = goal
        self.step_size = step_size
        self.max_iter = max_iter
        self.tree = KDTree(np.array([self.start], dtype=float))
        self.nodes = [start]
        self.parents = {start: None}
        self.x_bounds = (0, 140)
        self.y_bounds = (0, 200)
        self.obstacles = map_processor.get_obstacles()

    def build(self):
        for i in range(self.max_iter):
            rand_point = np.random.uniform(*self.x_bounds), np.random.uniform(*self.y_bounds)
            print(f"Random point: {rand_point}")
            
            nearest_node = self.nearest(rand_point)
            new_node = self.step_towards(nearest_node, rand_point, self.step_size)
            
            if self.is_collision_free(new_node, nearest_node, self.obstacles):
                self.nodes.append(new_node)
                self.tree = KDTree(np.array(self.nodes, dtype=float))
                self.parents[new_node] = nearest_node
                print(f"Iteration {i}: Added node {new_node}")

                if self.distance(new_node, self.goal) <= self.step_size:
                    if self.is_collision_free(new_node, self.goal, self.obstacles):
                        self.parents[self.goal] = new_node
                        self.nodes.append(self.goal)
                        print("Goal reached!")
                        return self.extract_path(self.goal)
            else:
                print(f"Iteration {i}: Collision detected for node {new_node}")
        
        return None

    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def is_collision_free(self, new_node, nearest_node, obstacles):
        for (ox, oy) in obstacles:
            if np.linalg.norm(np.array([ox, oy]) - np.array(new_node)) < 5:
                return False
        return True

    def nearest(self, point):

        _, idx = self.tree.query(np.array(point))  
        return tuple(self.tree.data[idx]) 

    def step_towards(self, from_node, to_node, step_size=1.0):

        vector = np.array(to_node) - np.array(from_node)
        length = np.linalg.norm(vector)
        direction = vector / length
        new_node = np.array(from_node) + direction * min(step_size, length)
        return tuple(new_node)  

    def extract_path(self, goal):
        print("Extracting Path")
        path = []
        current = goal
        while current is not None:
            path.append(current)
            current = self.parents.get(current)
        path.reverse()
        return path


class Navigation(Node):
    def __init__(self, node_name='Navigation'):
        super().__init__(node_name)
        self.map_processor = MapProcessor('map')
        kernel = np.ones((10, 10))  # Define a basic kernel for obstacle inflation
        self.map_processor.inflate_map(kernel)

        self.path_pub = self.create_publisher(Path, 'rrt_star_plan', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pose = PoseStamped()
        self.ttbot_pose = PoseStamped()
        self.map = self.map_processor.map

        self.path=[]
        self.current_path_index=0
        self.previous_angle_error = 0.0
        self.last_time = self.get_clock().now()

        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.__goal_pose_cbk, 10)
        
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.__ttbot_pose_cbk, 10)
        self.i_angle_error=0

    def __goal_pose_cbk(self, data):
        self.goal_pose = data
        self.get_logger().info(f"Received goal pose: ({self.goal_pose.pose.position.x}, {self.goal_pose.pose.position.y})")
        self.compute_path()

    def __ttbot_pose_cbk(self, data):
        self.ttbot_pose.pose = data.pose.pose
        self.follow_path()

    def compute_path(self):
        start_x,start_y = self.map.world_to_pixel(
            self.ttbot_pose.pose.position.x, self.ttbot_pose.pose.position.y)
        end_x,end_y= self.map.world_to_pixel(
            self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        
        start_key= start_y,start_x
        end_key=end_y,end_x
        self.rrt = RRT(self.map_processor, start_key, end_key, step_size=5, max_iter=1000)
        path = self.rrt.build()

        if path:
            self.publish_path(path)
            self.get_logger().info("PATH FOUND!")
        else:
            self.get_logger().info('No path found')


    def publish_path(self, path):
        nav_path = Path()
        nav_path.header.stamp = self.get_clock().now().to_msg()
        nav_path.header.frame_id = "map"

        for node in path:
            y, x = node
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x, pose.pose.position.y = self.map_processor.map.pixel_to_world(x, y)
            pose.pose.orientation.w = 1.0
            nav_path.poses.append(pose)

        self.path_pub.publish(nav_path)
        self.get_logger().info("Published RRT* path.")
        self.current_path_index=0
        self.path=nav_path.poses
        self.follow_path()
        
    
    def follow_path(self):
         
        if hasattr(self, 'path') and self.current_path_index < len(self.path):
            current_goal = self.path[self.current_path_index]
            error_x = current_goal.pose.position.x - self.ttbot_pose.pose.position.x
            error_y = current_goal.pose.position.y - self.ttbot_pose.pose.position.y

            distance = sqrt(pow(error_x, 2) + pow(error_y, 2))
            if distance < 0.2:  #Threshold so that it doesnt overcprrect
                self.current_path_index += 1
                if self.current_path_index >= len(self.path):
                    self.get_logger().info("Reached the end of the path.")
                    vel_msg = Twist()
                    self.vel_pub.publish(vel_msg)  # Stop 
                    return

            # Calculating.... the angle to the goal
            angle_to_goal = atan2(error_y, error_x)
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            angle_error = self.normalize_angle(angle_to_goal - self.get_yaw_from_quaternion(self.ttbot_pose.pose.orientation))
            d_angle_error = (angle_error - self.previous_angle_error) / dt if dt > 0 else 0
            self.previous_angle_error = angle_error
            self.i_angle_error += angle_error

            # if self.i_angle_error>=2:
            #     self.i_angle_error=0


            kp = 5.0
            kd = 0.5
            ki = 0.000008

            angular_velocity = kp * angle_error + ki*self.i_angle_error + kd * d_angle_error
            angular_velocity = max(-1.0, min(1.0, angular_velocity))  

            # Reduce linear speed when the angle error is large
            linear_velocity = 0.3 if abs(angle_error) < 0.05 else 0.05
          

            vel_msg = Twist()
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.vel_pub.publish(vel_msg)

    def normalize_angle(self, angle):

        while angle > pi:
            angle -= 2 * pi
        while angle < -pi:
            angle += 2 * pi
        return angle

    def get_yaw_from_quaternion(self, q):

        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

def main(args=None):
    rclpy.init(args=args)
    nav = Navigation()
    rclpy.spin(nav)
    nav.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main() 

