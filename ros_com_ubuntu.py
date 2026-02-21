#! /usr/bin/python3.8

import rospy
import numpy as np
import math
import random
from combinatory import Map,Obstacle
from dubin import sample_path

from geometry_msgs.msg import Polygon, Twist
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from obstacles_msgs.msg import ObstacleArrayMsg
from rosgraph_msgs.msg import Clock
from loco_planning.msg import Reference

class Combinatorial:


    def __init__(self):
        
        self.get_map_border = False
        self.get_obstacles = False
        self.get_gates = False
        self.get_road_map = False
        self.get_limo0 = False
        self.get_limo1 = False
        self.get_start_time = False
        self.path_found = False
        self.index = 0

        self.map_border_vertices =  []
        self.obstacles = []
        
        self.sub1 = rospy.Subscriber("/map_borders", Polygon,self.callback)
        self.sub2 = rospy.Subscriber("/obstacles", ObstacleArrayMsg,self.callback2)
        self.sub3 = rospy.Subscriber("/gates", PoseArray,self.callback3)
        self.sub4 = rospy.Subscriber("/limo0/odom", Odometry, self.callback4 )  # Limo 0 initial pose
        self.sub5 = rospy.Subscriber("/limo1/odom", Odometry, self.callback5 ) # Limo 1 initial pose
        self.sub6 = rospy.Subscriber("/clock",Clock, self.callback6)
        self.timer1 = rospy.Timer(rospy.Duration(0.1),self.find_path)
        self.timer2 = rospy.Timer(rospy.Duration(0.1),self.follow_path)

        self.pub1 = rospy.Publisher("/limo0/ref",Reference,queue_size = 10)
        self.pub2 = rospy.Publisher("/limo1/ref",Reference,queue_size = 10)

    
    def callback(self, msg):
        if not self.get_map_border:
            for point in msg.points:
                temp = np.array([point.x,point.y])
                self.map_border_vertices.append(temp)
            self.get_map_border = True
            print("Get map border Done")
            self.sub1.unregister()

    def callback2(self, msg):
        if not self.get_obstacles:
            for obstacle in msg.obstacles:
                vertices = []
                for point in obstacle.polygon.points:
                    temp = np.array([point.x,point.y])
                    vertices.append(temp)
                ob = Obstacle(vertices)
                self.obstacles.append(ob)
            self.get_obstacles = True
            print("Get Obstacles Done")
            self.sub2.unregister()

    def callback3(self,msg):
        if not self.get_gates:
            x = msg.poses[0].orientation.x
            y = msg.poses[0].orientation.y
            z = msg.poses[0].orientation.z
            w = msg.poses[0].orientation.w

            theta = math.atan2(2*(w*z+y*x),1-2*(z*z+y*y))

            self.gates = [msg.poses[0].position.x,msg.poses[0].position.y,theta]
            self.get_gates = True
            print("Get gates Done")
            self.sub3.unregister()

    def callback4(self,msg):
        if not self.get_limo0:
            x = msg.pose.pose.orientation.x
            y = msg.pose.pose.orientation.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w

            theta = math.atan2(2*(w*z+y*x),1-2*(z*z+y*y))

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.limo0 = [x,y,theta]
            self.get_limo0 = True
            print("Get limo0 Done")
            self.sub4.unregister()

    def callback5(self,msg):
        if not self.get_limo1:
            x = msg.pose.pose.orientation.x
            y = msg.pose.pose.orientation.y
            z = msg.pose.pose.orientation.z
            w = msg.pose.pose.orientation.w

            theta = math.atan2(2*(w*z+y*x),1-2*(z*z+y*y))

            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.limo1 = [x,y,theta]
            self.get_limo1 = True

            print("Get limo1 Done")
            self.sub5.unregister()

    def callback6(self,msg):
        if self.path_found:
            if not self.get_start_time:
                self.start_time = msg.clock.secs
                self.get_start_time = True
                print("Get start time done")
            else:
                self.current_time = msg.clock.secs
           
    
    def find_path(self,event):
        if not self.path_found:    
            if (self.get_gates and self.get_limo0 and
            self.get_limo1 and self.get_map_border and
            self.get_obstacles):    
                map = Map(self.map_border_vertices, self.obstacles,self.gates)
                map.find_path(self.limo0, self.limo1, self.gates)
                if not map.limo0_path_found:
                    print("Limo0 path not found")
                if not map.found_limo1_path:
                    print("Limo1 path not found")
                self.limo0_path = map.limo0_path
                self.limo1_path = map.limo1_path
                self.limo0_sample_path = sample_path(self.limo0_path,self.limo0, map.limo_turning_rad/10,map.limo_turning_rad)
                self.limo1_sample_path = sample_path(self.limo1_path,self.limo1, map.limo_turning_rad/10,map.limo_turning_rad)
                self.path_found = True
                print("Find path done")
                self.timer1.shutdown()

    def follow_path(self, event):
        if self.path_found:
            r0 = Reference()
            if self.index < len(self.limo0_sample_path):
                r0.x_d = self.limo0_sample_path[self.index][0] 
                r0.y_d = self.limo0_sample_path[self.index][1] 
                r0.theta_d = self.limo0_sample_path[self.index][2] 
                r0.v_d = 1
                self.pub1.publish(r0)

            r1 = Reference()
            if self.index < len(self.limo1_sample_path):
                r1.x_d = self.limo1_sample_path[self.index][0] 
                r1.y_d = self.limo1_sample_path[self.index][1] 
                r1.theta_d = self.limo1_sample_path[self.index][2]
                r1.v_d = 1
                self.pub2.publish(r1)
            self.index += 1
            


if __name__ == '__main__':
    rospy.init_node('Combinatorial_planning_node')
    Combinatorial()
    rospy.spin()