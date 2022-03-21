#!/usr/bin/python3

import time
from turtle import goto

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point
from task1.msg import ObjectDetection, PointArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math


class GoalSetter:
    
    def __init__(self) -> None:
        rospy.init_node("goal_setter")
        self.simpleAction = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("face_points", PointArray, self.face_points_callback)
        
        self.x_axis, self.y_axis, self.angle_r, self.angular_speed_r, self.angle = 0,0,0,0,0
        self.location = Point()
        
        self.goals =[["explore_goal", 0.89, -1.83],
            ["explore_goal", -0.93, -1.7],
            ["explore_goal", -2.16, -0.34],
            ["explore_goal", -1.0, 0.19],
            ["explore_goal", -1.82, 1.44],
            ["explore_goal", -2.37, 2.24],
            ["explore_goal", -0.7, 2],
            ["explore_goal", 1.3, 1.8],
            ["explore_goal", 1.5, 0.5],
            ["explore_goal", 0.1, 1],
            ["explore_goal", 0.18, -0.4],
            ["explore_goal", 1.6, -1],
            ["explore_goal", 2.6, -1.4]]
        
        self.exeTimeout = rospy.Duration(20)
        self.preemptTimeout = rospy.Duration(10)
        
        self.facePoints = PointArray()
        time.sleep(1)
        
        
    def pose_callback(self, neki: PoseWithCovarianceStamped):
        self.location = neki.pose.pose.position
        orientation = neki.pose.pose.orientation
        self.angle = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2] + math.pi
        
    def face_points_callback(self, points: PointArray):
        if (len(points.points) > len(self.facePoints.points)):
            new_face_pose = self.calculate_approaching_point(points.points[len(self.facePoints.points)])
            self.goals.insert(0, new_face_pose)
            self.facePoints.points.append(new_face_pose)
            
    
    def calculate_approaching_point(self, face_point):
        dist = math.dist((face_point.x, face_point.y), (self.location.x, self.location.y))
        ratio = (dist - 0.4) / dist
        distanceVector = [face_point.x - self.location.x, face_point.y - self.location.y]
        goal_coor = [distanceVector[0] * ratio, distanceVector[1] * ratio]
        approach_point = ["approach_face", self.location.x + goal_coor[0],self.location.y + goal_coor[1]]
        return approach_point
    
    
    def do_rotate_goal(self, goal_):
        print("rotate_goal")
        angles = [0, 2*math.pi/3, 4*math.pi/3, 0]
        start_angle = self.angle
        rat = rospy.Rate(2)

        for i in range(3):
            goal = MoveBaseGoal()
            q = quaternion_from_euler(0, 0, (angles[i]+start_angle)%(2*math.pi))
            
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            goal.target_pose.pose.position.x = goal_[1]
            goal.target_pose.pose.position.y = goal_[2]
            goal.target_pose.header.stamp = rospy.Time.now()
            self.simpleAction.send_goal_and_wait(goal, self.exeTimeout, self.preemptTimeout)
            rat.sleep()
        return
    
    
    def do_explore_goal(self, goal_):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position.x = goal_[1]
        goal.target_pose.pose.position.y = goal_[2]
        goal.target_pose.header.stamp = rospy.Time.now()
        self.simpleAction.send_goal_and_wait(goal, self.exeTimeout, self.preemptTimeout)
        self.do_rotate_goal(goal_)
        return
        
        
    def do_approach_face_goal(self, goal_):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position.x = goal_[1]
        goal.target_pose.pose.position.y = goal_[2]
        neki = quaternion_from_euler(0,0,self.angle)
        goal.target_pose.pose.orientation.w = neki[3]
        goal.target_pose.pose.orientation.z = neki[2]
        
        goal.target_pose.header.stamp = rospy.Time.now()
        self.simpleAction.send_goal_and_wait(goal, self.exeTimeout, self.preemptTimeout)
        print("Hello from ROS")
        time.sleep(1)
        return
                    
        # move_vec = np.array([p2.x - p1.x, p2.y-p1.y])
    def main_loop(self):
        self.simpleAction.wait_for_server()
        while len(self.goals) != 0:
            next_goal = self.goals.pop(0)
            print(next_goal)
            if next_goal[0] == "rotate_task":
                self.do_rotate_goal()
            elif next_goal[0] == "approach_face":
                self.do_approach_face_goal(next_goal)
            else:
                self.do_explore_goal(next_goal)
              
                
                
if __name__ == "__main__":
    hw = GoalSetter()
    hw.main_loop()