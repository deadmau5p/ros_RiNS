#!/usr/bin/python3

import time
from turtle import goto

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import Twist, Pose, Point
from task1.msg import ObjectDetection, PointArray, MakeMarker
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math


class GoalSetter:
    
    def __init__(self) -> None:
        rospy.init_node("goal_setter")
        self.simpleAction = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/face_detection", ObjectDetection, self.face_points_callback)
        self.localization_publisher = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size = 1)
        
        self.x_axis, self.y_axis, self.angle_r, self.angular_speed_r, self.angle = 0,0,0,0,0
        self.location = Point()
        
        self.marker_pub = rospy.Publisher('/marker_service/input', MakeMarker, queue_size=1000)
        
        #Aljaz goali, nevem zakaj mormo met drgacne ?
        """
        self.goals =[
            ["explore_goal", -3.8, 3.2],
            ["explore_goal", -3.34, 2.80],
            ["explore_goal", -2.8, 1.61],
            ["explore_goal", -3.61, 1.30],
            ["explore_goal", -2.66, 0.45],
            ["explore_goal", -2.23, -0.09],
            ["explore_goal", -1.19, -0.28],
            ["explore_goal", -0.06, 0.15],
            ["explore_goal", 0.6, 0.57],
            ["explore_goal", -0.37, 1.72],
            ["explore_goal", -1.12, 1.48],
            ["explore_goal", -1.45, 2.6],
            ["explore_goal",-0.69, 3.45],
            ["explore_goal",-2.26, 3.56],
        ]
        """
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
        
    def face_points_callback(self, data):
        self.goals.insert(0,["approach_face", data.goal.position.x, data.goal.position.y, data.goal.orientation.z, data.goal.orientation.w ] )
    
    
    def do_rotate_goal(self):
        print("rotate_goal")
        current_angle = 0
        t0 = rospy.Time.now().secs
        self.angular_speed_r = 60 * 3.14 / 180
        self.angle_r = 360 * 3.14 / 180
        rate = rospy.Rate(10)
        while (current_angle < self.angle_r):
            cmd=Twist()
            cmd.angular.z = self.angular_speed_r
            self.localization_publisher.publish(cmd)
            t1 = rospy.Time.now().secs
            current_angle = self.angular_speed_r * (t1 - t0)
            rate.sleep()
    
    
    def do_explore_goal(self, goal_):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position.x = goal_[1]
        goal.target_pose.pose.position.y = goal_[2]
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.header.stamp = rospy.Time.now()
        self.simpleAction.send_goal_and_wait(goal)
        self.do_rotate_goal()
        return
        
        
    def do_approach_face_goal(self, goal_):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id="map"
        goal.target_pose.pose.position.x = goal_[1]
        goal.target_pose.pose.position.y = goal_[2]
        goal.target_pose.pose.orientation.w = goal_[3]
        goal.target_pose.pose.orientation.z = goal_[4]
        goal.target_pose.header.stamp = rospy.Time.now()
        self.simpleAction.send_goal_and_wait(goal)
        print("Hello from ROS")
        time.sleep(3)
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