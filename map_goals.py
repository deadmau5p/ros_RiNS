import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
import numpy as np
from geometry_msgs.msg import Twist
from task1.msg import ObjectDetection
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion
import math


class Map_Goals:
    
    def __init__(self):
        
        rospy.init_node("map_goals")
        #inicializiramo ActionClient
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        #subscribamo da dobivamo trenutni cilj
        rospy.Subscriber("move_base/goal", MoveBaseActionGoal, self.get_current_goal)
        #subscribamo da dobivamo trenutno poso robota
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.poseCallBack)
        #subscribamo na nas face detector
        rospy.Subscriber('/face_detection', ObjectDetection, self.face_detection)
        #subscribamo na sliko mape
        self.localization_publisher = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size = 1)
        self.angle_r, self.angular_speed_r, self.start_angle = 0,0,0
        self.current_goal = MoveBaseGoal()
        self.current_task = []

        #goali za Aljazevo mapo
        goals_map1 = [
            ["explore_goal", -0.14, 0.74],
            ["explore_goal",-0.12, -0.81],
            ["explore_goal",0.66, -0.83],
            ["explore_goal",2.02, 0.94],
            ["explore_goal",2.75, -0.32],
            ["explore_goal",3.37, -0.29],
            ["explore_goal",2.64, 0.43],
            ["explore_goal",1.38, 0.90],
            ["explore_goal",1.22, 1.39],
            ["explore_goal",1.17, 2.16],
            ["explore_goal",0.73, 2.74],
            ["explore_goal",-0.74, 2.19]
        ]

        #goali za drugo mapo
        goal_map2 = [
            ["explore_goal", -2.56, 1.00],
            ["explore_goal",-2.4, -0.17],
            ["explore_goal",-1.27, -0.08],
            ["explore_goal",0.4, 0.44],
            ["explore_goal",-0.46, 1.45],
            ["explore_goal",-1.44, 2.22],
            ["explore_goal",-2.25, 3.55],
        ]

        self.goals = goal_map2
        self.angle = 0
        time.sleep(1)
    
    #maybe useless
    def feedback_cb(self, feedback):
        pass

    #nastavimo trenutno orientacijo in kot v eulerju
    def poseCallBack(self, neki: PoseWithCovarianceStamped):
        orientation = neki.pose.pose.orientation
        self.angle = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))[2] + math.pi
    
    #nastavimo trenutni goal
    def get_current_goal(self, goal):
        self.current_goal = goal

    #funkcija kamor prleti nov face detection iz face detectorja
    def face_detection(self, data):
        #klicemo funkcijo ki nardi approach do obraza
        self.approach_face(data)


    #funkcija ki nas premika po mapi s pomočjo arraya trenutnih ciljev
    def go_to_goals(self):
        self.ac.wait_for_server()

        #dokler array ciljev ni prazen izvajamo zanko
        while len(self.goals) != 0:
            #vzamemo prvi cilj 
            goal_ = self.goals.pop(0)
            #nastavimo trenutni cilj
            self.current_task = goal_
            print(goal_)

            #trenutno imamo dve opciji ali rotairamo ali pa gremo na določen cilj(approach ali basic explore)
            if goal_[0] == "rotate_task":
                self.rotate1()
                #self.rotate(goal_[1], goal_[2], goal_[3])
            else:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id="map"
                goal.target_pose.pose.orientation.w = 1
                goal.target_pose.pose.position.x = goal_[1]
                goal.target_pose.pose.position.y = goal_[2]
                if goal_[0] == "approach_face":
                    goal.target_pose.pose.orientation.z = goal_[3]
                    goal.target_pose.pose.orientation.w = goal_[4]                
                goal.target_pose.header.stamp = rospy.Time.now()
                self.ac.send_goal_and_wait(goal)

            #izbrisemo trenutni cilj saj je že izveden
            self.current_task = []


            if  goal_[0] == "approach_face":
                time.sleep(2)
                rospy.loginfo("Hello from ROS")
            elif goal_[0] == "explore_goal":
                #če smo bili na exploru se potem nakoncu vsakega explora vrtimo za 360
                self.goals.insert(0,["rotate_task", self.angle, goal_[1],goal_[2]])

    #funkcija ki vrne index zadnjega approch goala v ciljih
    def get_index_of_last_approach_task(self):
        ret_val = -1
        for i in range(len(self.goals)):
            if self.goals[i][0] == "approach_face":
                ret_val = i
        return ret_val

    #funkcija za pristop do obraza
    def approach_face(self, data):

        last_app = self.get_index_of_last_approach_task()

        if self.current_task == []:
            self.goals.insert(0,["approach_face", data.goal.position.x, data.goal.position.y, data.goal.orientation.z, data.goal.orientation.w ] )
        elif self.current_task[0] == "explore_goal": #če smo sredi explora ga canclamo in dodamo approach goal
            #self.ac.cancel_goal()
            #ČE UPORABLJAMO FUNKCIJO ROTATE1 NE CANCLAMO GOALA
            self.goals.insert(0,["approach_face", data.goal.position.x, data.goal.position.y, data.goal.orientation.z, data.goal.orientation.w ] )
        elif self.current_task[0] == "approach_face":
            last_app = self.get_index_of_last_approach_task()
            self.goals.insert(last_app+1,["approach_face", data.goal.position.x, data.goal.position.y , data.goal.orientation.z, data.goal.orientation.w] ) #dodamo approach za zadnji approach goal
        elif self.current_task[0] == "rotate_task":
            self.goals.insert(last_app+1,["approach_face", data.goal.position.x, data.goal.position.y, data.goal.orientation.z, data.goal.orientation.w  ] ) #dodamo approach za zadnji approach goal
            #pocakamo da se odrotira
        
    
    def rotate(self, angle, pointx, pointy):
        angles = [0, 2*math.pi/3, 4*math.pi/3, 0]
        
        rat = rospy.Rate(2)
        for i in range(3):
            start_angle = angle
            goal = MoveBaseGoal()
            q = quaternion_from_euler(0, 0, (angles[i]+start_angle)%(2*math.pi))
                
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = pointx
            goal.target_pose.pose.position.y = pointy
            goal.target_pose.pose.orientation.z = q[2]
            goal.target_pose.pose.orientation.w = q[3]
            goal.target_pose.header.stamp = rospy.Time.now()
            self.ac.send_goal_and_wait(goal)
            rat.sleep()


    def rotate1(self):
        current_angle = 0
        t0 = rospy.Time.now().secs
        self.angular_speed_r = 30 * 3.14 / 180
        self.angle_r = 360 * 3.14 / 180
        rate = rospy.Rate(10)
        while (current_angle < self.angle_r):
            cmd=Twist()
            cmd.angular.z = self.angular_speed_r
            self.localization_publisher.publish(cmd)
            t1 = rospy.Time.now().secs
            current_angle = self.angular_speed_r * (t1 - t0)
            rate.sleep()
        


if __name__ == "__main__":
    hw = Map_Goals()
    hw.go_to_goals()