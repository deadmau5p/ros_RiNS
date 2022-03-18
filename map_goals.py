import time
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
import numpy as np
from geometry_msgs.msg import Twist
from task1.msg import ObjectDetection


class Map_Goals:
    
    def __init__(self):

        rospy.init_node("map_goals")
        self.ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.localization_publisher = rospy.Publisher('/navigation_velocity_smoother/raw_cmd_vel', Twist, queue_size = 1)
        rospy.Subscriber('/face_detection', ObjectDetection, self.face_detection)
        self.ms = rospy.Subscriber("/map", OccupancyGrid, self.callback)
        self.x_axis, self.y_axis, self.angle_r, self.angular_speed_r, self.start_angle = 0,0,0,0,0

        self.goals = [
            [0.12, 0.01],
            [-0.12, -0.81],
            [0,66, -0.83],
            [2.02, 0.94],
            [2.75, -0.32],
            [3.37, -0.29],
            [2.64, 0.43],
            [1.38, 0.90],
            [1.22, 1.39],
            [1.17, 2.16],
            [0.73, 2.74],
            [-0.74, 2.19],
            [-0.55, -1.56]
        ]

        self.map=None
        self.map_reso = None
        self.map_data = None
        time.sleep(1)
    
    def feedback_cb(self, feedback):
        pass


    def flip_y(self, data):
        ret = np.copy(data)
        for j in range(self.y_axis):
            ret[j] = data[self.x_axis - 1 -j]
        return ret

    def face_detection(self, data):
        rospy.loginfo(data.s)
        self.approach_face(data)


    def callback(self, map):
        self.map = map
        self.map_reso = map.info.resolution
        self.map_data = np.array(map.data).reshape(map.info.height, map.info.width)
        self.x_axis = map.info.width
        self.y_axis = map.info.height
        self.map_data = self.flip_y(self.map_data)
        self.status = 0


    def go_to_goals(self):
        for goal_ in self.goals:
            self.ac.wait_for_server()
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id="map"
            goal.target_pose.pose.orientation.w = 1
            goal.target_pose.pose.position.x = goal_[0]
            goal.target_pose.pose.position.y = goal_[1]
            goal.target_pose.header.stamp = rospy.Time.now()
            self.ac.send_goal(goal, feedback_cb=self.feedback_cb)

            or_x = self.map.info.origin.position.x
            or_y = self.map.info.origin.position.y
            height_map = self.map.info.height

            pixel_x = int((goal_[0] - or_x)/self.map_reso)
            pixel_y = int((height_map*self.map_reso + or_y + (-1)*goal_[1])/self.map_reso)

            if pixel_x > self.x_axis or pixel_y > self.y_axis or pixel_x < 0 or pixel_y < 0:
                rospy.logwarn(f'Unreachable, canceling goal because out of bounds {goal_[0]}{goal_[1]}!')
                self.ac.cancel_goal()
                continue

            if self.map_data[pixel_y, pixel_x] != 0:
                rospy.logwarn(f'Unreachable, canceling goal, because occupied!{goal_[0]}{goal_[1]}')
                self.ac.cancel_goal()
                continue   

            self.ac.wait_for_result()
            rospy.loginfo("goal is done rotate around")
            self.rotate()


    def approach_face(self, data):
        self.ac.cancel_all_goals()
        rospy.loginfo(f"I am approaching face at {data.goal}")
        print(data)
        




    def rotate(self):
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