#!/usr/bin/python3

import rospy
import dlib
import cv2
import math
import numpy as np
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, Vector3, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task1.msg import MakeMarker



class marker_service:
    def __init__(self):
        self.current_pose = None
        rospy.init_node('marker_node', anonymous=True)

        self.marker_array = MarkerArray()
        self.marker_id = 0
        
        self.markers_pub = rospy.Publisher('/marker_service/output', MarkerArray, queue_size=1000)
        rospy.Subscriber('/marker_service/input', MakeMarker, self.on_marker_input)
        
        self.colors = {"red": ColorRGBA(1,0,0,1), "green": ColorRGBA(0,1,0,1), "blue": ColorRGBA(0,0,1,1)}

        
    def on_marker_input(self, make: MakeMarker):
        self.marker_id += 1
        pose = Pose()
        pose.position = make.point
        
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        #marker.lifetime = rospy.Duration.from_sec(10)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = self.colors[make.color]
        marker.id = self.marker_id
        self.marker_array.markers.append(marker)
        self.markers_pub.publish(self.marker_array)


    
    



def main():

        marker_serv = marker_service()

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
