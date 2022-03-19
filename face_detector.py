#!/usr/bin/python3

from cmath import nan
import math
from shutil import move
import sys
from time import time

from torch import mkldnn_convolution_backward_weights
import rospy
import dlib
import cv2
import numpy as np
import tf2_geometry_msgs
import tf2_ros
#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry
from task1.msg import ObjectDetection
import matplotlib.pyplot as plt


class face_localizer:
    def __init__(self):
        self.current_pose = None
        rospy.init_node('face_localizer', anonymous=True)
        self.bridge = CvBridge()
        self.face_detector = dlib.get_frontal_face_detector()
        self.dims = (0, 0, 0)
        self.marker_array = MarkerArray()
        self.marker_num = 1
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.face_pub = rospy.Publisher('face_detection', ObjectDetection, queue_size=1000)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def odom_callback(self, odom):
        self.current_pose = odom

    def get_pose(self,coords,dist,stamp):
        k_f = 554 # kinect focal length in pixels
        x1, x2, y1, y2 = coords
        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

        ### Define a stamped message for transformation - directly in "base_link"
        point_s = PointStamped()
        point_s.point.x = x
        point_s.point.y = y
        point_s.point.z = 0.3
        point_s.header.frame_id = "base_link"
        point_s.header.stamp = rospy.Time(0)

        # Define a stamped message for transformation - in the "camera rgb frame"
        point_s = PointStamped()
        point_s.point.x = -y
        point_s.point.y = 0
        point_s.point.z = x
        point_s.header.frame_id = "camera_rgb_optical_frame"
        point_s.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world = self.tf_buf.transform(point_s, "map")

            # Create a Pose object with the same position
            pose = Pose()
            pose.position.x = point_world.point.x
            pose.position.y = point_world.point.y
            pose.position.z = point_world.point.z
        except Exception as e:
            print(e)
            pose = None

        return pose
    

    def find_faces(self):
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            #print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            #print(e)
            return 0

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print("fuck rgb image")

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print("fuck rgb_image")

        self.dims = rgb_image.shape

        face_rectangles = self.face_detector(rgb_image, 0)

        for face_rectangle in face_rectangles:
            x1 = face_rectangle.left()
            x2 = face_rectangle.right()
            y1 = face_rectangle.top()
            y2 = face_rectangle.bottom()
            face_region = rgb_image[y1:y2,x1:x2]

            # Visualize the extracted face
            #plt.imshow(face_region)
            #plt.show())
            
            if y1 < 0 or y2 < 0 or x1 < 0 or x2 < 0:
                return 0
            face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))
            
                


            depth_time = depth_image_message.header.stamp
            pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

            if pose is not None:
                
                # Create a marker used for visualization
                self.marker_num += 1
                marker = Marker()
                marker.header.stamp = rospy.Time(0)
                marker.header.frame_id = 'map'
                marker.pose = pose
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.frame_locked = False
                marker.lifetime = rospy.Duration.from_sec(10)
                marker.id = self.marker_num
                marker.scale = Vector3(0.1, 0.1, 0.1)
                marker.color = ColorRGBA(0, 1, 0, 1)
                

                smallest_dist = 100

                for i in range(len(self.marker_array.markers)):
                    marker_i = self.marker_array.markers[i]
                    pose1 = marker_i.pose.position
                    pose2 = marker.pose.position
                    pose1_np = np.array([pose1.x, pose1.y ,pose1.z])
                    pose2_np = np.array([pose2.x, pose2.y ,pose2.z])


                    squared_dist = np.sum((pose1_np-pose2_np)**2, axis=0)
                    dist = np.sqrt(squared_dist)
                    if dist < smallest_dist:
                        smallest_dist = dist
                if len(self.marker_array.markers) == 0 or smallest_dist >= 0.5:
                        self.marker_array.markers.append(marker)
                        self.markers_pub.publish(self.marker_array)
                        msg = ObjectDetection()
                        print("found face")
                        msg.goal = self.calculate_approaching_point(pose, face_distance)
                        msg.s = "face detected :)"
                        self.face_pub.publish(msg)

    def calculate_approaching_point(self, pose, dist):
        p1 = self.current_pose.pose.pose.position
        p2 = pose.position
        move_vec = np.array([p2.x - p1.x, p2.y-p1.y])
        ratio = (dist - 0.5) / dist
        goal_coor = move_vec * ratio
        approach_point = Pose()
        approach_point.position.x = p1.x + goal_coor[0]
        approach_point.position.y = p1.y + goal_coor[1]
        approach_point.position.z = 0
        return approach_point


    def points_in_circle_np(self, radius, x0=0, y0=0, ):
        x_ = np.arange(x0 - radius - 1, x0 + radius + 1, dtype=int)
        y_ = np.arange(y0 - radius - 1, y0 + radius + 1, dtype=int)
        x, y = np.where((x_[:,np.newaxis] - x0)**2 + (y_ - y0)**2 <= radius**2)
        for x, y in zip(x_[x], y_[y]):
            yield x, y

def main():

        face_finder = face_localizer()

        rate = rospy.Rate(1)
        rospy.Subscriber("/camera/rgb/image_raw", Image)
        rospy.Subscriber("/camera/depth/image_raw", Image)
        while not rospy.is_shutdown():
            face_finder.find_faces()
            rate.sleep()

        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
