#!/usr/bin/python3
import sys
from time import time

from numpy.linalg import norm
import rospy
import dlib
import cv2
import math
import numpy as np
import tf2_geometry_msgs
import tf2_ros
#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, PoseWithCovarianceStamped
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task1.msg import ObjectDetection
import matplotlib.pyplot as plt
import  numpy as np
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class face_localizer:
    def __init__(self):
        self.current_pose = None
        rospy.init_node('face_localizer', anonymous=True)
        self.bridge = CvBridge()
        self.face_detector = dlib.get_frontal_face_detector()
        self.dims = (0, 0, 0)
        self.marker_array = MarkerArray()
        self.marker_num = 1
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)
        self.face_pub = rospy.Publisher('face_detection', ObjectDetection, queue_size=1000)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

    def pose_callback(self, pose):
        self.current_pose = pose


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

    def add_marker(self, pose):
        self.marker_num += 1
        marker = Marker()
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = 'map'
        marker.pose = pose
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime = rospy.Duration.from_sec(10)
        marker.scale = Vector3(0.1, 0.1, 0.1)
        marker.color = ColorRGBA(1, 1, 0, 1)
        marker.id = self.marker_num
        self.marker_array.markers.append(marker)
        self.markers_pub.publish(self.marker_array)

    #pofejkana funkcija get_pose da dobimo 4 točke na ravnini slike
    def get_4points_of_image_in_world(self,coords,depth_image,stamp):
        #TODO check error


        k_f = 554 # kinect focal length in pixels
        x1, x2, y1, y2 = coords
        if x1 < 0 or y1 < 0 or y2 > self.dims[0] or x2 > self.dims[1]:
            return None, None, None, None


        face_x1 = self.dims[1] / 2 - x1
        face_x2 = self.dims[1] / 2 - x2
        face_y1 = self.dims[0] / 2 - y1
        face_y2 = self.dims[0] / 2 - y2

        angle_to_target_x1 = np.arctan2(face_x1,k_f)
        angle_to_target_x2 = np.arctan2(face_x2,k_f)
        angle_to_target_y1 = np.arctan2(face_y1,k_f)
        angle_to_target_y2 = np.arctan2(face_y2,k_f)

        #print(y1, y1, y2, x2)
        #print(depth_image[y1,x1],depth_image[y1,x2],depth_image[y2,x1],depth_image[y2,x2])
        if math.isnan(depth_image[y1,x1]) or math.isnan(depth_image[y1,x2]) or math.isnan(depth_image[y2,x1]) or math.isnan(depth_image[y2,x2]):
            return None, None, None, None
        else:
            dist1= float(np.nanmean(depth_image[y1,x1]))
            dist2= float(np.nanmean(depth_image[y1,x2]))
            dist3= float(np.nanmean(depth_image[y2,x1]))
            dist4= float(np.nanmean(depth_image[y2,x2]))

        x11, y11 = dist1*np.cos(angle_to_target_x1), dist1*np.sin(angle_to_target_x1)
        x22, y22 = dist2*np.cos(angle_to_target_x2), dist2*np.sin(angle_to_target_x2)
        x33, y33 = dist3*np.cos(angle_to_target_y1), dist3*np.sin(angle_to_target_y1)
        x44, y44 = dist4*np.cos(angle_to_target_y2), dist4*np.sin(angle_to_target_y2)
        # Define a stamped message for transformation - in the "camera rgb frame"
        
        #vzamemo vse kote slike 
        point_s1 = PointStamped()
        point_s1.point.x = x11
        point_s1.point.y = y11
        point_s1.point.z = 0.4
        point_s1.header.frame_id = "base_link"
        point_s1.header.stamp = stamp

        point_s2 = PointStamped()
        point_s2.point.x = x22
        point_s2.point.y = y22
        point_s2.point.z = 0.4
        point_s2.header.frame_id = "base_link"
        point_s2.header.stamp = stamp

        point_s3 = PointStamped()
        point_s3.point.x = x33
        point_s3.point.y = y33
        point_s3.point.z = 0.3
        point_s3.header.frame_id = "base_link"
        point_s3.header.stamp = stamp

        point_s4 = PointStamped()
        point_s4.point.x = x44
        point_s4.point.y = y44
        point_s4.point.z = 0.3
        point_s4.header.frame_id = "base_link"
        point_s4.header.stamp = stamp

        # Get the point in the "map" coordinate system
        try:
            point_world1 = self.tf_buf.transform(point_s1, "map")
            point_world2 = self.tf_buf.transform(point_s2, "map")
            point_world3 = self.tf_buf.transform(point_s3, "map")
            point_world4 = self.tf_buf.transform(point_s4, "map")

            # Create a Pose object with the same position
            pose1 = Pose()
            pose1.position.x = point_world1.point.x
            pose1.position.y = point_world1.point.y
            pose1.position.z = point_world1.point.z

            pose2 = Pose()
            pose2.position.x = point_world2.point.x
            pose2.position.y = point_world2.point.y
            pose2.position.z = point_world2.point.z

            pose3 = Pose()
            pose3.position.x = point_world3.point.x
            pose3.position.y = point_world3.point.y
            pose3.position.z = point_world3.point.z

            pose4 = Pose()
            pose4.position.x = point_world4.point.x
            pose4.position.y = point_world4.point.y
            pose4.position.z = point_world4.point.z

            #print("1", pose1.position,"2",  pose2.position,"3", pose3.position,"4",  pose4.position)
        except Exception as e:
            print(e)
            pose1 = None
            pose2 = None
            pose3 = None
            pose4 = None

        return pose1, pose2, pose3, pose4    

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
            """plt.imshow(face_rectangle[0,0])
            plt.show()"""
            
            if y1 < 0 or y2 < 0 or x1 < 0 or x2 < 0:
                return 0
            face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))

            depth_time = depth_image_message.header.stamp
            pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

            if pose is not None:
                
                pose1, pose2, pose3, pose4 = self.get_4points_of_image_in_world((x1,x2,y1,y2), depth_image, depth_time)
                if pose1 is None:
                    continue

                
                
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
                    pose11 = marker_i.pose.position
                    pose22 = marker.pose.position
                    pose1_np = np.array([pose11.x, pose11.y ,pose11.z])
                    pose2_np = np.array([pose22.x, pose22.y ,pose22.z])


                    squared_dist = np.sum((pose1_np-pose2_np)**2, axis=0)
                    dist = np.sqrt(squared_dist)
                    if dist < smallest_dist:
                        smallest_dist = dist
                if len(self.marker_array.markers) == 0 or smallest_dist >= 0.5:
                        for p_ in [pose1, pose2, pose3, pose4]:
                            self.add_marker(p_)

                        self.marker_array.markers.append(marker)
                        self.markers_pub.publish(self.marker_array)
                        msg = ObjectDetection()
                        print("found face")
                        msg.goal = self.calculate_approaching_point(pose, face_distance, pose1, pose2, pose3, pose4)
                        msg.s = "face detected :)"
                        self.face_pub.publish(msg)



    def calculate_approaching_point(self, face_center_pose, dist, pose1, pose2, pose3, pose4):
        #trenutna poza robota
        p1 = self.current_pose.pose.pose.position
        #poza obraza
        p2 = face_center_pose.position

        #slika je ravnina zračunamo normalo
        normal_vector = np.array(self.get_normal_on_face(pose1, pose2, pose3, pose4))

        print(normal_vector)

        #vektor ki oribližno pove kam gledamo
        move_vec = np.array([p2.x - p1.x, p2.y-p1.y, 0])

        #kot med tem kam mi gledamo in kam gleda normala 
        cosine_between = np.dot(normal_vector,move_vec)/norm(normal_vector)/norm(move_vec) 

        #če je kot večji od -+ 90 (0 do -1) pomeni da normala gleda proti nam, če ne jo obrnemo proti nam 
        if cosine_between >= 0:
            normal_vector = normal_vector * (-1)
        
        #izračunamo ratio da bo nas vektor od slike dolg 0.5 metra (oddaljenost approaching pointa do slike)
        ratio = 0.5 / np.sum( np.sqrt(np.dot(normal_vector, normal_vector)))
        goal_coor = [p2.x, p2.y, 0]+ normal_vector * ratio

        #nastavimo approach point
        approach_point = Pose()
        approach_point.position.x = goal_coor[0]
        approach_point.position.y = goal_coor[1]
        approach_point.position.z = 0
        normal_vector = normal_vector * (-1)
        orient = quaternion_from_euler(normal_vector[0], normal_vector[1], normal_vector[2])
        print(orient)
        approach_point.orientation.x = orient[0]
        approach_point.orientation.y = orient[1]
        approach_point.orientation.z = orient[2]
        approach_point.orientation.w = orient[3]

        return approach_point


    def get_normal_on_face(self, pose1, pose2, pose3, pose4):
        #vzamemo dva vektorja med 4 točkami na ravnini
        r_b = [pose2.position.x - pose1.position.x, pose2.position.y - pose1.position.y, pose2.position.z - pose1.position.z]
        s_b = [pose3.position.x - pose1.position.x, pose3.position.y - pose1.position.y, pose3.position.z - pose1.position.z]
        #naredimo vektorski produkt
        cross_normal = [(r_b[1]*s_b[2]- r_b[2]*s_b[1]), (r_b[2]*s_b[0] - r_b[0]*s_b[2]), 0]
        return cross_normal


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
