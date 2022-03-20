#!/usr/bin/python3

from cmath import nan
import math


import rospy
import cv2
import numpy as np
import tf2_ros

from os.path import dirname, join

import tf2_geometry_msgs

#import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, Vector3, Pose, Polygon, Point32
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from task1.msg import PointArray


class face_localizer:
    def __init__(self):
        rospy.init_node('face_localizer', anonymous=True)

        # An object we use for converting images between ROS format and OpenCV format
        self.bridge = CvBridge()

        # The function for performin HOG face detection
        protoPath = join(dirname(__file__), "deploy.prototxt.txt")
        modelPath = join(dirname(__file__), "res10_300x300_ssd_iter_140000.caffemodel")
        self.face_net = cv2.dnn.readNetFromCaffe(protoPath, modelPath)

        # A help variable for holding the dimensions of the image
        self.dims = (0, 0, 0)

        # Marker array object used for showing markers in Rviz
        self.marker_array = MarkerArray()
        self.marker_num = 1

        # Subscribe to the image and/or depth topic, subcribing only to have higher refresh rate
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.dummy_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.dummy_callback)

        # Publiser for the visualization markers
        self.markers_pub = rospy.Publisher('face_markers', MarkerArray, queue_size=1000)

        # Object we use for transforming between coordinate frames
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)


        # distance in which faces are recognised as the same
        self.MAX_IMAGE_DISTANCE = 0.5
        self.face_point_array = PointArray()
        
        # marker publisher wihtout the extra marker visualisation stuff
        self.point_pub = rospy.Publisher('face_points', PointArray, queue_size=50)


    def get_pose(self,coords,dist,stamp):
        # Calculate the position of the detected face
        k_f = 554 # kinect focal length in pixels

        x1, x2, y1, y2 = coords

        face_x = self.dims[1] / 2 - (x1+x2)/2.
        face_y = self.dims[0] / 2 - (y1+y2)/2.

        angle_to_target = np.arctan2(face_x,k_f)

        # Get the angles in the base_link relative coordinate system
        x, y = dist*np.cos(angle_to_target), dist*np.sin(angle_to_target)

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
        # Get the next rgb and depth images that are posted from the camera
        try:
            rgb_image_message = rospy.wait_for_message("/camera/rgb/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        try:
            depth_image_message = rospy.wait_for_message("/camera/depth/image_raw", Image)
        except Exception as e:
            print(e)
            return 0

        # Convert the images into a OpenCV (numpy) format

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_image_message, "bgr8")
        except CvBridgeError as e:
            print(e)

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_image_message, "32FC1")
        except CvBridgeError as e:
            print(e)

        # Set the dimensions of the image
        self.dims = rgb_image.shape
        h = self.dims[0]
        w = self.dims[1]

        # Detect the faces in the image
        blob = cv2.dnn.blobFromImage(cv2.resize(rgb_image, (300, 300)), 1.0, (300, 300), (104.0, 177.0, 123.0))
        self.face_net.setInput(blob)
        face_detections = self.face_net.forward()

        for i in range(0, face_detections.shape[2]):
            confidence = face_detections[0, 0, i, 2]
            if confidence>0.5:
                box = face_detections[0,0,i,3:7] * np.array([w,h,w,h])
                box = box.astype('int')
                x1, y1, x2, y2 = box[0], box[1], box[2], box[3]

                # Extract region containing face
                face_region = rgb_image[y1:y2, x1:x2]

                # Find the distance to the detected face
                if y1 < y2 and x1 < x2 and y2 < 480 and x2 < 640:
                    face_distance = float(np.nanmean(depth_image[y1:y2,x1:x2]))
                else:
                    print("weird transformation")
                    break

                # print('Distance to face', face_distance)

                # Get the time that the depth image was recieved
                depth_time = depth_image_message.header.stamp

                # Find the location of the detected face
                pose = self.get_pose((x1,x2,y1,y2), face_distance, depth_time)

                if pose is not None:
                    
                    self.add_marker(pose)

                    self.markers_pub.publish(self.marker_array)
                    self.point_pub.publish(self.face_point_array)

    def dummy_callback(self,data):
        pass

    def add_marker(self, pose : Pose): # and add point
        # Checks if a marker already exists nearby and only add a new one if it doesnt
        # If it already exists average out past and current
        isClose = False
        
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
        
        # Create a point
        point = Point32()
        point.x = pose.position.x
        point.y = pose.position.y
        point.z = 0

        if math.isnan(point.x) or math.isnan(point.y):
            return
        else:

            print(point)
            
            
            for i in range(len(self.marker_array.markers)):
                m : Marker = self.marker_array.markers[i]
                p : Point32= self.face_point_array.points[i]
                if math.dist((p.x, p.y), (point.x, point.y)) < self.MAX_IMAGE_DISTANCE:
                    isClose = True
                    
                    m.pose.position.x = p.x * 0.2 + point.x * 0.8
                    m.pose.position.y = p.y * 0.2 + point.y * 0.8
                    
                    p.x = p.x * 0.2 + point.x * 0.8
                    p.y = p.y * 0.2 + point.y * 0.8
                    
                    break
            
            if not isClose:
                self.marker_array.markers.append(marker)
                self.face_point_array.points.append(point)

            




def main():

    face_finder = face_localizer()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        face_finder.find_faces()
        rate.sleep()



if __name__ == '__main__':
    main()