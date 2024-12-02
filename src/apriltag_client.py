#!/usr/bin/env python
"""
Wrapper client class that communicates with the apriltags_ros package to 
store tag information (images, pose, corners).
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)
"""

import roslib
import numpy as np
import numpy.matlib
import cv2
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import *
from apriltags_ros.msg import AprilTagDetectionArray
from utility import *

from std_msgs.msg import (
    Header,
    UInt16,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Point32,
    Quaternion,
)

import struct

class AprilTagClient(object):
    """
    Wrapper client class that communicates with the apriltags_ros package to 
    store tag information (images, pose, corners).
    """
    def __init__(self, target_marker):
        self._last_marker=-1

        self.marker_t=None
        self.marker_R=None
        self.corners = None
        self.depth = None
        self.image = None
        self.pose = None
        self._bridge = CvBridge()
        self.K = np.array([[405.78, 0, 639.16],
                          [0, 405.78, 425.09],
                          [0, 0, 1]])
        # ROS Pubs and Subs
        rospy.on_shutdown(self.__shutdown_hook)
        rospy.Subscriber("/tag_detections",AprilTagDetectionArray,self.__process_detection)
        rospy.Subscriber("/tag_detections_image",Image,self.__get_image)

    def __shutdown_hook(self):
        pass
    
    def __get_marker(self,detections):
        """
        Parses an apriltag detection message and gets the index for the last used marker, or
        simply the first one found if the last one was not seen. Returns -1 if nothing found.
        """
        if (len(detections)==0):
            return None,None
        marker=self.__find_marker(detections,self._last_marker)
        if (marker!=-1):
            return self._last_marker,marker
        else:
            self._last_marker=detections[0].id
            return detections[0].id,0

    def __find_marker(self,detections,marker_num):
        """
        Finds index of marker #marker_num in the apriltag detection message detections. If not found,
        returns -1.
        """
        for i in range(0,len(detections)):
            if detections[i].id==marker_num:
                return i
        return -1
    
    def __get_image(self,image):
        """
        # Dummy function to save incoming images
        """
        try:
            cv_image=self._bridge.imgmsg_to_cv2(image,"bgr8")
            self.image=cv_image
        except CvBridgeError as e:
            print(e)

    def __process_detection(self,apriltag_detections):
        """
        Process income apriltag detections message, store pose and corners if detections were found.
        """        
        if rospy.is_shutdown():
            return
        # Get best marker for gripper pose
        (my_marker,marker_pos)=self.__get_marker(apriltag_detections.detections)
        print("hi")
        if my_marker is None:
            return

        size = apriltag_detections.detections[marker_pos].size
        (self.marker_t,self.marker_R)=get_t_R(apriltag_detections.detections[marker_pos].pose.pose.position, apriltag_detections.detections[marker_pos].pose.pose.orientation)
        corners, depths = self.compute_tag_corners(apriltag_detections.detections[marker_pos].pose.pose, size, self.K)
        self.corners = corners
        self.depth = depths
        print(depths)
        
    def project_to_image_plane(self, corners_3d, camera_matrix):
        """
        Project the 3D corners onto the 2D image plane using the camera intrinsic matrix.

        Args:
        - corners_3d: List of 3D corner coordinates in the camera frame.
        - camera_matrix: The intrinsic camera matrix.

        Returns:
        - List of 2D image coordinates of the corners.
        """
        corners_2d = np.zeros(8)
        counter = 0
        for corner in corners_3d:
            # Project each corner using the intrinsic matrix
            homogeneous_corner = np.array([corner[0], corner[1], corner[2], 1.0])  # Convert to homogeneous coordinates
            image_point = np.dot(camera_matrix, homogeneous_corner[:3])  # Apply the intrinsic matrix
            u = image_point[0] / image_point[2]
            v = image_point[1] / image_point[2]
            #corners_2d.append([u, v])
            corners_2d[counter] = u
            corners_2d[counter+1] = v
            counter += 2
        
        return corners_2d

    def compute_tag_corners(self, tag_pose, tag_size, camera_matrix):
        """
        Compute the corners of the AprilTag in the camera frame and project them onto the image plane.

        Args:
        - tag_pose: Pose of the tag in the camera frame (geometry_msgs/Pose).
        - tag_size: Size of the tag (length of one side).
        - camera_matrix: The intrinsic camera matrix.

        Returns:
        - A list of projected 2D corners of the tag in the image frame.
        """
        # Tag corners in the tag's local frame (3D coordinates)
        half_size = tag_size / 2.0
        tag_corners_local = np.array([
            [-half_size, -half_size, 0],  # corner 0 (top-left)
            [half_size, -half_size, 0],   # corner 1 (top-right)
            [half_size, half_size, 0],    # corner 2 (bottom-right)
            [-half_size, half_size, 0],   # corner 3 (bottom-left)
        ])
        #print(tag_corners_local)
        # Convert the tag's rotation quaternion into a rotation matrix
        q = [tag_pose.orientation.x, tag_pose.orientation.y, tag_pose.orientation.z, tag_pose.orientation.w]
        rotation_matrix = quaternion_matrix(q)[:3, :3]  # 3x3 rotation matrix
        
        # Tag's translation vector (position in camera frame)
        translation = np.array([tag_pose.position.x, tag_pose.position.y, tag_pose.position.z])
        
        # Compute the corners in the camera frame
        tag_corners_camera = []
        corners_2d = np.zeros(8)
        depths = np.zeros(4)
        counter = 0
        for corner in tag_corners_local:
            # Apply the rotation and translation to the local corners
            corner_camera = np.dot(rotation_matrix, corner) + translation
            tag_corners_camera.append(corner_camera)
            
            
            corners_2d[counter] = corner_camera[0]/corner_camera[2]
            corners_2d[counter+1] = corner_camera[1]/corner_camera[2]
            depths[counter/2] = corner_camera[2]
            counter += 2
        
        # Project the corners onto the image plane
        #corners_2d = self.project_to_image_plane(tag_corners_camera, camera_matrix)
        
        return corners_2d, depths
