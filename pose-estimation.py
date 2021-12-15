#!/usr/bin/env python2.7

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np
from sensor_msgs.msg import LaserScan # LaserScan type message is defined in sensor_msgs
from geometry_msgs.msg import Twist #
from math import atan2, hypot
import sys

class pose_estimation:

    def __init__(self):
        self.image_pub = rospy.Publisher("/robot_detection", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_estimation", Float64MultiArray, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.pub_cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.on_shutdown(self.fnShutDown)
	#sim
        #self.K = np.array([[646.78828, 0.0, 641.2034], [0.0, 646.68516, 515.10905], [0.0, 0.0, 1.0]])
        #self.D = np.array([-0.142019, 0.022613, -0.000643, -0.000067, 0.000000])
	
	#real
        self.K = np.array([[241.3146179724349, 0, 289.9129206889141], [0, 241.5427930294155, 212.3126306832639], [0, 0, 1]])
        self.D = np.array([0.006193541464912011, -0.003120565161628539, 0.0006768715025856813, 0.001060718960472666, 0])

	img = cv2.imread('/home/mscv_gr1/catkin_ws/src/ours/scripts/goal_pose.png')
        self.goal_pose = []
        #cv2.imshow("Display window", img)
	self.frame_goal_pose, self.goal_pose = self.compute_pose_estimation(img, self.K, self.D)
	

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8") # Covert into cv format
            robot_pose = Float64MultiArray()
            frame_pose, pose = self.compute_pose_estimation(cv_image, self.K, self.D) # call function
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame_pose, "bgr8")) # Publish
            robot_pose.data = pose.flatten()
            self.pose_pub.publish(robot_pose)
            error_hmat = error_robot_to_parking(pose, self.goal_pose)
            print('the camera to goal matrix \n {}'.format(self.goal_pose))
            print('the camera to robot matrix \n {}'.format(pose))
            print('the robot to goal matrix \n {}'.format(error_hmat))

            delta_x = error_hmat[0][3]
            delta_y = error_hmat[1][3]

            alpha = atan2(delta_y, delta_x)
            delta_d = hypot(delta_x, delta_y)
            forward_velocity, angular_velocity = controller(delta_d, alpha)
            self.move(forward_velocity, angular_velocity)

        except CvBridgeError as e:
            print(e)


    def compute_pose_estimation(self, frame, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)
        rmat = np.zeros((3,3), dtype=np.float64)
        tvec = np.array([0.0, 0.0, 0.0])
        rvec = np.array([0.0, 0.0, 0.0])
        hmat = np.zeros((4,4), dtype=np.float64)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                ret = aruco.estimatePoseSingleMarkers(corners[i], 0.14, matrix_coefficients, distortion_coefficients)
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                # Draw a square around the markers
                aruco.drawDetectedMarkers(frame, corners) 

                # Draw Axis
                frame = aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1) 
		
		rmat, _ =cv2.Rodrigues(rvec)

        hmat = np.column_stack((rmat, tvec))
        hmat = np.row_stack((hmat, [0, 0, 0, 1]))
        print('camera to robot matrix')
        print(hmat)

        return frame, hmat

    def move(self, forward_velocity, angular_velocity):
        robot_velocity = Twist()
        robot_velocity.linear.x = forward_velocity
        robot_velocity.linear.y = 0
        robot_velocity.linear.z = 0
        robot_velocity.angular.x = 0
        robot_velocity.angular.y = 0
        robot_velocity.angular.z = angular_velocity
        self.pub_cmd_vel.publish(robot_velocity)
    
    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

def controller(error_distance, error_angle):
    kp1 = 0.07
    kp2 = 0.4
    linear_velocity = kp1 * error_distance
    angular_velocity = kp2 * error_angle

    if error_angle > 0.2 or error_angle < -0.2:
        linear_velocity = 0.0
        
    return linear_velocity, angular_velocity

def error_robot_to_parking(robot_mat, parking_mat):
    try:
        return np.matmul(np.linalg.inv(robot_mat),parking_mat)
    except:
        return np.zeros((4,4),dtype=np.float64)

 
def main():
  print("Initializing pose estimation")
  rospy.init_node('pose_estimation', anonymous=True)
  print("Pose estimation is running")
  pose_estimation()
  rospy.spin()

if __name__ == '__main__':
    main()
