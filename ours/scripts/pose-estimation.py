#!/usr/bin/env python2.7

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge, CvBridgeError
import cv2.aruco as aruco
import numpy as np
import sys

class pose_estimation:

    def __init__(self):
        self.image_pub = rospy.Publisher("/robot_detection", Image, queue_size=1)
        self.pose_pub = rospy.Publisher("/pose_estimation", Float64MultiArray, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        
	#sim
        #self.K = np.array([[646.78828, 0.0, 641.2034], [0.0, 646.68516, 515.10905], [0.0, 0.0, 1.0]])
        #self.D = np.array([-0.142019, 0.022613, -0.000643, -0.000067, 0.000000])
	
	#real
        self.K = np.array([[241.3146179724349, 0, 289.9129206889141], [0, 241.5427930294155, 212.3126306832639], [0, 0, 1]])
        self.D = np.array([0.006193541464912011, -0.003120565161628539, 0.0006768715025856813, 0.001060718960472666, 0])

	img = cv2.imread('/home/mscv_gr1/catkin_ws/src/ours/scripts/goal_pose.png')

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
	    print('the camera to goal matrix \n {}'.format(self.goal_pose))
        except CvBridgeError as e:
            print(e)


    def compute_pose_estimation(self, frame, matrix_coefficients, distortion_coefficients):

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)
        parameters = cv2.aruco.DetectorParameters_create()

        corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters)

            # If markers are detected
        if len(corners) > 0:
            for i in range(0, len(ids)):
                # Estimate pose of each marker and return the values rvec and tvec---(different from those of camera coefficients)
                ret = aruco.estimatePoseSingleMarkers(corners[i], 0.14, matrix_coefficients, distortion_coefficients)
                self.rvec, self.tvec = ret[0][0,0,:], ret[1][0,0,:]

                # Draw a square around the markers
                aruco.drawDetectedMarkers(frame, corners) 

                # Draw Axis
                frame = aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, self.rvec, self.tvec, 0.1) 
		
		self.rmat, _ =cv2.Rodrigues(self.rvec)

        self.hmat = np.column_stack((self.rmat, self.tvec))
        self.hmat = np.row_stack((self.hmat, [0, 0, 0, 1]))
        print('camera to robot matrix')
        print(self.hmat)
 

        return frame, self.hmat

   


def main():
  print("Initializing pose estimation")
  rospy.init_node('pose_estimation', anonymous=True)
  print("Pose estimation is running")
  pose_estimation()
  rospy.spin()

if __name__ == '__main__':
    main()
