# ROS - Visual Servoing

![UB](https://user-images.githubusercontent.com/62597513/145659645-9ab35c4d-694e-499d-8fad-6bf1091d32ec.jpeg)

## Prepared by:
### Lateef Olalekan Aderinoye and Olarinde Fatai Jimoh

## Supervised by: 
   ###            Omar Tahri (Professor)
   ###            Ahmed Wael Ahmed Hossameldin

   

# Project goal
The aim is to move a mobile robot "turtlebot3" from a current pose to a target pose using Aruco markers for pose detection with minimum distance using visual servoing techniques by calculating the pose of the camera and the error.
 
 # Requirement 
- **Turtlebot3**: It is the newest generational mobile robot. It is a basic model to use AutoRace packages for autonomous driving on ROS. For this project, the TurtleBot3 Burger (with packages installed on it) was  provided by our university https://condorcet.u-bourgogne.fr/en.  The main components are shown below: 

![3](https://user-images.githubusercontent.com/62597513/145630186-4da6bcb0-b4aa-4c0d-b006-39453fabb56b.png)

source : https://www.robotis.us/turtlebot-3-burger-us/


- Camera - **Raspberry Pi ‘fish-eye’** camera
- The system was run through a **stationary PC**, connected to the TurtleBot3 running **Ubuntu 18.04**
- Robotic Operating System **(ROS)** software


# Project Architecture 
- Camera Calibration
- Image Detection
- Pose Estimation
- Robot Navigation


# 1. Camera Calibration
The camera  calibration is the first phase of the project. To achieve the calibration, We used the ROS_camera_calibration_package for calibration of our monocular camera using a checkerboard calibration target. Therefore, camera calibration is an integral part of this project because it was understood that our camera has distortion which affects the functionality and sensining of the image view. For this reason, we calibrated the camera using a checkerboard to obtain the camera intrinsic and extrinsic parameters with distortion coefficients.

### The ROS package was used to achieve this step as stated below:
The command as such, **rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.08 image:=/camera/image_raw camera:=/camera** was retrieved from this link [Monocular calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) to launch our graphical user interface used to calibrated our **8x6 checkerboard** to obtain our intrinsic parameters used for our project. The intrinsic parameters therefore, allows a mapping between camera coordinates and pixel coordinates in the image frame.

The intrinsic is shown below:

![calibration parameters](https://user-images.githubusercontent.com/62597513/146257300-e954d881-9e4a-4ef2-b118-5e6a87731aba.png)


# 2. Image Detection 

This is the second phase of the project. Here, we used the fisheye camera to to acquire real time image by subscribing to the ROS topic **"/camera/image_raw"**. The image was saved as **goal_pose.png**. We then used the distortion coefficients acquired from the camera calibration stage to undistort the image. The image is shown below: 

![goal_pose](https://user-images.githubusercontent.com/62597513/146259814-f4354f2c-3400-4e60-850f-02a537bbb40a.png)


### 2.1 Aruco Marker Detection
We used the OpenCV library to detect the Aruco marker of size **7x7** placed at the top of our turtlebot3 to get the target pose by calling its function **cv2.detectMarkers** from which we receive the corners of our marker that was used in the next stage to navigate the robot. 

source file: [detect.py](https://github.com/jimohola/Visual_Servoing_project/blob/main/launch%20files/detect.py)

The detected image is shown below:

![detect_img](https://user-images.githubusercontent.com/62597513/146269872-b68618c9-fd22-40d2-a484-7b1abe7f6337.png)



# 3. Pose Estimation

To estimate the current and target pose, we call the function **aruco.estimatePoseSingleMarkers** in order to get the  **translational vector** [x, y, z] and **rotational vector** [x, y, z] in order to calculate the linear x and angular z speed of the robot in order to move through the target direction. As such, the controller will subscribe to get the  message from our source file [pose-estimation.py](https://github.com/jimohola/Visual_Servoing_project/blob/main/launch%20files/pose-estimation.py) in order to send those vectors.

Here is the image of the target pose:

![detect target pose](https://user-images.githubusercontent.com/62597513/146269854-677edffa-9cd0-4e97-bcb3-3bcdd4350181.png)



# 4. Robot Navigation 


# Demo

https://user-images.githubusercontent.com/62597513/146273122-4440165a-5ad5-448a-b301-9e258d0fdce2.mp4


https://user-images.githubusercontent.com/62597513/146277477-9d324bbb-15b8-4d70-b54d-1ae049d71fa9.mp4





