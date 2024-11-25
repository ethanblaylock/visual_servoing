# Visual Servoing

This package provides classes to perform both image based and pose based visual servoing on the Rethink Robotics Baxter platform. At present, the apriltag fiducial system is used as a marker for tracking image features and poses, although another detection system can be switched out easily in the ibvs_eih.py and pbvs_eih.py classes. 

To run this package, you must have the included apriltags_ros package (or a similar apriltags package for ROS), the [PyKDL package for Baxter](http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL) and a set of python functions to control the Baxter.

As of June 29, 2015, both eye in hand (eih) image based visual servoing (ibvs) and eih pose based visual servoing (pbvs) are supported, although there is a bug in the pbvs so that rotation of the arm does not converge properly, and so the arm is only able to match the desired pose, not rotation.

## Recent Changes
Several changes have been made to update this control package. Updating from python2 to python3. The apriltags_ros package has changed to no longer make corners a part of its message. Corners were computed using the pose of message.



## Running the package (CB 152)
### Configure the workspace
 ```
cd ~/baxter_ws           #moves you to the correct folder 
./baxter.sh                   #sets up environment variables correctly for baxter to work
./save_ears.sh            # sends a message through ROS to disable the sonar sensors
rosrun baxter_tools enable_robot.py -r       #this resets the robot state if you pushed the e-stop
rosrun baxter_tools enable_robot.py -e      #this re-enables the robot, and it may make a noise
rosrun baxter_tools tuck_arms.py -u           #this will untuck his arms, make sure there is space
source ~/Desktop/baxter_ws/devel/setup.bash #this helps
source ~/Desktop/robotics_ws/devel/setup.bash --extend  #this makes the code able to run

roslaunch visual_servoing ibvseih_left.launch # change to right to make right arm instead
```
### Launch Rviz in separate terminal
```
cd ~/baxter_ws
./baxter.sh
rosrun rviz rviz -d ~/Desktop/lab01.rviz   #this will launch the rviz configuration file
```
### Shutting it down
```
rosrun baxter_tools tuck_arms.py -t   # if you’ve opened a new terminal, you’ll need to do the
					# “cd ~/baxter_ws” and “./baxter.sh” commands again.
```

## Reference
For more information about the control theory used in this package, please see:

[Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches." Robotics & Automation Magazine, IEEE 13.4 (2006): 82-90.](https://www.researchgate.net/publication/3344795_Visual_servo_control_I_Basic_approaches)

Based on visual servoing library by Alex Zhu (alexzhu(at)seas.upenn.edu)

