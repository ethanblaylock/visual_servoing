# Visual Servoing
Written by Alex Zhu (alexzhu(at)seas.upenn.edu)

This package provides classes to perform both image based and pose based visual servoing on the Rethink Robotics Baxter platform. At present, the apriltag fiducial system is used as a marker for tracking image features and poses, although another detection system can be switched out easily in the ibvs_eih.py and pbvs_eih.py classes. 

To run this package, you must have the included apriltags_ros package (or a similar apriltags package for ROS), the [PyKDL package for Baxter](http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL) and a set of python functions to control the Baxter. The current package that is used is developed at the GRASP Lab at the University of Pennsylvania, and is not included in this package. If you do not have access to this package, you simply need to replicate two functions:

As of June 29, 2015, both eye in hand (eih) image based visual servoing (ibvs) and eih pose based visual servoing (pbvs) are supported, although there is a bug in the pbvs so that rotation of the arm does not converge properly, and so the arm is only able to match the desired pose, not rotation.

## Recent Changes
Several changes have been made to update this control package. Updating from python2 to python3. The apriltags_ros package has changed to no longer make corners a part of its message. Corners were computed using the pose of message.

For more information about the control theory used in this package, please see:

[Chaumette, François, and Seth Hutchinson. "Visual servo control. I. Basic approaches." Robotics & Automation Magazine, IEEE 13.4 (2006): 82-90.](https://www.researchgate.net/publication/3344795_Visual_servo_control_I_Basic_approaches)
