# CAATO Arm

Steps: 

1. Git Clone this package
2. Git Clone rosserial package
3. Both of these package has to be in your catkin source folder
4. Upload the arm_arduino_srv.ino code to Arduino
2. roslaunch arm_srv arm.launch
3. rostopic list (to check if all the topics are there)
4. rosrun arm_srv arm_client [followed by a number] (1 moves the actuator down, 2 moves the actuator up, 0 breaks the actuator)

Job of each file:

arm_server.cpp: Server, posting info about where the linear actuar is moving# arm
# arm
