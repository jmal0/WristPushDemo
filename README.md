WristPushDemo
=============

This is a ros node that uses the force torque readings for the wrist to control movement of the 
shoulder and elbow motors. 

The node will initially extend the arm, and either halt movement or retract the arm depending 
on the force applied.

Dependencies:
This demo depends on the ROS package MAESTRO for robot control.


REQUIREMENTS
=============

- Ubuntu 12.04 LTS
- ROS Fuerte
- MAESTRO (ROS stack)

To Run
=============

Preconditions:
- Maestro must already be started.
- Motors must already be homed and enabled in Maestro
- Motors RSP and REP must be enabled
- Force Torque Sensor RWT must be available

Run:
- run 'roslaunch wrist_demo demo.launch' 
- Wait for the robot arm to extend fully
- Press <Enter> to calibrate low
- While pushing the wrist in the direction of the arm, press <Enter> to calibrate high
- Push the wrist in to move the arm inwards, and release to make the arm extend.
- Exit the demo by pressing <Enter> a third time. The arm will return to ground state.
- If the program is interrupted (via a break, for example) the arm will not return to ground state.

