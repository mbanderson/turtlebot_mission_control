# TurtleBot Mission Control
TurtleBot Mission Control was developed by Michael Anderson, Connor Anderson,
Kyle Brown, and Hala Al-Khalil as part of Stanford University's Robotic
Autonomy course. This ROS-based system was created in 3 days and deployed on 
a TurtleBot robot to semi-autonomously navigate an unknown maze. The mission was 
a success! Additional information (and video results) available at
[TurtleBot Mission Control](https://mbanderson.github.io/turtlebot_mission_control/).

# Mission Overview
At a high level, this project is modeled after the challenges of navigating
a rover on Mars. The communicatation delay between the Earth and Mars
necessitates some level of robotic autonomy. While a human operator may specify
scientific targets of interest or general mission objectives, the robot must
be able to safely navigate hazardous terrain on its own.

# Mission Phases
In this project, mission control (the Robotic Autonomy course staff) specify
a list of scientific targets that must be visited by the robot. This target list
is not known to the robot (or developer team!) beforehand, and targets are
marked by AprilTag fiducials.

The mission consists of two stages: semi-autonomous mapping, and autonomous
navigation. In the first stage, a human driver specifies objective waypoints
the TurtleBot should investigate. The robot uses an A* path planner to navigate
to the waypoints while also estimating the location of visible scientific
targets. Once each target has been located, the TurtleBot switches to
autonomous mode and must return to each scientific target in a specified order.

# Dependencies
We use the SLAM package [gmapping](http://wiki.ros.org/gmapping) to estimate
environment obstacles to feed into the A* path planner. Mission objectives
are marked with fiducials and identified by the [AprilTags](http://wiki.ros.org/apriltags_ros)
package.

# Running a Mission
The onboard ROS nodes must first be initialized by running:
```bash
roslaunch turtlebot_mission_control mission.launch
```
This launches the `supervisor.py`, `navigator.py`, and `controller.py` nodes. 

The mission objectives are unknown to the TurtleBot during initialization. To
simulate receiving a mission specification, run:
```bash
rosrun turtlebot_mission_control mission_publisher.py
```

By default, the mission simulation will be displayed in Gazebo. To perform the
simulation headless, run:
```bash
roslaunch asl_turtlebot turtlebot_project_sim.launch gui:=false headless:=true
```