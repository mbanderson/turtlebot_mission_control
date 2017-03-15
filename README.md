# turtlebot_mission_control

Authors: Connor Anderson, Michael Anderson, Kyle Brown, and Hala Al-Khalil

Run
```bash
roslaunch turtlebot_mission_control mission.launch
```
to run the `supervisor.py`, `navigator.py`, and `controller.py` scripts. To simulate receiving a mission specification, run
```bash
rosrun turtlebot_mission_control mission_publisher.py
```

To run Gazebo without the simulation window, run
```bash
roslaunch asl_turtlebot turtlebot_project_sim.launch gui:=false headless:=true
```
