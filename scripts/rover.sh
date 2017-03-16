#!/bin/bash

gnome-terminal -e "roslaunch asl_turtlebot turtlebot_project_sim.launch gui:=false headlines:=true"

sleep 5 && gnome-terminal -e "rosrun turtlebot_mission_control mission_publisher.py"
gnome-terminal -e "rosrun turtlebot_mission_control explorer.py"
sleep 8 && gnome-terminal -e "rosrun turtlebot_mission_control supervisor.py"
sleep 1 && gnome-terminal -e "rosrun turtlebot_mission_control navigator.py"
sleep 1 && gnome-terminal -e "rosrun turtlebot_mission_control controller.py"

