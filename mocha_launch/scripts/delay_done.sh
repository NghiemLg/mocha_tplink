#!/bin/bash
sleep 15
/opt/ros/noetic/lib/rostopic/rostopic pub -1 /delay/done std_msgs/Empty '{}'
