#!/bin/bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/Group09
rosmake navibot
~/Group09/navibot/bin/navibot
