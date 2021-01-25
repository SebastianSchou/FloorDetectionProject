#!/bin/bash
exit_script() {
    trap - SIGINT SIGTERM # clear the trap
    QUIT=1
}

trap exit_script SIGINT SIGTERM

. /opt/ros/melodic/setup.bash
. devel/setup.bash

roslaunch traversable_area_detection take_picture.launch --screen