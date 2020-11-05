#!/bin/bash
exit_script() {
    trap - SIGINT SIGTERM # clear the trap
    QUIT=1
}

trap exit_script SIGINT SIGTERM

. /opt/ros/melodic/setup.bash
. devel/setup.bash

ARGUMENTS=()
SCREEN=""
for arg; do
	case "$arg" in
		--*)
		ARGUMENTS+=("$arg")
		;;
		*)
		PICTURE="$arg"
		;;
	esac
done

if [ -z "$PICTURE" ]; then
	PICTURE="all"
fi

roslaunch master_project load_picture.launch --screen file:=$PICTURE