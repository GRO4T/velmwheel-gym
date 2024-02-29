#!/bin/bash
cd $VELMWHEEL_ROOT
source source_me.bash
ros2 launch velmwheel_bringup bringup_navigation.launch.py \
	params_file:=src/velmwheel/bringup/velmwheel_bringup/config/nav2_params.yaml \
	map:=src/velmwheel/sim/velmwheel_gazebo/velmwheel_gazebo/gazebo/media/maps/test_map.yaml \
	use_sim_time:=True