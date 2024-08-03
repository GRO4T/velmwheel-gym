#!/bin/bash
cd $VELMWHEEL_ROOT
source source_me.bash
bringup_sim test.world with_gui:=true with_rviz:=false
