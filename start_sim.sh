#!/bin/bash
cd $VELMWHEEL_ROOT
source source_me.bash
bringup_sim test8x.world with_gui:=false
