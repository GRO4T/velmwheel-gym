#!/bin/bash
ps -ef | grep -E "(velmwheel|gzserver|gzclient|nav2)" | grep -vE "(grep|tensorboard|vim)" | awk '{ print $2 }' | xargs -I{} kill -9 {}
