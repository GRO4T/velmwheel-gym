#!/bin/bash
ps -ef | grep -E "(velmwheel|gzserver|gzclient|nav2)" | grep -v grep | awk '{ print $2 }' | xargs -I{} kill -9 {}
