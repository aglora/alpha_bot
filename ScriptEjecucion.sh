#!/bin/bash

source devel/setup.sh
catkin_make
roslaunch alpha_bot_simulation sim.launch
