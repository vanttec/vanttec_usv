#!/bin/bash

cd
mkdir vanttec_usv_ws
mv vanttec_usv vanttec_usv_ws/src
cd vanttec_usv_ws/src
git submodule update --init --recursive
cd 
cd vanttec_usv_ws
catkin_make
