#!/bin/bash

cd
mkdir -p vanttec_usv_ws
cd vanttec_usv_ws
catkin_make
cd
mv vanttec_usv vanttec_usv_ws/src
cd vanttec_usv_ws/src
git submodule update --init --recursive