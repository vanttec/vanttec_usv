#!/bin/bash

#install acados
echo "Installing acados"
ORIG_DIR=$(pwd)
cd / 
sudo git clone https://github.com/acados/acados.git 
cd acados
git submodule update --recursive --init 
mkdir -p build
cd build
cmake -DACADOS_INSTALL_DIR=/acados ..
make install -j4 
sudo cp /acados/lib/*.so /usr/lib
sudo ldconfig

echo "ACADOS_SOURCE_DIR=/acados" | sudo tee -a /etc/environment
export ACADOS_SOURCE_DIR=/acados

echo "Setting up ROS WS"
cd ${ORIG_DIR}/..
mkdir -p ~/vanttec_usv_ws/src
mv vanttec_usv ~/vanttec_usv_ws/src/
cd ~/vanttec_usv_ws/src/vanttec_usv
git submodule update --init --recursive
cd ~/vanttec_usv_ws

echo "Compiling libacados_solver_usv_model_guidance_ca1.so"
cd ~/vanttec_usv_ws/src/vanttec_usv/usv_avoidance/scripts/usv_guidance_ca1/c_generated_code
make bundled_shared_lib

cd ~/vanttec_usv_ws

echo "Compiling project..."
max_retry=15
counter=0
until catkin_make
do
	sleep 1
	[[ counter -eq $max_retry ]] && echo "Failed!" && exit 1
	((counter++))
done