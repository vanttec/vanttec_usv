FROM stereolabs/zed:3.4-ros-devel-cuda10.2-ubuntu18.04
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update -y && \
	apt-get upgrade -y && \
	apt-get install ros-melodic-pcl-ros ros-melodic-roslint ros-melodic-angles libyaml-cpp-dev libpcap-dev -y && \
	rm -rf /var/lib/apt/lists/*

RUN cd / && git clone https://github.com/acados/acados.git && cd acados && ls && \
	git submodule update --recursive --init && mkdir -p build && cd build && \
	cmake -DACADOS_INSTALL_DIR=/acados .. && make install -j4 && cp /acados/lib/*.so /usr/lib && \
	ldconfig

RUN pip3 install wheel && pip3 install -e /acados/interfaces/acados_template

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/acados/lib
ENV ACADOS_SOURCE_DIR=/acados
