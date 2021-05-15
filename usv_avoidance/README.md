# usv_avoidance pkg

## Setup
```
  cd catkin_ws/src/usv_avoidance/
  git submodule update --recursive --init
  cd catkin_ws/src/usv_avoidance/acados/
  mkdir -p build
  cd build
  cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON ..
  make install 
```