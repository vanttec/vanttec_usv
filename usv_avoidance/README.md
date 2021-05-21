# usv_avoidance pkg

## Setup
### Install Acados
```
  cd vanttec_usv_ws/src/usv_avoidance/
  git submodule update --recursive --init
  cd vanttec_usv_ws/src/usv_avoidance/include/acados/
  mkdir -p build
```
If on PC 64bit architecture platform:
```
  cd build
  cmake -DACADOS_WITH_QPOASES=ON -DACADOS_WITH_OSQP=OFF/ON ..
  make install 
```
If on JetsonTX2 platform:
```
  cd build
  cmake -DACADOS_WITH_QPOASES=ON -DBLASFEO_TARGET=ARMV8A_ARM_CORTEX_A57 ..
  make install 
```
Continue on all platforms: 
```
  cd vanttec_usv_ws/src/usv_avoidance/include/acados/interfaces/acados_template
  sudo python3.7 -m pip install -e .
  cd tera_renderer
  cargo build --verbose --release
```
Replace t_render file in `vanttec_usv_ws/src/usv_avoidance/include/acados/bin` with t_renderer file generated in `vanttec_usv_ws/src/usv_avoidance/include/acados/interfaces/acados_template/t_renderer`

### Run python script to generate C files
```
  cd vanttec_usv_ws/src/usv_avoidance/scripts/usv_guidance_ca1
  python3.7 main.py
```
### Compile workspace
```
  cd vanttec_usv_ws
  catkin_make
```
