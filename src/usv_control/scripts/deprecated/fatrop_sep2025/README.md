# MPC Casadi Development

After running the asv_code_gen.py script inside this directory (``src/usv_control/scripts/mpc``), compile the generated C code with any of these commands:

```Shell
# TESTING (RECOMMENDED): Compile casadi C generated code with medium performance (faster compilation, recommended for general development)
gcc -fPIC -shared asv.c -g -O1 -march=native -lfatrop -lblasfeo -I/usr/local/include -L/usr/local/lib -o asv.so

# PERFORMANCE: Compile casadi C generated code with highest performance (slower compilation, recommended for best performance)
gcc -fPIC -shared asv.c -g -O3 -march=native -lfatrop -lblasfeo -I/usr/local/include -L/usr/local/lib -o asv.so
```