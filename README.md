This is my workspace I built for development on the Oculus Rift while working
at University of Pennsylvania's GRASP lab in the summer of 2014.


Steps to build the main project in sandbox:

1) Add this to .bashrc:
    export LD_LIBRARY_PATH=%PROJECT_PATH%/REU_Summer_2014/sandbox/assimp-3.1.1/build/code/:$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=%PROJECT_PATH%/REU_Summer_2014/sandbox/glew-1.10.0/lib/:$LD_LIBRARY_PATH

2) Building assimp
    > cd assimp-3.1.1
    > mkdir build
    > cmake ..
    > make all

3) Building glew
    > cd glew-1.10.0
    > make

4) Building glfw
    > cd glfw-3.0.4
    > mkdir build
    > cmake ..
    > make all

**NOTE** GLM does not need to be build because it is an all header library

5) Building LibOVR (Oculus SDK)
    > cd LibOVR
    > make all

6) Building vrpn
    > cd sandbox
    > tar xzvf vrpn.tar.gz
    > cd vrpn
    > cd quat
    > make
    > cd ..
    > make
    > cd client_src
    > make
    > cd ../server_src
    > make
