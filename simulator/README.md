# simulator #

prediction-challenge simulator application

## Prerequisites ##

 - libprotobuf-c-dev v1.2.1-2
 - libgrpc++-dev v1.3.2-1.1~build1
 - docker
 - gcc & g++ with c++14 support

## Build and Run ##

**To build and run the simulator locally:**
 ```bash
 make
 ./simulator
 ```

**To build and run the simulator with docker:**
 ```bash
 cd depoly
 ./make_image.sh
 ```
