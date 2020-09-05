## Rviz visualization

After the simulator (C++ server) and the predictor (python client) start, you can use this tool to visualize.

This visualizer will visit port `8086`.

## Prerequisites

- ros, rviz, ...

  - follow  http://wiki.ros.org/melodic/Installation/Ubuntu

  - ```
    sudo apt install ros-melodic-desktop
    ```

- python 2.7

  - requirement: `rospkg`
  - `lanelet2`, please follow  `https://github.com/fzi-forschungszentrum-informatik/Lanelet2` . I recommend that you can use `conda ` to build an `python 2.7` environment, then use `catkin_make` to install `lanelet2`; and finally use `catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/<distro> install` so `rospack` can find this package.

- copy `libjsoncpp.a` to `/usr/lib`

## Build and Run

1. deactivate conda, and use `catkin_make` to generate `./build` and `./devel`

```bash
conda deactivate
catkin_make
catkin_make
catkin_make		# You need run this command several times until success.
```

2. start visualizer (open 4 terminates)

```bash
roscore

rviz
# In the rviz graph interface, click `file` -> `open config` -> select `./my.rviz`

. ./devel/setup.bash
rosrun visualization read_osm_map.py
# then type 7 (MA)

. ./devel/setup.bash
rosrun visualization plotter_rviz.py
```

3. connect to server (make sure the server is already opened)

```bash
. ./devel/setup.bash
rosrun visualization visualization_client_node
```

