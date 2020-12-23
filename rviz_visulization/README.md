# Rviz Visualization

The `rviz_visualization` folder supports real-time visualization. After the simulator (the C++ side) and the predictor (the python side) start to work, you can use this tool to visualize.

**NOTE**: `rviz_visualization` requires the port `8086`. You have to run the simulator with `-r 8086`.

**NOTE:** If you feel hard to set the environment, you can refer to `py_replay`, a non-real-time visualization tool.

### Prerequisites

- ros, rviz, ...

  - follow  http://wiki.ros.org/melodic/Installation/Ubuntu

  - ```
    sudo apt install ros-melodic-desktop
    ```

- python 2.7

  - `rospkg`
  - `lanelet2`, please follow  `https://github.com/fzi-forschungszentrum-informatik/Lanelet2` . 
  - I recommend you to use `conda ` to build a `python 2.7` environment, then 
    - Use `catkin_make` to install `lanelet2`; 
    - Use `catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/<distro> install` to make `rospack`  link this package.
    

- copy `libjsoncpp.a` to `/usr/lib`

### Build and Run

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

