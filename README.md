# Amazon_Deepracer_Simulation

## Overview

Opensource code for National Undergraduate Electronic Design Contest, Amazon Deepracer Track, Simulation Tasks.

Tasks:

- Simulation environment construction
- Drive Deepracer car in the simulation environment
- QR code detection and lane change
- Stop sign detection and break the car
- Multi car following

*! Warning: the simulation result may vary among different systems. We sacrificed robustness for higher speed performance in the contest. You may need to modify speed and PID params base on your requirements. Thanks! :)*

## Requirements

- ROS1 Noetic Desktop (with gazebo)
- openvino 2021.4
- openvino-dev 2021.4.1
- ssdlite_mobilenet_v2 model
  - trained on COCO, available from [open model zoo](https://docs.openvino.ai/2021.4/model_zoo.html)
  - convert it into Openvino Inference Engine format

### deepracer_single_race1

```
cd deepracer_single
catkin_make
source devel/setup.bash
roslaunch raceworld raceworld1.launch
rosrun raceworld follow_raceworld1.py
```

### deepracer_single_race2

```
cd deepracer_single
catkin_make
source devel/setup.bash
roslaunch raceworld raceworld2.launch
rosrun raceworld follow_raceworld2.py
```

### deepracer_multiple

```
cd deepracer_multiple
catkin_make
source devel/setup.bash
roslaunch raceworld raceworld_multi.launch
rosrun raceworld follow_switch_multi.py
```
