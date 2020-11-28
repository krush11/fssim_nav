# TDR-SDC's Software Stack

Welcome to the TDR-SDC Formula Student navigation repo
<br/>
Skeleton code forked from [f1tenth_simulator](https://github.com/f1tenth/f1tenth_simulator)

## Bulding code
```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/TDR-SDC/fssim_nav
cd ..
catkin_make
```
##### Run wall following node:
Run the two commands in different terminals:
```bash
roslaunch fssim_nav simulator.launch
```
```bash
roslaunch fssim_nav control.launch
```
