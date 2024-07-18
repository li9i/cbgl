# Install and run natively

This package was tested and works under Ubuntu 16.04 and ROS kinetic. You will need [`csm`](https://github.com/AndreaCensi/csm), `CGAL 4.7` and `fftw3` as dependencies. 

## Install 

```sh
cd ~/catkin_ws/src
git clone git@github.com:li9i/cbgl.git
cd cbgl; mv cbgl/* $PWD; rmdir cbgl; cd ../..
catkin build cbgl
```

## Run

### Launch


```sh
roslaunch cbgl cbgl.launch
```

### Call

Launching `cbgl` simply makes it go into stand-by mode and does not actually execute global localisation. To do so simply call the provided service

```sh
rosservice call /global_localization
```
