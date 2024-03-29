# CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor

#### [![arxiv.org](http://img.shields.io/badge/cs.RO-arXiv%3A2307.14247-B31B1B.svg)](https://arxiv.org/abs/2307.14247) [![youtube.com](https://img.shields.io/badge/2'_presentation-YouTube-FF0000)](https://www.youtube.com/watch?v=xaDKjI0WkDc)


`cbgl` is a ROS package written in C++ that allows you to localise your 2D
LIDAR sensor in a given 2D map under global uncertainty in position and
orientation in minimal time.

CBGL does not require motion for performing global localisation: it's a
one-shot approach that only requires a single laser scan measurement and the
map of the sensor's environment.  You can expect the execution time to roughly
have an order of magnitude of $`\text{area} \cdot \dfrac{N_s}{360} \cdot
10^{-2}`$ seconds, where $\text{area}$ is the area of the map's free space and
$`N_s`$ is the LIDAR's number of rays. (Strictly
speaking the execution time varies according to the geometry of the
environment and other factors.) In the video below the environment area is
$`2000`$ m$`^2`$ and localisation is performed in under four seconds.

<!--
Click on the image for a brief demo
[![CBGL in Willowgarage](https://img.youtube.com/vi/DkKdxFNJG4g/maxresdefault.jpg)](https://youtu.be/DkKdxFNJG4g)
-->
https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9

<sub>A panoramic 2D LIDAR sensor mounted on a turtlebot 2 is spawned into
an environment at a pose whose immediate surroundings are repeated in (almost)
the same geometry and proportions at locations other than the sensor's spawning
ground. The user calls the global localisation service once before moving the
robot at a second challenging pose, at which she calls it for a second time.
Both times `cbgl` is successful in estimating the sensor's pose. Sensor
characteristics: $`N_s = 360`$ rays; noise: $`\sim N (0.0, 0.05^2)`$ [m,m$`^2`$]</sub>





Table of Contents
=================
* [Install](#install)
  * [Via Docker](#via-docker)
  * [Via traditional means](#via-traditional-means)
* [Launch](#launch)
  * [Via Docker](#via-docker-1)
  * [Via traditional means](#via-traditional-means-1)
* [Call](#call)
* [Input/output at a glance](#inputoutput-at-a-glance)
* [Citation](#citation)


## Install

### Via Docker

If this is your first time running docker then I happen to find
[this](https://youtu.be/SAMPOK_lazw?t=67) docker installation guide very
friendly and easy to follow.

Build the image with the most recent code of this repository with:

```
git clone git@github.com:li9i/cbgl.git; cd cbgl
docker build --progress=plain --no-cache -t li9i/cbgl:latest .
```

### Via traditional means

This package was tested and works under Ubuntu 16.04 and ROS kinetic.
You will need [`csm`](https://github.com/AndreaCensi/csm), `CGAL 4.7`
and `fftw3` as dependencies. Then, as always

```sh
cd ~/catkin_ws/src
git clone git@github.com:li9i/cbgl.git
cd cbgl; mv cbgl/* $PWD; rmdir cbgl; cd ../..
catkin build cbgl
```

## Launch

Before launching your robot, `cbgl`, etc, you will need to

- export the map of your environment, say `map_x.pgm`, into a .`png` file
(that is `map_x.png`; `gimp` does it)
- place `map_x.png` into `cbgl/map/`
-  in file `configuration_files/params_cbgl.yaml`: set the `map_png_file`
   variable to point to the absolute path of `map_x.png`
    - If you run `cbgl` via docker then the path needs to be expressed relative to the *container's path structure*, e.g.
    ```map_png_file: "/home/user_cbgl/catkin_ws/src/cbgl/map/map_x.png"```
    - If you run `cbgl` via traditional means then the path needs to be expressed relative to your own machine's path structure, e.g.
    ```map_png_file: "/home/me/catkin_ws/src/cbgl/map/map_x.png"```

### Via Docker

```
docker run -it \
    --name=cbgl_container \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --rm \
    li9i/cbgl:latest
```
### Via traditional means


```sh
roslaunch cbgl cbgl.launch
```


## Call

Finally CBGL can be called as a service with

```sh
rosservice call /global_localization
```

## Input/output at a glance
- [IN]  A `sensor_msgs/LaserScan` message published through topic `configuration_files/scan_topic`
- [IN]  A `nav_msgs/OccupancyGrid` message published through topic `configuration_files/map_topic`
- [OUT] A `geometry_msgs/PoseWithCovarianceStamped` message published through topic `configuration_files/output_pose_topic`
- [OUT] The transform between the `odom` frame and the `map` frame if `configuration_files/tf_broadcast` is set to `true`


## Citation

```bibtex
@article{cbgl,
  title={CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor},
  author={Filotheou, Alexandros},
  journal={arXiv:2307.14247},
  url={https://arxiv.org/abs/2307.14247}
  year={2023}}
