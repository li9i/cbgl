# CBGL

`cbgl` is a ROS package written in C++ that allows you to localise your 2D
LIDAR sensor in a given 2D map under global uncertainty in position and
orientation.

<!--
Click on the image for a brief demo
[![CBGL in Willowgarage](https://img.youtube.com/vi/DkKdxFNJG4g/maxresdefault.jpg)](https://youtu.be/DkKdxFNJG4g)
-->


https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9

## How to install `cbgl`

### Via traditional means

This package was tested and works under Ubuntu 16.04 and ROS kinetic (I'm so
sorry). You will need [`csm`](https://github.com/AndreaCensi/csm), `CGAL 4.7`
and `fftw3` as dependencies. Then, as always

```sh
cd ~/catkin_ws/src
git clone git@github.com:li9i/cbgl.git
cd cbgl; mv cbgl/* $PWD; rmdir cbgl; cd ../..
catkin build cbgl
```

For your convenience `cbgl` may also be installed and executed via Docker.

### Via Docker

If this is your first time running docker then I happen to find
[this](https://youtu.be/SAMPOK_lazw?t=67) docker installation guide very
friendly and easy to follow.

Build the image with the most recent code of this repository with:

```
git clone git@github.com:li9i/cbgl.git; cd cbgl
docker build --progress=plain --no-cache -t li9i/cbgl:latest .
```


## How to launch `cbgl`

Before launching your robot, cbgl, etc, you will need to

- export the map of your environment, say `map_x.pgm`, into a .`png` file
(that is `map_x.png`; `gimp` does it)
- place `map_x.png` into `cbgl/map/`
-  in file `configuration_files/params_cbgl.yaml`: set the `map_png_file`
variable to point to the absolute path of `map_x.png`
    - If you run cbgl via docker then the path needs to be expressed relative to the *container's path structure*, e.g.
    ```map_png_file: "/home/user_cbgl/catkin_ws/src/cbgl/map/map_x.png"```
    - If you run cbgl via traditional means then the path needs to be expressed relative to your own machine's path structure, e.g.
    ```map_png_file: "/home/me/catkin_ws/src/cbgl/map/map_x.png"```

### Via traditional means

```sh
roslaunch cbgl cbgl.launch
```

### Via Docker

```
docker run -it \
    --name=cbgl_container \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --rm \
    li9i/cbgl:latest
```

## How to call `cbgl`

Finally `cbgl` can be called as a service with

```sh
rosservice call /global_localization
```


## Citation
If you have used `cbgl` in your research please consider citing the following preprint:

```bibtex
@article{cbgl,
  title={CBGL: Fast Monte Carlo Passive Global Localisation of 2D LIDAR Sensor},
  author={Filotheou, Alexandros},
  journal={arXiv:2307.14247},
  year={2023}}
