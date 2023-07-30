# CBGL

`cbgl` is a ROS package that allows you to localise your 2D LIDAR sensor in
a given 2D metric map under global uncertainty in position and orientation.

<!--
Click on the image for a brief demo
[![CBGL in Willowgarage](https://img.youtube.com/vi/DkKdxFNJG4g/maxresdefault.jpg)](https://youtu.be/DkKdxFNJG4g)
-->


https://github.com/li9i/cbgl/assets/1658819/5794cd21-651d-4924-b453-25c46b9e42a9


## Dependencies

You will need `CGAL` and `fftw3` as dependencies.

## How to get and build

As always

```sh
cd ~/catkin_ws/src
git clone https://github.com/li9i/cbgl
cd ..
catkin build cbgl
```

## How to launch

Before launching your robot, map, etc, you will need to export your
`this_place_map.pgm` map into a `this_place_map.png` file. Then

```sh
roslaunch cbgl cbgl.launch
```

and

```sh
rosservice call /global_localization
```
