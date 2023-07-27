# CBGL

https://i.imgur.com/7Ko3Vx0.mp4

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
