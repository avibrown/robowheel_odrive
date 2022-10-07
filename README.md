### Robowheel + ODrive + ROS2

![WhatsApp Video 2022-10-06 at 07 31 43](https://user-images.githubusercontent.com/63222803/194214835-836d72d2-6441-4f5e-8502-0a29d9c77815.gif)

This repository will contain configuration files, scripts, and eventually a ROS 2 node pertaining to the control of "RoboWheel" hub motors from [Sky's Edge](https://skysedge.com/robotics/robowheel170/index.html). specifically using the [ODrive](https://odriverobotics.com/shop/odrive-v36) brushless motor controller (anxiously awaiting the release of Sky's Edge [NearZero2](https://skysedge.com/robotics/nz2/index.html) controller!).

Big thanks to Andy (check out his robot donkeys [here](https://hackaday.io/project/187319-robot-donkeys)!) for sharing his ODrive configuration.

---

### To get moving
- Paste the "odrive_driver" directory to your `ros2_ws/src`
- Edit the serial number in `odrive_command.py` to match your own (instructions for getting SN are in the comments of the file)
- Build the package from within `ros2_ws` using `colcon build --packages-select odrive_driver`
- Run the node using
```
ros2 run odrive_driver odrive_node
```
- Then you can publish `vel` or `pos` commands to whichever axis you'd like, for instance:
```
ros2 topic pub -r 10 /axis0_vel_sub std_msgs/msg/Float32 "{data: 0.5}"
```
