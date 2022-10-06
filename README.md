### Robowheel + ODrive + ROS2

![WhatsApp Video 2022-10-06 at 07 31 43](https://user-images.githubusercontent.com/63222803/194214835-836d72d2-6441-4f5e-8502-0a29d9c77815.gif)

This repository will contain configuration files and nodes pertaining to the control of "RoboWheel" hub motors from [Sky's Edge](https://skysedge.com/robotics/robowheel170/index.html). specifically using the [ODrive](https://odriverobotics.com/shop/odrive-v36) brushless motor controller (anxiously awaiting the release of Sky's Edge [NearZero2](https://skysedge.com/robotics/nz2/index.html) controller!), and ROS 2.

Big thanks to Andy (check out his robot donkeys [here](https://hackaday.io/project/187319-robot-donkeys)!) for sharing his ODrive configuration.

---

### Flashing config file

You can put the `.json` config file on your ODrive by saving the file where you'd like, opening a terminal from the save directory, and running:

`odrivetool restore-config NAME_OF_FILE.json`

changing `NAME_OF_FILE` of course... This is assuming you've installed `odrivetools`.

---

### Motor & encoder calibration

After flashing the config file, perform the following (this is for calibration both axes):

```
# Motor calibration
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_MOTOR_CALIBRATION

dump_errors(odrv0) # Check if there were any errors in calibration

odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis1.motor.config.pre_calibrated = True

odrv0.save_configuration()
odrv0.reboot()

# Encoder offset calibration
odrv0.axis0.encoder.config.calib_range = 0.05
odrv0.axis1.encoder.config.calib_range = 0.05

odrv0.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
```

---

### Moving the motors

```
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrv0.axis0.controller.input_vel = 0.5
odrv0.axis1.controller.input_vel = 0.5

# Your motors should be spinning...
```

