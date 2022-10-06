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

