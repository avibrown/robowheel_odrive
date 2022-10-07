#!/usr/bin/env python3

import sys
from time import sleep
import odrive
from odrive.enums import *

'''
Find ODrive serial number (need it in hex format) in odrivetool by running:
    hex(odrv0.serial_number).split('x')[1].upper()
'''
ODRIVE_SERIAL_NUMBER = "364D38623030"

class ODriveController:
    def __init__(self):
        print(f'Attempting to connect to ODrive {ODRIVE_SERIAL_NUMBER}')
        try:
            self.odrive = odrive.find_any(serial_number=ODRIVE_SERIAL_NUMBER)
            print(f'Connected to ODrive {ODRIVE_SERIAL_NUMBER}')
        except Exception as e:
            print(f'Failed to connect to ODrive: {e}')

        self.axis0 = self.odrive.axis0
        self.axis1 = self.odrive.axis1
        self.axes = {0: self.odrive.axis0,
                     1: self.odrive.axis1}

        self.armed_vel = False
        self.armed_pos = False
        self.calibration_override_timer = 10

    def encoder_offset_calibration(self, axes=[0, 1], calibration_override=False):
        self.armed = False

        if not calibration_override:
            for axis in axes:
                if not self.axes[axis].encoder.is_ready:
                    print(f'Calibrating encoder on axis{axis}')
                    self.axes[axis].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
                else:
                    pass
            while len(axes):
                for axis in axes:
                    if self.axes[axis].encoder.is_ready:
                        axes.remove(axis)
        else:
            for axis in axes:
                print(f'Calibrating encoder on axis{axis}')
                self.axes[axis].requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION

            print(f'Waiting {self.calibration_override_timer} seconds for calibration to complete...')
            sleep(self.calibration_override_timer)
        
        print('Encoders calibrated.')

    def arm_velocity_control(self, axes=[0, 1]):
        for axis in axes:
            print(f'Arming axis{axis}')
            self.axes[axis].controller.input_vel = 0
            self.axes[axis].controller.config.input_mode = INPUT_MODE_VEL_RAMP
            self.axes[axis].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.armed_vel = True

    def arm_position_control(self, axes=[0, 1]):
        for axis in axes:
            print(f'Arming axis{axis}')
            self.axes[axis].controller.input_pos = 0
            self.axes[axis].controller.config.input_mode = INPUT_MODE_POS_FILTER
            self.axes[axis].requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.armed_pos = True

    def command_velocity(self, axis, velocity):
        if self.armed_vel:
            print(f'Commanding velocity {velocity} on axis{axis}')
            self.axes[axis].controller.input_vel = velocity

    def command_position(self, axis, position):
        if self.armed_pos:
            self.axes[axis].controller.input_pos = position

    def get_velocity(self, axis):
        return self.axes[axis].encoder.vel_estimate

    def get_position(self, axis):
        return (self.axes[axis].encoder.pos_estimate)

    def get_errors(self, axes=[0, 1]):
        for axis in axes:
            print(f'\nGetting errors for axis{axis}:')
            print('Axis: ' + str(hex(self.axes[axis].error)))
            print('Motor: ' + str(hex(self.axes[axis].motor.error)))
            print('Controller :' + str(hex(self.axes[axis].controller.error)))
            print('Encoder: ' + str(hex(self.axes[axis].encoder.error)))

if __name__ == '__main__':
    import math
    odrv0 = ODriveController()
    odrv0.encoder_offset_calibration()
    odrv0.arm_position_control()
    sleep(2)

    position = 0
    while True:
        odrv0.command_position(0, 3 * math.cos(position))
        odrv0.command_position(1, 3 * math.sin(position))

        print(odrv0.get_position(0))
        position += math.pi / 100
        sleep(0.1)

    # position = 1
    # while True:
    #     odrv0.command_position(0, position)
    #     odrv0.command_position(1, position)

    #     print(odrv0.get_position(0))
    #     position *= -1
    #     sleep(5)