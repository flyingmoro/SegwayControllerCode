# -*- encoding: utf-8 -*-

import time

import odrive.core

my_drive = odrive.core.find_any(consider_usb=True, consider_serial=False, printer=print)
print(dir(my_drive.motor0))
print("\n\n\n")

# my_drive.motor0.current_control.current_lim = 5.0
print("Current limit is " + str(my_drive.motor0.current_control.current_lim))
print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
print("encoder offset0 " + str(my_drive.motor0.encoder.encoder_offset))
print("encoder offset1 " + str(my_drive.motor1.encoder.encoder_offset))
print("encoder dir0 " + str(my_drive.motor0.encoder.motor_dir))
print("encoder dir1 " + str(my_drive.motor1.encoder.motor_dir))
print("encoder pos0 " + str(my_drive.motor0.encoder.pll_pos))
print("encoder pos1 " + str(my_drive.motor1.encoder.pll_pos))
print("phase resistance0 " + str(my_drive.motor0.phase_resistance))
print("phase resistance1 " + str(my_drive.motor1.phase_resistance))
print("error0 " + str(my_drive.motor0.error))
print("error1 " + str(my_drive.motor1.error))

print()

# my_drive.motor0.vel_gain = 2 / 10000.0
# print("vel_gain", my_drive.motor0.vel_gain)

# my_drive.motor0.pos_gain = 10.0

# my_drive.motor0.set_vel_setpoint(10000.0, 0.0)
# print("Velocity setpoint is " + str(my_drive.motor0.vel_setpoint))
# time.sleep(1)
# my_drive.motor0.set_vel_setpoint(0.0, 0.0)
# print("Velocity setpoint is " + str(my_drive.motor0.vel_setpoint))

# my_drive.motor0.pos_setpoint = 3000
# time.sleep(1)
# my_drive.motor0.pos_setpoint = 0.0

# my_drive.anti_cogging_calibration(0)

# my_drive.motor0.control_mode = 1 # current control
# my_drive.motor1.control_mode = 1 # current control
# my_drive.motor0.control_mode = 2   # vel control
# my_drive.motor1.control_mode = 2   # vel control
# my_drive.motor0.control_mode = 3   # position control
# my_drive.motor1.control_mode = 3   # position control


# my_drive.motor0.pos_gain = 400.0
# my_drive.motor0.pos_gain = 400.0

# my_drive.motor0.set_vel_setpoint(000.0, 0.0)

my_drive.motor0.current_control.p_gain = 0.15
my_drive.motor0.current_control.i_gain = 200.0
my_drive.motor0.pos_gain = 50.0
my_drive.motor0.pos_gain_i = 500.0
my_drive.motor0.pos_gain_d = 0.05

my_drive.motor1.current_control.p_gain = 0.15
my_drive.motor0.current_control.i_gain = 0.0
my_drive.motor1.pos_gain = 50
my_drive.motor1.pos_gain_i = 500.0
my_drive.motor1.pos_gain_d = 0.05


print("p_gain_0 {}".format(my_drive.motor0.current_control.p_gain))
print("i_gain_0 {}".format(my_drive.motor0.current_control.i_gain))
print("pos_gain_i_0 {}".format(my_drive.motor0.pos_gain_i))
print("pos_gain_d_0 {}".format(my_drive.motor0.pos_gain_d))
print("pos_setpoint_0 {}".format(my_drive.motor0.pos_setpoint))
print("control_mode_0 {}".format(my_drive.motor0.control_mode))
print()
print("p_gain_1 {}".format(my_drive.motor1.current_control.p_gain))
print("i_gain_1 {}".format(my_drive.motor1.current_control.i_gain))
print("pos_gain_i_1 {}".format(my_drive.motor1.pos_gain_i))
print("pos_gain_d_1 {}".format(my_drive.motor1.pos_gain_d))
print("pos_setpoint_1 {}".format(my_drive.motor1.pos_setpoint))
print("control_mode_1 {}".format(my_drive.motor1.control_mode))

my_drive.motor0.set_current_setpoint(100.0)
print("current setpoint is " + str(my_drive.motor0.current_setpoint))
time.sleep(3)
my_drive.motor0.set_current_setpoint(0.0)
print("current setpoint is " + str(my_drive.motor0.current_setpoint))
#
# print("error0 " + str(my_drive.motor0.error))
# print("error1 " + str(my_drive.motor1.error))

# myMotor = getattr(my_drive, "motor0")
# myMotorError = getattr(myMotor, "error")
# print(myMotorError)

# my_drive.print("t")

# my_drive.motor0.anticogging.calib_anticogging = True