# -*- encoding: utf-8 -*-

import time

import odrive.core

my_drive = odrive.core.find_any(consider_usb=True, consider_serial=False, printer=print)

print("\n\n\n")

# my_drive.motor0.current_control.current_lim = 5.0
print("Current limit is " + str(my_drive.motor0.current_control.current_lim))

print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

# my_drive.motor0.pos_setpoint = 3.14
# print("Position setpoint is " + str(my_drive.motor0.pos_setpoint))


print("encoder offset " + str(my_drive.motor0.encoder.encoder_offset))
print("encoder pos " + str(my_drive.motor0.encoder.pll_pos))
print("error " + str(my_drive.motor0.error))

print("phase resistance " + str(my_drive.motor0.phase_resistance))


my_drive.motor0.vel_gain = 2 / 10000.0
print("vel_gain", my_drive.motor0.vel_gain)

# my_drive.motor0.pos_gain = 10.0

# my_drive.motor0.set_vel_setpoint(10000.0, 0.0)
# print("Velocity setpoint is " + str(my_drive.motor0.vel_setpoint))
# time.sleep(1)
# my_drive.motor0.set_vel_setpoint(0.0, 0.0)
# print("Velocity setpoint is " + str(my_drive.motor0.vel_setpoint))

# my_drive.motor0.pos_setpoint = 1000
# time.sleep(1)
# my_drive.motor0.pos_setpoint = 0.0


my_drive.motor0.set_current_setpoint(2.5)
print("current setpoint is " + str(my_drive.motor0.current_setpoint))
time.sleep(2)
my_drive.motor0.set_current_setpoint(0.0)
print("current setpoint is " + str(my_drive.motor0.current_setpoint))
