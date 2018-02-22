# -*- encoding: utf-8 -*-

import time
import odrive.core
import matplotlib.pyplot as plt
import numpy as np
import threading

data_rate = 100
plot_rate = 10
num_samples = 1000

my_odrive = odrive.core.find_any()

plt.ion()
global vals
global vals2
vals = []
vals2 = []

def fetch_data():
    global vals
    global vals2
    while True:
        vals.append(my_odrive.motor0.current_meas_phB)
        vals2.append(my_odrive.motor0.current_meas_phC)
        # vals.append(my_odrive.motor0.encoder.pll_pos)
        if len(vals) > num_samples:
            vals = vals[-num_samples:]
            vals2 = vals2[-num_samples:]
        time.sleep(1/data_rate)

def plot_data():
    global vals
    while True:
        try:
            plt.clf()
            plt.plot(vals)
            plt.plot(vals2)
            plt.pause(1/plot_rate)
        except:
            with open("lastMeasurement.csv", "w")as f:
                for i in range(len(vals)):
                    f.write("{},{}\n".format(vals[i], vals2[i]))

fetch_thread = threading.Thread(target=fetch_data, daemon=True)
fetch_thread.start()

plot_data()