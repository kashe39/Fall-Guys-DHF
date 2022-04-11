# streaming quaternion data 
from __future__ import print_function
from math import sqrt
from re import X
from tkinter import Y
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
import time
from time import sleep
from threading import Event

import platform
import sys
import numpy as np

import pandas as pd
import matplotlib.pyplot as plt


def kneeSensorStream():

    #if sys.version_info[0] == 2:
     #   range = xrange

    quatArrayW = []
    quatArrayX = []
    quatArrayY = []
    quatArrayZ = []


    class State:
        # init
        def __init__(self, device):
            self.device = device
            self.samples = 0
            self.callback = FnVoid_VoidP_DataP(self.data_handler)
        # callback
        def data_handler(self, ctx, data):
            print("QUAT: %s -> %s" % (self.device.address, parse_value(data)))
            # Store quaternion values in individual arrays 
            quatArrayW.append(round(copy.deepcopy(parse_value(data)).w, 5))
            quatArrayX.append(round(copy.deepcopy(parse_value(data)).x, 5))
            quatArrayY.append(round(copy.deepcopy(parse_value(data)).y, 5))
            quatArrayZ.append(round(copy.deepcopy(parse_value(data)).z, 5))
            self.samples+= 1
        

    states = []
    # connect
    address = "ED:2D:55:01:11:90"
    d = MetaWear(address)
    d.connect()
    print("Connected to " + d.address)
    states.append(State(d))

    # configure
    for s in states:
        print("Configuring device")
        # setup ble
        libmetawear.mbl_mw_settings_set_connection_parameters(s.device.board, 7.5, 7.5, 0, 6000)
        sleep(1.5)
        # setup quaternion
        libmetawear.mbl_mw_sensor_fusion_set_mode(s.device.board, SensorFusionMode.NDOF)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(s.device.board, SensorFusionAccRange._8G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(s.device.board, SensorFusionGyroRange._2000DPS)
        libmetawear.mbl_mw_sensor_fusion_write_config(s.device.board)
        # get quat signal and subscribe
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board,SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, s.callback)
        # start acc, gyro, mag
        libmetawear.mbl_mw_sensor_fusion_enable_data(s.device.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_sensor_fusion_start(s.device.board)
        # Light green led
        pattern = LedPattern()
        libmetawear.mbl_mw_led_load_preset_pattern(byref(pattern), LedPreset.SOLID)
        libmetawear.mbl_mw_led_write_pattern(s.device.board,byref(pattern), LedColor.GREEN)
        libmetawear.mbl_mw_led_play(s.device.board)
    
    # manual shut down
    #logState = input("Stop Logging?")
    #if logState == "Y": 
    sleep(1)
   
    # Convert Quaternions to Axis Angle 
    theta = []
    axisX = []
    axisY = []
    axisZ = []


    for i in range(0, len(quatArrayW)):
        if quatArrayW[i] == 1.0 and quatArrayX[i] == 0.0 and quatArrayY[i] == 0.0 and quatArrayZ[i] == 0.0:
            theta.append(0)
            axisX.append(1)
            axisY.append(0)
            axisZ.append(0)

        else:
            theta.append(2*np.arccos(quatArrayW[i]))
        if theta[i] != 0:
            # divide by user height for normalization
            axisX.append(quatArrayX[i]/(np.sin(theta[i]/2)))
            axisY.append(quatArrayY[i]/(np.sin(theta[i]/2)))
            axisZ.append(quatArrayZ[i]/(np.sin(theta[i]/2)))

    # test Axis Angle Output
    print("X' = ", axisX[0])
    print("Y' = ", axisY[0])
    print("Z' = ", axisZ[0])

   
    # tear down
    for s in states:
        # stop
        libmetawear.mbl_mw_sensor_fusion_stop(s.device.board)
        # unsubscribe to signal
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        libmetawear.mbl_mw_led_stop_and_clear(s.device.board)
        # disconnect
        libmetawear.mbl_mw_debug_disconnect(s.device.board)


    # recap
    print("Total Samples Received")
    for s in states:
        print("%s -> %d" % (s.device.address, s.samples))
    
    print("QW = ", quatArrayW[0])
    print("QX = ", quatArrayX[0])
    print("QY = ", quatArrayY[0])
    print("QZ = ", quatArrayZ[0])

    # Export Quaternions to CSV
    import csv
    log_time = time.asctime()
    filetime = log_time.replace(':', '_')

    # open the file in the write mode
    with open(r'C:\Users\kelto\OneDrive\Capstone\Project Code\Knee_Sensor_data_logs\Knee_data_stream_' + filetime + '.csv', 'w', encoding='UTF8') as f:
        # create the csv writer
        writer = csv.writer(f, delimiter = "," )

        # add headers to lists of quats
        header = ["QW", "QX", "QY", "QZ", "X", "Y", "Z"]
        writer.writerow(header)
        for i in range(1, len(quatArrayW)-1):
            # Write quat and axis angle data to csv
            writer.writerow([quatArrayW[i], quatArrayX[i], quatArrayY[i], quatArrayZ[i], axisX[i], axisY[i], axisZ[i]])
        
    
    #Plot received data
    plt.rcParams["figure.figsize"] = [10, 6]
    plt.rcParams["figure.autolayout"] = True
    df = pd.read_csv(r'C:\Users\kelto\OneDrive\Capstone\Project Code\Knee_Sensor_data_logs\Knee_data_stream_' + filetime + '.csv', usecols = header)

    #fig, (ax1, ax2) = plt.subplots(1, 1)
    #fig.suptitle('Knee Position (Axis Angle)')

    #ax1.plot(df[["QW", "QX", "QY", "QZ"]])
    #ax1.legend(["QW", "QX", "QY", "QZ"])
    #ax1.set_ylabel('Quaternions')

    plt.plot(df[["X", "Y", "Z",]])
    plt.xlabel('Index')
    plt.ylabel('Axis Angle')
    plt.legend(["X", "Y", "Z"])
    plt.title("Knee Position (Axis Angle)")

    plt.show()
    return(axisX, axisY, axisZ)

if __name__ == '__main__': 
    kneeSensorStream()
