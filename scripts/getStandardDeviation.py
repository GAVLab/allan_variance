#!/usr/bin/env python
import rospy
import sys
import rosbag
import numpy as np
import csv
import rospkg
import os
import matplotlib.pyplot as plt  # only for plotting, not required for calculations
import math

def main(args):

    bagfile = args[1]
    topic = args[2]

    """""""""""""""""
    " Parse Bagfile "
    """""""""""""""""
    bag = rosbag.Bag(bagfile)

    N = bag.get_message_count(topic) # number of measurement samples

    data = np.zeros( (6,N) ) # preallocate vector of measurements

    cnt = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data[0,cnt] = msg.linear_acceleration.x
        data[1,cnt] = msg.linear_acceleration.y
        data[2,cnt] = msg.linear_acceleration.z
        data[3,cnt] = msg.angular_velocity.x
        data[4,cnt] = msg.angular_velocity.y
        data[5,cnt] = msg.angular_velocity.z
        cnt = cnt + 1

    bag.close()

    print data[0].shape

    stdDevAccX = np.std(data[0])
    stdDevAccY = np.std(data[1])
    stdDevAccZ = np.std(data[2])
    stdDevGyrX = np.std(data[3])
    stdDevGyrY = np.std(data[4])
    stdDevGyrZ = np.std(data[5])

    print "stdDevAccX = ",stdDevAccX
    print "stdDevAccY = ",stdDevAccY
    print "stdDevAccZ = ",stdDevAccZ
    print "stdDevGyrX = ",stdDevGyrX
    print "stdDevGyrY = ",stdDevGyrY
    print "stdDevGyrZ = ",stdDevGyrZ

    stdDevAcc = (stdDevAccX+stdDevAccY+stdDevAccZ)/3.
    stdDevGyr = (stdDevGyrX+stdDevGyrY+stdDevGyrZ)/3.

    print "\nstdDevAcc = ",stdDevAcc
    print "stdDevGyr = ",stdDevGyr

    """""""""""""""
    " Plot Result "
    """""""""""""""
    # # Accel X
    # plt.figure(figsize=(12,8))

    # plt.plot(data[0])
    # plt.plot([0,N],[3*stdDevAccX+np.mean(data[0]),3*stdDevAccX+np.mean(data[0])],'r--')
    # plt.plot([0,N],[-3*stdDevAccX+np.mean(data[0]),-3*stdDevAccX+np.mean(data[0])],'r--')

    # plt.title("Accel X")
    # plt.legend(["data","3 sigma"])

    # plt.show(block=False)

    # # Accel Y
    # plt.figure(figsize=(12,8))

    # plt.plot(data[1])
    # plt.plot([0,N],[3*stdDevAccY+np.mean(data[1]),3*stdDevAccY+np.mean(data[1])],'r--')
    # plt.plot([0,N],[-3*stdDevAccY+np.mean(data[1]),-3*stdDevAccY+np.mean(data[1])],'r--')

    # plt.title("Accel Y")
    # plt.legend(["data","3 sigma"])

    # plt.show(block=False)

    # # Accel Z
    # plt.figure(figsize=(12,8))

    # plt.plot(data[2])
    # plt.plot([0,N],[3*stdDevAccZ+np.mean(data[2]),3*stdDevAccZ+np.mean(data[2])],'r--')
    # plt.plot([0,N],[-3*stdDevAccZ+np.mean(data[2]),-3*stdDevAccZ+np.mean(data[2])],'r--')

    # plt.title("Accel Z")
    # plt.legend(["data","3 sigma"])

    # plt.show(block=False)





    # plt.savefig(resultsPath + fname)

    # currentAxis = currentAxis + 1 + axis*6 # increment currentAxis also break if axis is not =0

    inp=raw_input("Press Enter key to close figures and end program\n")

if __name__ == '__main__':
  main(sys.argv)