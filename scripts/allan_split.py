#!/usr/bin/env python
import rospy
import sys
import allantools
import rosbag
import numpy as np
import csv
import rospkg
import os
import matplotlib.pyplot as plt  # only for plotting, not required for calculations
import math

def getRandomWalkSegment(tau,sigma,sigma_err):

    m = -0.5 # slope of random walk
    """""""""""""""""""""""""""""""""
    " Find point where slope = -0.5 "
    """""""""""""""""""""""""""""""""
    randomWalk = None
    i = 1
    idx = 1
    mindiff = 999
    logTau = -999
    while (logTau<0):
        logTau = math.log(tau[i],10)
        logSigma = math.log(sigma[i],10)
        prevLogTau = math.log(tau[i-1],10)
        prevLogSigma = math.log(sigma[i-1],10)
        slope = (logSigma-prevLogSigma)/(logTau-prevLogTau)
        diff = abs(slope-m)
        if (diff<mindiff):
            mindiff = diff
            idx = i
        i = i + 1

    """"""""""""""""""""""""""""""
    " Project line to tau = 10^0 "
    """"""""""""""""""""""""""""""
    x1 = math.log(tau[idx],10)
    y1 = math.log(sigma[idx],10)
    x2 = 0
    y2 = m*(x2-x1)+y1

    return (pow(10,x1),pow(10,y1),pow(10,x2),pow(10,y2),sigma_err[idx])

def getBiasInstabilityPoint(tau,sigma):
    i = 1
    while (i<tau.size):
        if (tau[i]>1) and ((sigma[i]-sigma[i-1])>0): # only check for tau > 10^0
            break
        i = i + 1
    return (tau[i],sigma[i])

def main(args):

    rospy.init_node('allan_variance_node')

    t0 = rospy.get_time()

    """"""""""""""
    " Parameters "
    """"""""""""""
    bagfile = rospy.get_param('~bagfile_path','~/data/static.bag')
    topic = rospy.get_param('~imu_topic_name','/imu')
    axis = rospy.get_param('~axis',0)
    sampleRate = rospy.get_param('~sample_rate',100)
    isDeltaType = rospy.get_param('~delta_measurement',False)
    numTau = rospy.get_param('~number_of_lags',1000)
    resultsPath = rospy.get_param('~results_directory_path',None)

    """"""""""""""""""""""""""
    " Results Directory Path "
    """"""""""""""""""""""""""
    if resultsPath is None:
        paths = rospkg.get_ros_paths()
        path = paths[1] # path to workspace's devel
        idx = path.find("ws/")
        workspacePath = path[0:(idx+3)]
        resultsPath = workspacePath + 'av_results/'

        if not os.path.isdir(resultsPath):
            os.mkdir(resultsPath)

    print "\nResults will be save in the following directory: \n\n\t %s\n"%resultsPath

    """"""""""""""""""
    " Form Tau Array "
    """"""""""""""""""
    taus = [None]*numTau

    cnt = 0;
    for i in np.linspace(-2.0, 5.0, num=numTau): # lags will span from 10^-2 to 10^5, log spaced
        taus[cnt] = pow(10,i)
        cnt = cnt + 1

    """""""""""""""""
    " Parse Bagfile "
    """""""""""""""""
    bag = rosbag.Bag(bagfile)

    Ntot = bag.get_message_count(topic) # number of measurement samples

    numChunks = 6
    finalTime = 6

    rwErrSum = np.zeros(numChunks)

    for chunk in range(1,numChunks):
        print "chunk = ",chunk

        N = int(Ntot*chunk/numChunks)

        print "N = ",N
        print "Ntot = ",Ntot

        data = np.zeros( (6,N) ) # preallocate vector of measurements

        if isDeltaType:
            scale = sampleRate
        else:
            scale = 1.0

        cnt = 0
        for topic, msg, t in bag.read_messages(topics=[topic]):
            if (cnt>(N-1)):
                break
            data[0,cnt] = msg.linear_acceleration.x * scale
            data[1,cnt] = msg.linear_acceleration.y * scale
            data[2,cnt] = msg.linear_acceleration.z * scale
            data[3,cnt] = msg.angular_velocity.x * scale
            data[4,cnt] = msg.angular_velocity.y * scale
            data[5,cnt] = msg.angular_velocity.z * scale
            cnt = cnt + 1

        print "[%0.2f seconds] Bagfile parsed\n"%(rospy.get_time()-t0)

        """"""""""""""""""
        " Allan Variance "
        """"""""""""""""""
        if axis is 0:
            currentAxis = 1 # loop through all axes 1-6
        else:
            currentAxis = axis # just loop one time and break

        while (currentAxis <= 6):
            (taus_used, adev, adev_err, adev_n) = allantools.oadev(data[currentAxis-1], data_type='freq', rate=float(sampleRate), taus=np.array(taus) )

            (rw_x0,rw_x1,rw_y0,rw_y1,rw_err) = getRandomWalkSegment(taus_used,adev,adev_err)
            biasInstabilityPoint = getBiasInstabilityPoint(taus_used,adev)

            randomWalk = rw_y1
            biasInstability = biasInstabilityPoint[1]
            
            rwErrSum[chunk-1] = rwErrSum[chunk-1] + abs(rw_err)

            currentAxis = currentAxis + 1 + axis*6 # increment currentAxis also break if axis is not =0

        print "finished processing for N = ",N,"/",Ntot
        """""""""""""""
        " Plot Result "
        """""""""""""""
        plt.figure(figsize=(12,8))
        ax = plt.gca()

        plt.plot(rwErrSum,'rx-',markeredgewidth=2.5,markersize=14.0)

        plt.title("Error vs. AV time")
        plt.xlabel('Dataset length [hours]')
        plt.ylabel('Error')

        for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                    ax.get_xticklabels() + ax.get_yticklabels()):
            item.set_fontsize(20)

        plt.show(block=False)

    bag.close()
    inp=raw_input("Press Enter key to close figures and end program\n")





if __name__ == '__main__':
  main(sys.argv)