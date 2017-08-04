#!/usr/bin/env python
import rospy
import allantools
import rosbag
import numpy as np
import csv
import rospkg
import os
import matplotlib.pyplot as plt  # only for plotting, not required for calculations

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

N = bag.get_message_count(topic) # number of measurement samples

data = np.zeros( (6,N) ) # preallocate vector of measurements

if isDeltaType:
    scale = sampleRate
else:
    scale = 1.0

cnt = 0
for topic, msg, t in bag.read_messages(topics=[topic]):
    data[0,cnt] = msg.linear_acceleration.x * scale
    data[1,cnt] = msg.linear_acceleration.y * scale
    data[2,cnt] = msg.linear_acceleration.z * scale
    data[3,cnt] = msg.angular_velocity.x * scale
    data[4,cnt] = msg.angular_velocity.y * scale
    data[5,cnt] = msg.angular_velocity.z * scale
    cnt = cnt + 1

bag.close()

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

    """""""""""""""
    " Save as CSV "
    """""""""""""""
    if (currentAxis==1):
        fname = 'allan_accel_x'
        title = 'Allan Deviation: Accelerometer X'
    elif (currentAxis==2):
        fname = 'allan_accel_y'
        title = 'Allan Deviation: Accelerometer Y'
    elif (currentAxis==3):
        fname = 'allan_accel_z'
        title = 'Allan Deviation: Accelerometer Z'
    elif (currentAxis==4):
        fname = 'allan_gyro_x'
        title = 'Allan Deviation: Gyroscope X'
    elif (currentAxis==5):
        fname = 'allan_gyro_y'
        title = 'Allan Deviation: Gyroscope Y'
    elif (currentAxis==6):
        fname = 'allan_gyro_z'
        title = 'Allan Deviation: Gyroscope Z'

    print "[%0.2f seconds] Finished calculating allan variance - writing results to %s"%(rospy.get_time()-t0,fname)

    f = open(resultsPath + fname + '.csv', 'wt')

    try:
        writer = csv.writer(f)
        writer.writerow( ('Tau', 'AllanDev', 'AllanDevError', 'AllanDevN') )
        for i in range(taus_used.size):
            writer.writerow( (taus_used[i],adev[i],adev_err[i],adev_n[i])  )
    finally:
        f.close()

    """""""""""""""
    " Plot Result "
    """""""""""""""
    plt.figure(figsize=(12,8))
    ax = plt.gca()
    ax.set_yscale('log')
    ax.set_xscale('log')

    plt.plot(taus_used,adev)
    plt.grid(True, which="both")
    plt.title(title)
    plt.xlabel('Tau (s)')
    plt.ylabel('ADEV')

    for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
                ax.get_xticklabels() + ax.get_yticklabels()):
        item.set_fontsize(20)

    plt.show(block=False)

    plt.savefig(resultsPath + fname)

    currentAxis = currentAxis + 1 + axis*6 # increment currentAxis also break if axis is not =0

inp=raw_input("Press Enter key to close figures and end program\n")