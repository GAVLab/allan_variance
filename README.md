# allan_variance #
Code for IMU characterization using Allan variance techniques. Plays nicely with ROS bagfiles.

## Installing Dependencies ##

Install pip (if necessary)
```
sudo easy_install pip
```

Install allantools (https://github.com/aewallin/allantools)
```
sudo pip install allantools
```

## Usage ##

A launch file is provided for using the allan.py script. The script parses a bagfile and performs allan variance on the specified topic and IMU axis. 

### Parameters ###

~bagfile_path (string, default: "")

&nbsp;&nbsp;&nbsp;&nbsp;The global path to the bagile containing static IMU data.

~imu_topic_name (string, default: "/imu")

&nbsp;&nbsp;&nbsp;&nbsp;The name of the IMU topic within the bagfile. Note: topic must be of the sensor_msgs/Imu type.

~axis (int, default: 0)

  + Specify which axis/measurement to perform the allan variance. (0 = all six axes, 1 = x-accel, 2 = y-accel, 3 = z-accel, 4 = x-gyro, 5 = y-gyro, 6 = z-gyro).

~sample_rate (int, default: 100)

&nbsp;&nbsp;&nbsp;&nbsp;The sample rate of the IMU in Hz.

~delta_measurement (bool, default: false)

&nbsp;&nbsp;&nbsp;&nbsp;'true' if delta measurements (change in angle/velocity)

&nbsp;&nbsp;&nbsp;&nbsp;'false' if rate measurements (rotation rate/acceleration)

~number_of_lags (int, default: 1000)

&nbsp;&nbsp;&nbsp;&nbsp;The number of lags (tau values) used in calculating the allan variance.

~results_directory_path (string, default: None)

&nbsp;&nbsp;&nbsp;&nbsp;The path to the directory where the results should be stored. If left unspecified, a directory will be created in the workspace (e.g. ~/allan_ws/av_results/allan_accel_x.csv).

## To do ##

- write node that subscribes to IMU topic and calculates Allan variance periodically. Inform user when the AV is within an acceptable amount of error
- use allantools visualization to show a plot of the resulting allan variance and errors
- calculate values for random walk and bias instability based on result
- incorporate other intertial characterization techniques (simulation, Monte Carlo, autocorrelation, etc.)

## Maintainer ##
Dan Pierce (jdp0009@auburn.edu)
