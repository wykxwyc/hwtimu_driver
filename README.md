hwtimu_driver
===================================

Overview
-----------------------------------

hwtimu_driver is the implementation of the drivers and some IMU tools.

This stack contains:

 * `hwtimu`: a imu driver for GYROSCOPE.
 
 * `imu_tools`: IMU-related filters and visualizers.
 It contains: 
	1. `imu_filter_madgwick`: a filter which fuses angular velocities,
		accelerations, and (optionally) magnetic readings from a generic IMU 
		device into an orientation. Based on the work of [1].

	2. `imu_complementary_filter`: a filter which fuses angular velocities,
		accelerations, and (optionally) magnetic readings from a generic IMU 
		device into an orientation quaternion using a novel approach based on a complementary fusion. Based on the work of [2].

	3. `rviz_imu_plugin` a plugin for rviz which displays `sensor_msgs::Imu`
		messages
		
 * `memsic_driver`: a driver for memsic IMU.
 
 * `nodecbk`: a ROS node contains a callback function for ROS topic `/imu/data`.

How to use
-----------------------------------

 * hwtimu
```shell
catkin_make
roslaunch hwtimu hwtimusubexp.launch
```

 * imu_tools
 read the [README.md](!./imu_tools/README.md).
 
 * memsic_driver
```shell
rosrun memsic_driver imu_talker
```

 * nodecbk
```shell
rosrun nodecbk nodecbk
```


More info
-----------------------------------

* hwtimu

A driver fot both HWT901B(www.wit-motion.com) and ADIS16365.

* imu_tools:

https://github.com/ccny-ros-pkg/imu_tools
	
http://wiki.ros.org/imu_tools

License
-----------------------------------

* `imu_filter_madgwick`: currently licensed as GPL, following the original implementation
	 
* `imu_complementary_filter`: BSD

* `rviz_imu_plugin`: BSD

References
-----------------------------------



