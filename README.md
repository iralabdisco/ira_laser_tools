This library includes some tools for laser handling in ROS. At the moment two nodes are available:

 - laser_multi_merger
 - laser_virtualizer

Both use part of the pointcloud_to_laserscan code available in ROS.

laser_multi_merger allows to easily and dynamically (rqt_reconfigure) merge multiple laser scans into a single one, which results very useful for applications like gmapping or amcl and pamcl that require a laser scan as input.

laser_virtualizer allows to easily and dynamically (rqt_reconfifure) generate virtual laser scans from a pointcloud (such as a velodyne one). The only requirement is the rototranslation between the virtual laser scanner and the base frame to be known to TF.

Both nodes compile under catkin in hydro and use PCL1.7

The documentation is at the moment very brief, for any question please contact us at furlan@disco.unimib.it or ballardini@disco.unimib.it


