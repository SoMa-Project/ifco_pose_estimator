# ifco_pose_estimator

ROS package that estimates the pose of an IFCO with respect to a camera.

## Installation

* Installing the following packages via `apt` should be enough for building `ifco_pose_estimator`:
	- ros-ROS_DISTRO-ros-core
	- ros-ROS_DISTRO-pcl-ros
	- ros-ROS_DISTRO-moveit
* `ifco_pose_estimator` also uses the `OpenCV`, `Boost` and `PCL` libraries, which will be installed by installing the packages above.  
* Clone this repository into a catkin workspace and run `catkin_make`.
* The package has been tested on Ubuntu 14.04 with ROS Indigo and Ubuntu 16.04 with ROS Kinetic.

## Functionality

The package performs the following steps:

* The `ifco_pose_server` subscribes to the `pcl_topic` and keeps only the points of the point cloud that might correspond to the IFCO based on thresholding of the A channel of an image of the scene in LAB format.
* The `ifco_pose_server` fits an IFCO model to the resultant point cloud using ICP and the `ifco_pose` service returns the estimated `pose` of the IFCO with respect to the camera and the achieved `fitness`. The IFCO coordinate frame is placed on the IFCO base with the z axis pointing away from the opening of the IFCO and the x axis pointing along the long side of the IFCO.
* If the `ifco_pose` service was called with the parameter `publish_ifco` set to `True`, then the `ifco_pose_server` will also publish the IFCO model in the planning scene.

## Node input

The user must provide the node with the following information (usually via `launch_ifco_pose_server.launch` ):

* A `pcl_topic` to subscribe to.
* Parameters `A_high` and `A_low` with which the aforementioned thresholding will be performed.
* Maximum number of `icp_iterations` that ICP will perform to converge.
* An `initial_pose_estimate` of the pose of the IFCO with respect to the camera.
* Parameter `use_initial_pose_estimate` to denote whether the user's `initial_pose_estimate` will be used.

## Notes

* All topics and tfs referenced above are expressed wrt the `camera` frame. This is the `camera_rgb_optical_frame` for the Primesense or the `kinect_ir_frame` for the Kinect.
* If `use_initial_pose_estimate` is set to `0`, then the node will try to compute an estimate of the IFCO pose by averaging the position of the points of the point cloud that might correspond to the IFCO and rotating the model by -45 degrees along the x axis of the IFCO (this is based on the assumption that the camera will be usually overlooking the IFCO).

## How to run

* Create a node that launches your camera and maps the proper frame to the `camera` frame.
* Point the camera to an IFCO and do `roslaunch ifco_pose_estimator launch_ifco_pose_server.launch`
* Open Rviz.
* Do `rosrun ifco_pose_estimator ifco_pose_client` to test the node.