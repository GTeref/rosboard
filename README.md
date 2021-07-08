# ROSboard 

ROS node that runs a web server on your robot.
Run the node, point your web browser at http://your-robot-ip:8888/ and you get nice visualizations.

This repo is currentl under heavy overhauling. There *were* nice visualizations for PointCloud/LaserScan/NavSatFix/etc. but due to complete restructuring to be ROS1+ROS2 compatible and less dependent on external ROS packages, they are not available temporarily. Use ane earlier commit if you are looking for those. I'll bring them back soon under the new framework.

**ROS1/ROS2 compatible.** This package will work in either ROS version.

**Mobile friendly.** Designed so you can walk around next to your robot with a phone while viewing ROS topics.

**Light weight.** Doesn't depending on much. Consumes extremely little resources when it's not actually being used.

You can run it on your desktop too and play a ROS bag.

Also be sure to check out my terminal visualization tool, [ROSshow](https://github.com/dheera/rosshow/).

![screenshot](/screenshots/screenshot1.jpg?raw=true "screenshot")

## ROS1 setup

1. Prerequisites:

```
sudo apt install python3-pip
sudo pip3 install rospkg
sudo pip3 install tornado
```

2. Run `./configure-ros1` after cloning this repo. It should now be a valid ROS1 package. Throw it inside your catkin workspace, then run `catkin_make`, `source devel/setup.bash`, the usual stuff.

3. `rosrun rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888 (or replace localhost with your robot's IP)

## ROS2 setup

1. Prerequisites:

```
sudo pip3 install tornado
```

2. Run `./configure-ros2` after cloning this repo. It should now be a valid ROS2 package. Throw it inside your colcon workspace, then run `colcon build`, `source install/setup.bash`, the usual stuff.

3. `ros2 run rosboard rosboard_node` or put it in your launch file

4. Point your web browser at http://localhost:8888 (or replace localhost with your robot's IP)

## FAQ

**How do I write a visualizer for a custom type?**

Just add a new viewer class that inherits from Viewer, following the examples of the [default viewers](https://github.com/dheera/rosboard/tree/master/rosboard/html/js/viewers). Then add it to the imports at the top of [index.js](https://github.com/dheera/rosboard/blob/master/rosboard/html/js/index.js) and you're done.

**How does this work in both ROS1 and ROS2?**

I make use of [rospy2](https://github.com/dheera/rospy2), a shim library I wrote that behaves like ROS1's `rospy` but speaks ROS2 to the system, communicating with `rclpy` in the background. This allows using the same ros node code for both ROS1 and ROS2, and only needs slight differences in the package metadata files (`package.xml` and `CMakeLists.txt`, hence the configure scripts). It does mean that everything is written in ROS1 style, but it ensures compatibility with both ROS1 and ROS2 without having to maintain multiple branches or repos.

**Why don't you use rosbridge-suite or Robot Web Tools?**

They are a great project, I initially used it, but moved away from it in favor of a custom Tornado-based websocket bridge, for a few reasons:

* It's less easy to be ROS1 and ROS2 compatible when depending on these libraries. The advantage of doing my own websocket implementation in Tornado is that the custom websocket API speaks exactly the same language regardless of whether the back-end is ROS1 or ROS2.

* Custom implementation allows me to use lossy compression on large messages (e.g. Image or PointCloud2) before sending it over the websocket, robot-side timestamps on all messages, and possibly throttling in the future.

* I don't want the browser to have "superuser" access to the ROS system, only have the functionality necessary for this to work.

* I also want to add a basic username/password authorization at some point in the future.

* Many times in the past, the robot web tools are not available immediately on apt-get when ROS distros are released, and one has to wait months. This depends on only some standard Python libraries like `tornado` and optionally `PIL` and does not depend on any distro-specific ROS packages, so it should theoretically work immediately when new ROS distros are released.
