# open_manipulator_haptic_bridge
ROS node to interface my custom haptic device with Openmanipulator-X

The node uses the joint trajectory interface of Openmanipulator-X, to start the joint trajectory interface on the real robot:
```console
roslaunch open_manipulator_controllers joint_trajectory_controller.launch sim:=false
```

See details:  
https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controls/

This node works together with `mogi_haptic_device` package!
```console
roslaunch mogi_haptic_device haptic.launch
```

This node can be launched with:
```
rosrun open_manipulator_haptic_bridge open_manipulator_bridge
```