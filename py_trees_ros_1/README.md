**Py Trees Implementation with ROS 1**

TO install py_trees run 
```
pip3 install py_trees
```

We will be using https://github.com/TechnoYantra/igt_100. Launch the following files from the igt 100 package:

```
roslaunch igt_one_description gazebo.launch
roslaunch igt_one_nav map_server.launch
roslaunch igt_one_nav movebase_node.launch
roslaunch igt_one_nav amcl.launch

```
Once everything is running, run the following command to start the behavior tree:

```
rosrun py_trees_test test.py
```

If battery is low, the robot will complete the current goal and then stop.
Publish a value before 30 to stop execution. Then, publish something above 30 to resume.
Publish a battery level using:

```
rostopic pub battery_status py_trees_test/battery_status "battery_level: 10.0" 

```

If there's an emergency, the bot will halt immediately without completing the current task.
Use the following command to change the emergency status. In case of emergency,publish 'Yes'. To resume, publish 'No'.

```
rostopic pub emergency_status std_msgs/String "data: 'Yes'"

```

