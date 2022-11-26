# securityRobot

This package was developed in order to learn nav2 stack. The robot should move the pose where the user click and publish point in RVIZ simulation tool. 

# Spawn the robot in Gazebo environment

ros2 launch two_wheeled_robot launch_urdf_into_gazebo.launch.py

This launch file will spawn the specified robot by using a urdf file into a specified world in Gazebo. I have used ready-built Amazon-Home. A screenshot has been attached after this command. 

**![](https://user-images.githubusercontent.com/60695165/185794352-95ec39cf-955c-47a0-a9c2-953a5cdb4381.png)**

# Run the navigation lifecycle node 

ros2 launch two_wheeled_robot nav2_lifecycle.launch.py

This launch file runs the nav2 lifecycle manager node which contains map_server, amcl, planner_server, controller_server, recoveries_server, bt_navigator. 

* nav2_map_server is reading the map which was created before.
* amcl node is for robot localization.
* Planner server is responsible for creating a path from Point A to Point B. It computes the path while avoiding known obstacles which are on the map.
* Controller server is resposible for creating reactive path in a certain range. It computes the path while avoiding dynamic obstacles.
* Recoveries server will be used when the robot is stuck.


All configuruations are stored in config folder. 

**![](https://user-images.githubusercontent.com/60695165/185794376-1affcbe8-4484-49cd-8a2b-734ccdbbd463.png)**

# Navigate to pose 

ros2 run two_wheeled_robot navigate_to_pose 

This node will be subscribed of the topic called "/clicked_point" and gets the user input  targer position of the robot. Then, it will the target position to action client NavigateToPose type. 

And robot moves to the target point.. :)

**![](https://user-images.githubusercontent.com/60695165/185794391-0faff145-646c-41be-bf3e-d83475699350.png)**
