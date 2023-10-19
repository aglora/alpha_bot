# ALPHA-BOT PROJECT
![Diff_car_image](https://github.com/ResitanceBot/beta-bot/blob/doc/imgs/1.png)

## Authors
[RoboticsLeon](https://github.com/RoboticsLeon) //
[arubedaq](https://github.com/arubedaq) //
[aglora](https://github.com/aglora) 

## WORKSPACE DESCRIPTION
In this ROS workspace of our project, we can find in the folder /src/alpha_bot the following developed packages:
- ***/alpha_bot_control**: Here is our high-level driver variant of the pure-pursuit algorithm.
- ***/alpha_bot_description**: Contains the robot model in xacro.
- ***/alpha_bot_localization**: It allows to choose between ground_truth, odometry with encoders, laser odometry, integration of all of them by EKF.
- ***/alpha_bot_odometry**: It contains its own odometry per wheel rotation.
- ***/alpha_bot_planner**: It contains Dijkstra's planner, as well as launching move_base for costmap.
- ***/alpha_bot_simulation**: It is responsible for launching the simulation in Gazebo, the visual representation of data in RVIZ, all the packages necessary for the full operation of the system, and rqt_steering to allow varying high-level controller parameters.

 <img src="https://github.com/aglora/alpha_bot/blob/main/imgs/demo.gif" width="800" />
 
If you want to launch the program, you can use the script ScriptEjecucion.sh with the command:
- ./ScriptEjecucion.sh

If you want to launch by yourself, you can use sim.launch, as it will call everything you need. To do this:
- source devel/setup.sh
- roslaunch alpha_bot_simulation sim.launch
