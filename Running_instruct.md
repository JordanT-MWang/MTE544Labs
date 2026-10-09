source /opt/ros/humble/setup.bash

to run the sim
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

To run motion script:
    python3 motions.py --motion line
    python3 motions.py --motion circle
    python3 motions.py --motion spiral

To run visualizer:
    python3 filePlotter.py --files odom_content_spiral.csv

Then the laser one:
    python3 laser_plotter.py --file laser_content_circle.csv

Teleop
Unlike on the real robot, in simulation the robot is not docked, so you do not need to dock and undock it. So now you can test anything you want, for example the teleop. You can use the default teleop to move the robot manually:

ros2 run turtlebot3_teleop teleop_keyboard

or this is the same command as on the real robot, which works also in simulation:

ros2 run teleop_twist_keyboard teleop_twist_keyboard

See the prompt for help on the keys.

Exiting the simulation
To close the simulator, close the Gazebo windows, then CTRL+C in the terminal from which you ran command to launch Gazebo (or directly do CTRL+C).

To run slam:
    ros2 launch slam_toolbox online_sync_launch.py
if not visualizer: 
    unset GTK_PATH
    ros2 launch turtlebot3_bringup rviz2.launch.py

to save the map:
    ros2 run nav2_map_server map_saver_cli -f map