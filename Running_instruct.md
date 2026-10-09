to run the sim
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

To run motion script:
    python3 motions.py --motion line
    python3 motions.py --motion circle
    python3 motions.py --motion spiral

To run visualizer:
    python3 filePlotter.py --files odom_content_spiral.csv