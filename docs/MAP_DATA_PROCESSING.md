# Map Data Processing

Initial prototype: subscribes to a SLAM OccupancyGrid and logs its
frame, dimensions, resolution and origin.

Run with a simulator and SLAM already publishing /husky1/map:

    ros2 run 41068_ignition_bringup map_processor.py --ros-args -p use_sim_time:=true

The map topic can be changed with the map_topic parameter.
Coverage tracking and perception integration are not implemented yet.