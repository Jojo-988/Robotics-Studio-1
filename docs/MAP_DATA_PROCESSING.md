# Map Data Processing

Initial prototype: subscribes to a SLAM OccupancyGrid and logs its
frame, dimensions, resolution and origin.

Run with a simulator and SLAM already publishing /husky1/map:

    ros2 run 41068_ignition_bringup map_processor.py --ros-args -p use_sim_time:=true

The map topic can be changed with the map_topic parameter.
Coverage tracking and perception integration are not implemented yet.


## Validation

- Tested with Husky simulation and SLAM Toolbox.
- Subscribed to /husky1/map.
- Received repeated OccupancyGrid messages.
- Observed frame: husky1_map.
- Observed dimensions: 109 × 57 cells.
- Observed resolution: 0.05 m/cell.
- Coverage tracking and perception integration are not implemented yet.