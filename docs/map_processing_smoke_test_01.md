# Map processing minimum prototype verification

Day:2026-10-13
Scene: simple_trees
Configuration startup: slam=true, nav2=false, rviz=false
Rendering mode: LIBGL_ALWAYS_SOFTWARE=1
Input topic: /husky1/map
Map coordinate system: husky1_map
Resolution: 0.05 m/cell

## Verified
- After pressing 'j', the vehicle body rotates in Gazebo.
- During the rotation, the angular velocity of /husky1/odometry is significantly non-zero.
- map_processor.py continuously receives the map and prints statistics.
- The known area of this log has increased from 2.01 m² to 2.53 m².
- The map size has changed from 110×60 to 110×61.

## Verification Scope
This verification confirms that the motion control, map reception, and statistical output functions are operational.
The geometric accuracy of the map, positioning drift, and the coverage of the entire survey area have not been verified.
The "known %" indicates the known grid ratio within the current map range.