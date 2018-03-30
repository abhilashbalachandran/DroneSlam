# Drone SLAM

Please refer to the RSPprojectreport for more info about the contents of this repo.

# For running in gazebo 
(please install dependencies as provided in the file RSPprojectreport)
```
roslaunch map_building_gazebo2 ardrone_map_building.launch
roslaunch map_building_gazebo2 create_occupancy_grid.launch
```

This will create an occupancy grid. Note that the launch file fake_localization.launch was used for a robot called EduMip whose files are not opensource. You still get the occupancy grid, which can be used for planning using move_base.

# For running in ar_drone
```
roscore
rosrun ardrone_autonomy ardrone_driver
roslaunch map_building ardrone_map_building.launch
roslaunch map_building create_occupancy_grid.launch
```

This will create an occupancy grid. Note that the launch file fake_localization.launch was used for a robot called EduMip whose files are not opensource. You still get the occupancy grid, which can be used for planning using move_base.

