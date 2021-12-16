!! Moved development to a fork of clearpath driver: https://github.com/silverjoda/spot_ros 


Repo for testing Boston dynamics Spot advanced SDK functions and integrating them into the existing clearpath ROS driver.
I'll soon be pushing a fork of the Clearpath driver which includes extensions to set all mobility parameters such as terrain friction hint, obstacle
avoidance cushion, reading spot local grid maps and more, through ros topics.

Current work in progress:

- local_grid.py publishes spot's local grid to ROS GridCell (requires optimization for faster processing)
- Changing spot's gait parameters (mobility parameters) on the fly through ros topics (will push soon)
