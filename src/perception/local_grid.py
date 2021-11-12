import rospy
import bosdyn.client.local_grid
import bosdyn.client
import numpy as np
import yaml

import tf2_ros
from nav_msgs.msg import GridCells

class LocalGridPublisher:
    def __init__(self, config):
        self.config = config
        self.init_spot_com()
        self.init_ros()

    def init_spot_com(self):
        self.sdk = bosdyn.client.create_standard_sdk('local-grid-publisher')
        self.robot = sdk.create_robot('192.168.50.3')
        self.robot.authenticate('robot', 'niftiniftinifti')
        self.grid_client = robot.ensure_client(bosdyn.client.local_grid.LocalGridClient.default_service_name)

    def init_ros(self):
        self.rate = rospy.Rate(self.config["ros_rate"])
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.gc_terrain_publisher = rospy.Publisher("spot/grid/terrain", GridCells, queue_size=1)
        self.gc_intensity_publisher = rospy.Publisher("spot/grid/intensity", GridCells, queue_size=1)
        self.gc_valid_publisher = rospy.Publisher("spot/grid/valid", GridCells, queue_size=1)

    def get_local_grids(self, names):
        # Get all grids in protobuff format
        grids = self.grid_client.get_local_grids(names)

        for name, grid in zip(names, grids):
            # Get all available fields info out of the protobuff msg
            local_grid_type_name = grid.local_grid.local_grid_type_name
            acquisition_time = grid.local_grid.acquisition_time
            transforms_snapshot = grid.local_grid.transforms_snapshot
            frame_name_local_grid_data = grid.local_grid.frame_name_local_grid_data
            extent = grid.local_grid.extent
            cell_format = grid.local_grid.cell_format
            encoding = grid.local_grid.encoding
            data = grid.local_grid.data
            rle_counts = grid.local_grid.rle_counts
            cell_value_scale = grid.local_grid.cell_value_scale
            cell_value_offset = grid.local_grid.cell_value_offset

            # Check that we have the correct name
            assert local_grid_type_name == name

            # Make the decoded grid as an array









    def loop(self):
        while not rospy.is_shutdown():

            # Get local grid and transforms

            # Process and make into ROS PC

            # Publish pc as ros msg

            self.rate.sleep()


sdk = bosdyn.client.create_standard_sdk('understanding-spot')
robot = sdk.create_robot('192.168.50.3')
robot.authenticate('robot', 'niftiniftinifti')
grid_client = robot.ensure_client(bosdyn.client.local_grid.LocalGridClient.default_service_name)
#print(grid_client.get_local_grid_types())
local_grid = grid_client.get_local_grids(["terrain"])
data = local_grid[0].local_grid.data
#print(dir(local_grid[0].local_grid))
#print(local_grid[0].local_grid.transforms_snapshot)
print((local_grid[0].local_grid.transforms_snapshot.child_to_parent_edge_map["vision"].parent_tform_child))
print(local_grid[0].local_grid.extent)
#print(local_grid[0].local_grid.rle_counts)
print(len(data))
#arr = np.array([d for d in data]).reshape(128,128)
#print(arr.shape)


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    with open("configs/local_grid_config.yaml") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    local_grid_publisher = LocalGridPublisher(config)
    local_grid_publisher.loop()
