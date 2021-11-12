import rospy
import bosdyn.client.local_grid
import bosdyn.client
import numpy as np
import yaml
import struct

import tf2_ros
from nav_msgs.msg import GridCells
from geometry_msgs.msg import Point

class LocalGridPublisher:
    def __init__(self, config):
        self.config = config
        self.init_spot_com()
        self.init_ros()

    def init_spot_com(self):
        self.sdk = bosdyn.client.create_standard_sdk('local-grid-publisher')
        self.robot = self.sdk.create_robot('192.168.50.3')
        self.robot.authenticate('robot', 'niftiniftinifti')
        self.grid_client = self.robot.ensure_client(bosdyn.client.local_grid.LocalGridClient.default_service_name)
        self.grid_names = ["terrain", "terrain_valid", "intensity", "no_step", "obstacle_distance"]
        self.format_map = {0: "@", 1: "<f", 2: "<d", 3: "<b", 4:"<B", 5:"<h", 6: "<H"}

    def init_ros(self):
        self.rate = rospy.Rate(self.config["ros_rate"])
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.gc_publisher_dict = {}
        for grid_name in self.grid_names:
            self.gc_publisher_dict[grid_name] = rospy.Publisher(f"spot/grid/{grid_name}", GridCells, queue_size=1)

    def decode(self, data, cell_format):
        return struct.unpack(self.format_map[cell_format], data)

    def calc_transform_to_body(self, transforms_snapshot, frame_name_local_grid_data):
        current_frame = frame_name_local_grid_data

        pose_list = []
        for i in range(10):
            tf = transforms_snapshot.child_to_parent_edge_map[current_frame]
            pose_list.append(tf.parent_tform_child)
            current_frame = tf.parent_frame_name
            if current_frame == "body":
                break

        # Multiply all transforms and then reverse to get body to grid
        grid_to_body_tf = pose_list[0]
        for p in pose_list[1:]:
            grid_to_body_tf *= p
        #grid_to_body_tf = grid_to_body_tf.reverse()
        return grid_to_body_tf

    def get_local_grids(self, names):
        # Ros grid messages
        grid_msgs = {}

        # Get all grids in protobuff format
        grids = self.grid_client.get_local_grids(names)

        for name, grid in zip(names, grids):
            # Get all available fields info out of the protobuff msg
            local_grid_type_name = grid.local_grid.local_grid_type_name
            acquisition_time = grid.local_grid.acquisition_time
            transforms_snapshot = grid.local_grid.transforms_snapshot
            frame_name_local_grid_data = grid.local_grid.frame_name_local_grid_data
            extent = grid.local_grid.extent # cell_size, num_cells_x, num_cells_y
            cell_format = grid.local_grid.cell_format
            encoding = grid.local_grid.encoding # 0: unspecified, 1: raw, 2: rle
            data = grid.local_grid.data
            rle_counts = grid.local_grid.rle_counts
            cell_value_scale = np.maximum(1, grid.local_grid.cell_value_scale)
            cell_value_offset = grid.local_grid.cell_value_offset

            # Check that we have the correct name
            assert local_grid_type_name == name

            # Calculate transform from grid corner to body frame
            grid_to_body_tf = self.calc_transform_to_body(transforms_snapshot, frame_name_local_grid_data)

            # Decode the data
            points = []
            idx = 0
            for i, d in enumerate(data):
                # Height of cell
                z = self.decode(d, cell_format) * cell_value_scale + cell_value_offset

                # Go over rle counts to decode repetitions
                for _ in range(rle_counts[i]):
                    # x and y values (in grid frame)
                    x = (idx % extent.num_cells_x) * extent.cell_size
                    y = (idx // extent.num_cells_x) * extent.cell_size
                    idx += 1

                    # Apply transform (just translation here)
                    x_tf = grid_to_body_tf.position.x
                    y_tf = grid_to_body_tf.position.y
                    z_tf = grid_to_body_tf.position.z

                    # Add point to points list
                    pt = Point()
                    pt.x = x_tf
                    pt.y = y_tf
                    pt.z = z_tf

                    points.append(pt)

            # Make ros message
            grid_msg = GridCells()
            grid_msg.header.frame_id = "body"
            grid_msg.header.stamp = acquisition_time
            grid_msg.cells = points
            grid_msg.cell_width = extent.cell_size
            grid_msg.cell_height = extent.cell_size

            grid_msgs[name] = grid_msg

        return grid_msgs

    def publish_msgs(self, cell_msg_dict):
        for name, msg in cell_msg_dict.items():
            self.gc_publisher_dict[name].publish(msg)

    def loop(self):
        while not rospy.is_shutdown():

            cell_msg_dict = self.get_local_grids(self.grid_names)
            self.publish_msgs(cell_msg_dict)

            self.rate.sleep()

if __name__ == '__main__':
    with open("configs/local_grid_config.yaml") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
    local_grid_publisher = LocalGridPublisher(config)
    local_grid_publisher.loop()
