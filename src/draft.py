import bosdyn.client.local_grid
import bosdyn.client
import io
from PIL import Image
import numpy as np
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

