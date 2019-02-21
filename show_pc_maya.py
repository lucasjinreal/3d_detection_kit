import os
import numpy as np
from open3d import *
import cv2

# a = 'training/velodyne/000900.bin'
# a = '/media/jintain/sg/permanent/datasets/KITTI/kitti_object_vis/data/object/training/velodyne/000000.bin'
a = 'data/testing/valodyne/000003.bin'

b = np.fromfile(a, dtype=np.float32)
b = np.reshape(b, (-1, 4))
print(b)
b = b[:, :3]

# show point cloud using maya