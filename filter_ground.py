"""
this file shows
how to filter ground points in point cloud

"""
import numpy as np
import cv2
import os
import mayavi.mlab as mlab

res = 0.05
# image size would be 400x800
side_range = (-20, 20)
fwd_range = (-20, 20)


def load_pc(f):
    b = np.fromfile(f, dtype=np.float32)
    return b.reshape((-1, 4))[:, :3]

a = 'data/testing/valodyne/000003.bin'
points = load_pc(a)