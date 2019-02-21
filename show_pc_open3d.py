import os
import numpy as np
from open3d import *
import cv2

# a = 'training/velodyne/000900.bin'
# a = '/media/jintain/sg/permanent/datasets/KITTI/kitti_object_vis/data/object/training/velodyne/000000.bin'
a = 'data/testing/valodyne/000003.bin'
b = np.fromfile(a, dtype=np.float32)

print(b)
print(b.shape)

points = np.random.rand(10000, 3)
point_cloud = PointCloud()
point_cloud.points = Vector3dVector(points)

def custom_draw_geometry_with_key_callback(pcd):
    def change_background_to_black(vis):
        opt = vis.get_render_option()
        opt.background_color = np.asarray([0, 0, 0])
        return False
    def load_render_option(vis):
        vis.get_render_option().load_from_json(
                "../../TestData/renderoption.json")
        return False
    def capture_depth(vis):
        depth = vis.capture_depth_float_buffer()
        plt.imshow(np.asarray(depth))
        plt.show()
        return False
    def capture_image(vis):
        image = vis.capture_screen_float_buffer()
        cv2.imshow('snap', np.asarray(image))
        # cv2.im
        cv2.waitKey(0)
        return False
    key_to_callback = {}
    key_to_callback[ord("K")] = change_background_to_black
    key_to_callback[ord("R")] = load_render_option
    key_to_callback[ord(",")] = capture_depth
    key_to_callback[ord(".")] = capture_image
    draw_geometries_with_key_callbacks([pcd], key_to_callback)

# custom_draw_geometry_with_key_callback(point_cloud)

b = np.reshape(b, (-1, 4))
print(b)
b = b[:, :3]
print(b.shape)
# we need to know
point_cloud.points = Vector3dVector(b)
custom_draw_geometry_with_key_callback(point_cloud)