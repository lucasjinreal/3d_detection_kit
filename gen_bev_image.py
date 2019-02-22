"""
to generate bird eye view, we have to filter point cloud
first. which means we have to limit coordinates


"""
import numpy as np
import cv2
import os
import mayavi.mlab as mlab



def load_pc(f):
    b = np.fromfile(f, dtype=np.float32)
    return b.reshape((-1, 4))[:, :3]

a = 'data/testing/valodyne/000003.bin'
points = load_pc(a)


def gen_bev_map(pc, lr_range=[-10, 10], bf_range=[-20, 20], res=0.05):
    x = pc[:, 0]
    y = pc[:, 1]
    z = pc[:, 2]

    # filter point cloud
    f_filt = np.logical_and((x>bf_range[0]), (x<bf_range[1]))
    s_filt = np.logical_and((y>-lr_range[1]), (y<-lr_range[0]))
    filt = np.logical_and(f_filt, s_filt)
    indices = np.argwhere(filt).flatten()
    x = x[indices]
    y = y[indices]
    z = z[indices]

    # convert coordinates to 
    x_img = (-y/res).astype(np.int32)
    y_img = (-x/res).astype(np.int32)
    # shifting image, make min pixel is 0,0
    x_img -= int(np.floor(lr_range[0]/res))
    y_img += int(np.ceil(bf_range[1]/res))

    # crop y to make it not bigger than 255
    height_range = (-2, 0.5)
    pixel_values = np.clip(a=z, a_min=height_range[0], a_max=height_range[1])
    def scale_to_255(a, min, max, dtype=np.uint8):
        return (((a - min) / float(max - min)) * 255).astype(dtype)
    pixel_values = scale_to_255(pixel_values, min=height_range[0], max=height_range[1])

    # according to width and height generate image
    w = 1+int((lr_range[1] - lr_range[0])/res)
    h = 1+int((bf_range[1] - bf_range[0])/res)
    im = np.zeros([h, w], dtype=np.uint8)
    im[y_img, x_img] = pixel_values

im = gen_bev_map(points)
cv2.imshow('rr', im)
cv2.waitKey(0)

mlab.points3d(x, y, z, z,  mode="point", colormap='spectral')
mlab.show()