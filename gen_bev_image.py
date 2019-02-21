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
# max x, max y, max z
print('max x: {}, min x: {}'.format(np.max(points[0,:]), np.min(points[0,:])))
print('max y: {}, min y: {}'.format(np.max(points[1,:]), np.min(points[1,:])))
print('max z: {}, min z: {}'.format(np.max(points[2,:]), np.min(points[2,:])))

side_range = (-10, 10)
fwd_range = (0, 20)

x = points[:, 0]
y = points[:, 1]
z = points[:, 2]

# filter point cloud
f_filt = np.logical_and((x>fwd_range[0]), (x<fwd_range[1]))
s_filt = np.logical_and((y>-side_range[1]), (y<-side_range[0]))
filt = np.logical_and(f_filt, s_filt)
indices = np.argwhere(filt).flatten()
x = x[indices]
y = y[indices]
z = z[indices]

# now mapping the real world point to image, expand 200x
res = 0.05
# convert coordinates to 
x_img = (-y/res).astype(np.int32)
y_img = (-x/res).astype(np.int32)
# shifting image, make min pixel is 0,0
x_img -= int(np.floor(side_range[0]/res))
y_img += int(np.ceil(fwd_range[1]/res))

print(x_img.min(), x_img.max())
print(y_img.min(), y_img.max())

# crop y to make it not bigger than 255
height_range = (-2, 0.5)
pixel_values = np.clip(a=z, a_min=height_range[0], a_max=height_range[1])
def scale_to_255(a, min, max, dtype=np.uint8):
    return (((a - min) / float(max - min)) * 255).astype(dtype)
pixel_values = scale_to_255(pixel_values, min=height_range[0], max=height_range[1])

# according to width and height generate image
w = 1+int((side_range[1] - side_range[0])/res)
h = 1+int((fwd_range[1] - fwd_range[0])/res)
im = np.zeros([h, w], dtype=np.uint8)
print(pixel_values)
print(pixel_values.shape)
print('final image size is: ', im.shape)
print(x.shape)
print((y.shape))

im[y_img, x_img] = pixel_values

cv2.imshow('rr', im)
cv2.waitKey(0)

mlab.points3d(x, y, z, z,  mode="point", colormap='spectral')
mlab.show()