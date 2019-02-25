# 3D Detection Kit

This repo contains several useful scripts which using for 3D detection algorithm development. Including:

- Loading and visualization on lidar point cloud;
- Generate bev map(bird view map);
- Showing 3D Bounding box on point cloud;
- Project 3D Bounding Box on image with calib params;
- Showing bbox on bev image angle;



## Usage

directly using:

```
python3 show_pc.py
```

and you can see point cloud. you should also install *open3d* first, using:

```
sudo pip3 install open3d-python
```



## Tutorials

to get more information, you can visit our community forum to talk: http://talk.strangeai.pro

### convert point cloud to bev map

To get an bev map for point cloud. it should construct a image with certain width and height, then every pixel value should be the z, if there is no point from top view, then z should be 0, so that bev image would be black. So to construct a bev map, should crop the point cloud first, then make x,y coordinates min to be 0, which is mapping x, y to image coordinates. After that, mapping z values to 0~255 which will be using for pixel values. There mainly 3 things to make it done:

- Crop the point cloud to certain range, like left-right range, back-forward range;
- Shifting x, y values to min be 0;
- Mapping z to image pixel values.





## Copyright

The codes opensource under MIT license. All right belongs to Jin Fagang