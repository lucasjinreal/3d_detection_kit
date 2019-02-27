"""
Utils functions for visualization 3D, include:

1. project 3D box on image;
2. drawing 3D box on point cloud;
"""
import numpy as np
import cv2
import mayavi.mlab as mlab


# ------------------------------------ Drawing 2D --------------------------------------
def draw_projected_box3d(image, qs, color=(0, 255, 0), thickness=2):
    ''' Draw 3d bounding box in image
        qs: (8,3) array of vertices for the 3d box in following order:
            1 -------- 0
           /|         /|
          2 -------- 3 .
          | |        | |
          . 5 -------- 4
          |/         |/
          6 -------- 7
    '''
    qs = qs.astype(np.int32)
    for k in range(0, 4):
       # Ref: http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
        i, j = k, (k+1) % 4
        # use LINE_AA for opencv3
        # cv2.line(image, (qs[i,0],qs[i,1]), (qs[j,0],qs[j,1]), color, thickness, cv2.CV_AA)
        cv2.line(image, (qs[i, 0], qs[i, 1]),
                 (qs[j, 0], qs[j, 1]), color, thickness)
        i, j = k+4, (k+1) % 4 + 4
        cv2.line(image, (qs[i, 0], qs[i, 1]),
                 (qs[j, 0], qs[j, 1]), color, thickness)
        i, j = k, k+4
        cv2.line(image, (qs[i, 0], qs[i, 1]),
                 (qs[j, 0], qs[j, 1]), color, thickness)
    return image


# ----------------------------------- Drawing 3D -----------------------------------------
def draw_3d_box_on_lidar_pc(pc, boxes_3d, fig=None, pc_color=None, box_color=(1, 1, 1), line_width=1, draw_text=True, text_scale=(1, 1, 1), color_list=None):
    """
    Draw 3d box on lidar point cloud
    """
    if fig is None:
        fig = mlab.figure(figure=None, bgcolor=(0,0,0), fgcolor=None, engine=None, size=(1280, 960))
    fig = draw_lidar_simple(pc, fig=fig, color=pc_color)
    draw_gt_boxes3d(boxes_3d, fig=fig)
    # mlab.show(1)
    return fig


def draw_lidar_simple(pc, fig=None, color=None):
    if color is None:
        color = pc[:, 2]
    # draw points
    mlab.points3d(pc[:, 0], pc[:, 1], pc[:, 2], mode="point", color=color, figure=fig)
    return fig


def draw_gt_boxes3d(gt_boxes3d, fig, color=(1, 1, 1), line_width=1, draw_text=True, text_scale=(1, 1, 1), color_list=None):
    ''' Draw 3D bounding boxes
    Args:
        gt_boxes3d: numpy array (n,8,3) for XYZs of the box corners
        fig: mayavi figure handler
        color: RGB value tuple in range (0,1), box line color
        line_width: box line width
        draw_text: boolean, if true, write box indices beside boxes
        text_scale: three number tuple
        color_list: a list of RGB tuple, if not None, overwrite color.
    Returns:
        fig: updated fig
    '''
    num = len(gt_boxes3d)
    for n in range(num):
        b = gt_boxes3d[n]
        if color_list is not None:
            color = color_list[n]
        if draw_text:
            mlab.text3d(b[4, 0], b[4, 1], b[4, 2], '%d' %
                        n, scale=text_scale, color=color, figure=fig)
        for k in range(0, 4):
            # http://docs.enthought.com/mayavi/mayavi/auto/mlab_helper_functions.html
            i, j = k, (k+1) % 4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [
                        b[i, 2], b[j, 2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i, j = k+4, (k+1) % 4 + 4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [
                        b[i, 2], b[j, 2]], color=color, tube_radius=None, line_width=line_width, figure=fig)

            i, j = k, k+4
            mlab.plot3d([b[i, 0], b[j, 0]], [b[i, 1], b[j, 1]], [
                        b[i, 2], b[j, 2]], color=color, tube_radius=None, line_width=line_width, figure=fig)
    mlab.view(azimuth=180, elevation=70, focalpoint=[ 12.0909996 , -1.04700089, -2.03249991], distance=92.0, figure=fig)
    return fig
