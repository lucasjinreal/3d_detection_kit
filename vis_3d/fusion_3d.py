
"""
converting between lidar points and image


"""
import numpy as np



class LidarCamCalibration(object):
    """
    3x4    p0-p3      Camera P matrix. Contains extrinsic
                          and intrinsic parameters.

    3x3    r0_rect    Rectification matrix, required to transform points
                      from velodyne to camera coordinate frame.

    3x4    tr_velodyne_to_cam    Used to transform from velodyne to cam
                                 coordinate frame according to:
                                 Point_Camera = P_cam * R0_rect *
                                                Tr_velo_to_cam *
                                                Point_Velodyne.
    """
    def __init__(self):
        self.p0 = []
        self.p1 = []
        self.p2 = []
        self.p3 = []
        self.r0_rect = []
        self.tr_velodyne_to_cam = []

    def __str__(self):
        return 'p0: {}\np1: {}\np2: {}\np3: {}\nr0_rect: {}\ntr_veodyne_to_cam: {}\n'.format(
            self.p0, self.p1, self.p2, self.p3, self.r0_rect, self.tr_velodyne_to_cam
        )
    
    def load_from_file(self, f):
        """
        load from file with Kitti format calibration
        """
        pass


# --------------- Convert 3D points to image --------------------