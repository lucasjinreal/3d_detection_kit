/**
 *
 * this file will do
 * ground filter, object cluster,
 * 3d box regression on lidar point
 * not using deep learning models
 * */

#include "pcl/pcl_base.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/visualization/cloud_viewer.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <iostream>
#include <string>
#include "Eigen/Core"
#include "Eigen/Dense"
#include "glog/logging.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;
using namespace Eigen;
using namespace google;
typedef pcl::PointXYZI PointType;

void readPointCloudFromKitti(string bin_file, pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
{
    int32_t num = 1000000;
    float *data = (float *)malloc(num * sizeof(float));

    // pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pr = data + 3;

    // load point cloud
    FILE *stream;
    stream = fopen(bin_file.c_str(), "rb");
    num = fread(data, sizeof(float), num, stream) / 4;
    for (int32_t i = 0; i < num; i++)
    {
        pcl::PointXYZI point;
        point.x = *px;
        point.y = *py;
        point.z = *pz;
        point.intensity = *pr;
        point_cloud->points.push_back(point);
        px += 4;
        py += 4;
        pz += 4;
        pr += 4;
    }
}

void calibPointCloud(pcl::PointCloud<PointType>::Ptr &cloud)
{
    pcl::SACSegmentation<PointType> plane_seg;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);

    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.3);
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*plane_inliers, *plane_coefficients);
}

Eigen::Matrix4f createRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{
    before.normalize();
    after.normalize();

    float angle = acos(before.dot(after));
    Vector3f p_rotate = before.cross(after);
    p_rotate.normalize();

    Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
    rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
    rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle)); //这里跟公式比多了一个括号，但是看实验结果它是对的。
    rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
    rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

    rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
    rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));
    return rotationMatrix;
}

void detectObjectsOnCloud(pcl::PointCloud<PointType>::Ptr &cloud,
                          pcl::PointCloud<PointType>::Ptr &cloud_filtered)
{
    if (cloud->size() > 0)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<PointType> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            LOG(ERROR) << "Could not found any inliers.";
        }

        // extract ground
        pcl::ExtractIndices<PointType> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
          extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "filter done.";
    }
    else
    {
        LOG(INFO) << "cloud has no data.";
    }
}

int main(int argc, char **argv)
{
    string bin_f = argv[1];
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
    readPointCloudFromKitti(bin_f, cloud);

    // process on cloud
    pcl::PointCloud<PointType>::Ptr cloud_filtered(new pcl::PointCloud<PointType>);
    detectObjectsOnCloud(cloud, cloud_filtered);

    cout << "detect done.\n";

    // now we can show this cloud
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    // viewer.showCloud(cloud);
    viewer.showCloud(cloud_filtered);

    while (!viewer.wasStopped())
    {
    }

    // test eigen
    Vector3f v(6, 6, 3);
    cout << v << endl;
    return 0;
}
