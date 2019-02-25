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
#include <iostream>
#include <string>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

using namespace std;

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

void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                          pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered)
{
    pcl::PointIndicesPtr ground(new pcl::PointIndices);

    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZI> pmf;
    pmf.setInputCloud(cloud);
    pmf.setMaxWindowSize(20);
    pmf.setSlope(1.0f);
    pmf.setInitialDistance(0.5f);
    pmf.setMaxDistance(3.0f);
    pmf.extract(ground->indices);

    cout << "pmf done.\n";
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(ground);
    extract.filter(*cloud_filtered);
    cout << "extract done.\n";
}

int main(int argc, char **argv)
{
    string bin_f = argv[1];
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    readPointCloudFromKitti(bin_f, cloud);

    // process on cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    // detectObjectsOnCloud(cloud, cloud_filtered);
    cout << "detect done.\n";

    // now we can show this cloud
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    // viewer.showCloud(cloud_filtered);

    while (!viewer.wasStopped())
    {
    }
    return 0;
}
