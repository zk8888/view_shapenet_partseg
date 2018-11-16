//
// Created by zk8888 on 18-11-15.
//

#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <pcl/common/common.h>
#include <omp.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <string>
#include <iostream>

// yaml
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud6DN;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud6D;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud3D;
typedef pcl::PointCloud<pcl::PointXYZINormal> Cloud3DN;

const int WIDTH=640;
const int HEIGHT=480;
const double fx = 572.4114;
const double fy = 573.57043;
const double px = 325.2611;
const double py = 242.04899;

int main()
{
//    string modelBasePath = "/home/zk8888/Documents/sixd_toolkit-master/datasets/hinterstoisser/models/";
    string testData02Path = "/home/zk8888/Documents/hinterstoisser_seg/test_data/02/100000.pts";
    string testLabel02Path = "/home/zk8888/Documents/hinterstoisser_seg/test_label/02/100000.seg";
//    string renderedDataBasePath = "/home/zk8888/Documents/sixd_toolkit-master/datasets/hinterstoisser/test/";

    // hash
    int hash[13] = {0};
    hash[1] = 0;
    hash[2] = 1;
    hash[5] = 2;
    hash[6] = 3;
    hash[8] = 4;
    hash[9] = 5;
    hash[10] = 6;
    hash[11] = 7;
    hash[12] = 8;
    hash[0] = 9;

    uchar color[][3] = {0,0,255, // Blue
                    0,191,255,  // DeepSkyBlue
                    0,255,0,    // Green
                    0,100,0,    // DarkGreen
                    139,69,19,  // SaddleBrown
                    178,34,34,  // Firebrick
                    255,165,0,  // Orange
                    255,0,255,  // Magenta
                    255,20,147,  // DarkPink
                    0,0,0};     // Black
    ifstream fileData(testData02Path);
    ifstream fileLabel(testLabel02Path);

    Cloud6D::Ptr new_cloud(new Cloud6D());
    new_cloud->is_dense = false;
    new_cloud->width = WIDTH;
    new_cloud->height = HEIGHT;
    new_cloud->resize(WIDTH*HEIGHT);
    for(size_t i = 0; i < new_cloud->points.size(); ++i)
    {
        pcl::PointXYZRGB p;
        fileData >> p.x;
        fileData >> p.y;
        fileData >> p.z;
        int segIdx;
        fileLabel >> segIdx;
        p.r = color[hash[segIdx]][0];
        p.g = color[hash[segIdx]][1];
        p.b = color[hash[segIdx]][2];
        new_cloud->points[i] = p;
    }

    fileLabel.close();
    fileData.close();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("view"));
    view->setBackgroundColor(1.0,1,1);

    view->addPointCloud(new_cloud);
    while(!view->wasStopped())
    {
        view->spinOnce();
    }

    return 0;
}