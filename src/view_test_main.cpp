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


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZRGBNormal> Cloud6DN;
typedef pcl::PointCloud<pcl::PointXYZRGB> Cloud6D;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud3D;
typedef pcl::PointCloud<pcl::PointXYZINormal> Cloud3DN;



int main()
{
    string modelBasePath = "/home/zk8888/Documents/sixd_toolkit-master/datasets/hinterstoisser/models/";
    /// load model

    vector<Cloud6D::Ptr, Eigen::aligned_allocator<Cloud6D::Ptr> > sourceClouds;
    string model_idx[9] = { "01", "02", "05", "06", "08", "09", "10", "11", "12"};
    for(int i = 0; i < 9; ++i)
    {
        Cloud6D::Ptr sourceCloud(new Cloud6D());
        string model_path = modelBasePath + "obj_" + model_idx[i] + ".ply";
        if(pcl::io::loadPLYFile(model_path,*sourceCloud)!=0)
        {
            return -1;
        }
        sourceClouds.push_back(sourceCloud);
//        sourceCloud.clear();
    }

    for(int i = 0; i < 9; ++i)
    {
        // view
        boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("view"));
        view->setBackgroundColor(1.0,1,1);
//                view->addPointCloud(new_cloud);
        view->addPointCloud(sourceClouds[i]);
        while(!view->wasStopped())
        {
            view->spinOnce();
        }
        view->close();

    }

    return 0;
}