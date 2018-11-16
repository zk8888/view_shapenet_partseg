//
// Created by zk8888 on 18-11-13.
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
    string pts_path = "/home/zk8888/Documents/data/shapenet_partseg/train_data/02691156/100001.pts";
    string label_path = "/home/zk8888/Documents/data/shapenet_partseg/train_label/02691156/100001.seg";


    ifstream file(pts_path,ios::in);
    while(!file.eof())
    {

    }


}
