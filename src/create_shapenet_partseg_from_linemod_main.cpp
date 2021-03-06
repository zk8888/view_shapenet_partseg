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

const int WIDTH=640;
const int HEIGHT=480;
const double fx = 572.4114;
const double fy = 573.57043;
const double px = 325.2611;
const double py = 242.04899;

int TotalIndex = 10504;

int getAbsoluteFiles(string directory, vector<string>& filesAbsolutePath);
cv::Mat loadDepth(string a_name);

int main()
{
    string trainDataBasePath = "/home/zk8888/Documents/hinterstoisser_seg/train_data/";
    string trainLabelBasePath = "/home/zk8888/Documents/hinterstoisser_seg/train_label/";
    string renderedDataBasePath = "/home/zk8888/Documents/sixd_toolkit-master/datasets/hinterstoisser/train/";
    vector<string> ss;
    ss.push_back("01");
    ss.push_back("02");
    ss.push_back("03");
    ss.push_back("04");
    ss.push_back("05");
    ss.push_back("06");
    ss.push_back("07");
    ss.push_back("08");
    ss.push_back("09");
    ss.push_back("10");
    ss.push_back("11");
    ss.push_back("12");
    ss.push_back("13");
    ss.push_back("14");

    for(int ss_idx = 8; ss_idx < ss.size(); ss_idx++)
    {
        string objpath = renderedDataBasePath + ss[ss_idx] + "/";

        string obj_rgb_path = objpath + "rgb/";
        string obj_dpt_path = objpath + "depth/";
        vector<string> filesPath;
        getAbsoluteFiles(obj_rgb_path, filesPath);
        sort(filesPath.begin(),filesPath.end());
        vector<string> filesPath2;
        getAbsoluteFiles(obj_dpt_path, filesPath2);
        sort(filesPath2.begin(),filesPath2.end());
        vector<string> rgbPath;
        vector<string> dptPath;
        for(int file_idx = 0; file_idx < filesPath.size(); ++file_idx)
        {
            if(filesPath[file_idx].find("png")!=string::npos)
                rgbPath.push_back(filesPath[file_idx]);
            if(filesPath2[file_idx].find("png")!=string::npos)
                dptPath.push_back(filesPath2[file_idx]);
        }
        std::sort(rgbPath.begin(),rgbPath.end(),[](string a, string b){ return a < b;});
//        cout << "rgbPath: " << endl;
//        cout << rgbPath[0] << endl;
        std::sort(dptPath.begin(),dptPath.end(),[](string a, string b){ return a < b;});
        string train_data_path = trainDataBasePath + ss[ss_idx];
        if(train_data_path.size()>0&&access(train_data_path.c_str(),0)!=0)
            mkdir(train_data_path.c_str(),00700);
        string train_label_path = trainLabelBasePath + ss[ss_idx];
        if(train_label_path.size()>0&&access(train_label_path.c_str(),0)!=0)
            mkdir(train_label_path.c_str(),00700);

#pragma omp parallel for
        for(int rgb_index = 0; rgb_index < rgbPath.size(); rgb_index++)
        {
            stringstream sss;
            sss << TotalIndex;
            string rgb_index_n_s = "000000" + sss.str();
            string save_train_data_path = train_data_path + "/" + rgb_index_n_s.substr(
                    rgb_index_n_s.size()-6, rgb_index_n_s.size()) + ".pts";
            string save_train_label_path = train_label_path + "/" + rgb_index_n_s.substr(
                    rgb_index_n_s.size()-6, rgb_index_n_s.size()) + ".seg";
            ofstream file_data(save_train_data_path, ios::out | ios::trunc);
            ofstream file_label(save_train_label_path, ios::out | ios::trunc);
            Cloud6DN::Ptr new_cloud(new Cloud6DN());
            new_cloud->width = WIDTH;
            new_cloud->height = HEIGHT;
            new_cloud->resize(WIDTH * HEIGHT);
            new_cloud->is_dense = false;
            cv::Mat depthImg = cv::imread(dptPath[rgb_index],CV_LOAD_IMAGE_ANYDEPTH);
//            cout << depthImg << endl;
            cv::Mat rgbImg = cv::imread(rgbPath[rgb_index]);
            for(int i = 0; i < HEIGHT; ++i)
            {
                uchar *datargb = rgbImg.ptr<uchar>(i);
                unsigned short *datadpt = depthImg.ptr<unsigned short>(i);
                for(int j = 0; j < WIDTH; ++j)
                {
//                    cout << datadpt[j] << endl;
//                    pcl::PointXYZRGBNormal *p = &new_cloud->points[i*WIDTH+j];
//                    p->z = datadpt[j]*1.0;
//                    p->x = (j-px)*1.0/fx*p->z;
//                    p->y = (i-py)*1.0/fy*p->z;

//                    p->b = datargb[j*3+0];
//                    p->g = datargb[j*3+1];
//                    p->r = datargb[j*3+2];
                    float z = datadpt[j]*1.0;
                    float x = (j-px)*1.0/fx*z;
                    float y = (i-py)*1.0/fy*z;
                    int r = datargb[j*3+0];
                    int g = datargb[j*3+1];
                    int b = datargb[j*3+2];
                    file_data << x << " " << y << " " << z << endl;
                    if(r || g || b)
                    {
                        file_label << ss_idx + 1 << endl;
                    }else
                    {
                        file_label << 0 << endl;
                    }
//                    cout << "px,py,pz:" << p->x << "," << p->y << "," << p->z << endl;
//                    cout << "pb,pg,pr:" << p->b << "," << p->g << "," << p->r << endl;
                }
            }
            file_label.close();
            file_data.close();
            cout << ss[ss_idx] << ": " << rgb_index_n_s.substr(
                    rgb_index_n_s.size()-6, rgb_index_n_s.size()) << ".pts" << endl;
            TotalIndex++;
        }

    }

    return 0;
}

int getAbsoluteFiles(string directory, vector<string>& filesAbsolutePath) //参数1[in]要变量的目录  参数2[out]存储文件名
{
    DIR* dir = opendir(directory.c_str()); //打开目录   DIR-->类似目录句柄的东西
    if ( dir == NULL )
    {
        cout<<directory<<" is not a directory or not exist!"<<endl;
        return -1;
    }

    struct dirent* d_ent = NULL;       //dirent-->会存储文件的各种属性
    char fullpath[128] = {0};
    char dot[3] = ".";                //linux每个下面都有一个 .  和 ..  要把这两个都去掉
    char dotdot[6] = "..";

    while ( (d_ent = readdir(dir)) != NULL )    //一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
    {
        if ( (strcmp(d_ent->d_name, dot) != 0)
             && (strcmp(d_ent->d_name, dotdot) != 0) )   //忽略 . 和 ..
        {
            if ( d_ent->d_type == DT_DIR ) //d_type可以看到当前的东西的类型,DT_DIR代表当前都到的是目录,在usr/include/dirent.h中定义的
            {

                string newDirectory = directory + string("/") + string(d_ent->d_name); //d_name中存储了子目录的名字
                if( directory[directory.length()-1] == '/')
                {
                    newDirectory = directory + string(d_ent->d_name);
                }

                if (getAbsoluteFiles(newDirectory, filesAbsolutePath)==-1)  //递归子目录
                {
                    return -1;
                }
            }
            else   //如果不是目录
            {
                string absolutePath = directory + string("/") + string(d_ent->d_name);  //构建绝对路径
                if( directory[directory.length()-1] == '/')  //如果传入的目录最后是/--> 例如a/b/  那么后面直接链接文件名
                {
                    absolutePath = directory + string(d_ent->d_name); // /a/b/1.txt
                }
                filesAbsolutePath.push_back(absolutePath);
            }
        }
    }

    closedir(dir);
    return 0;
}

cv::Mat loadDepth(string a_name)
{
    std::ifstream l_file(a_name.c_str(),std::ofstream::in|std::ofstream::binary);

    if(l_file.fail() == true)
    {
        printf("cv_load_depth: could not open file for writing!\n");
        cv::Mat none = cv::Mat::zeros(1,1,CV_8UC1);
        return none;
    }
    int l_row;
    int l_col;

    l_file.read((char*)&l_row,sizeof(l_row));
    l_file.read((char*)&l_col,sizeof(l_col));

    cv::Mat image(l_row,l_col,IPL_DEPTH_16U,1);

    for(int l_r = 0; l_r < l_row; ++l_r)
    {
        unsigned short *data = image.ptr<unsigned short>(l_r);
        for(int l_c = 0; l_c < l_col; ++l_c)
        {
            l_file.read((char*)&data[l_c], sizeof(unsigned short));
//            cout << data[l_c] << endl;
        }
    }
    l_file.close();
    cout << image.size().width << endl;
    return image;
}