//
// Created by zk8888 on 18-11-13.
//

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>
#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <vector>

using namespace std;

struct Single
{
    Eigen::Matrix3f matrix3f;
    Eigen::Vector3f vector3f;
    vector<int> obj_bb;
    int obj_id;
};

void operator >> (const YAML::Node& node, Eigen::Matrix3f& matrix3f)
{
    for(int i = 0; i < 9; ++i)
    {
        float tmp;
        node[i] >> tmp;
        matrix3f(i/3,i%3) = tmp;
    }
}

// the extraction operators for types
void operator >> (const YAML::Node& node, Eigen::Vector3f& vector3f)
{
    for(int i = 0; i < 3; ++i)
    {
        float tmp;
        node[i] >> tmp;
        vector3f(i) = tmp;
    }
}

void operator >> (const YAML::Node& node, Single& single)
{
    node["cam_R_m2c"] >> single.matrix3f;
    node["cam_t_m2c"] >> single.vector3f;
    node["obj_bb"] >> single.obj_bb;
    node["obj_id"] >> single.obj_id;
}

int main()
{
    cv::Mat srcImg = cv::imread("0026.png");
//    cout << srcImg << endl;
//    imshow("view", srcImg);
//    waitKey();
    ifstream fin("gt_02.yml");
    YAML::Parser parser(fin);
    YAML::Node doc;
    parser.GetNextDocument(doc);


    for(int i = 0; i < doc['0'].size()-1; ++i)
    {
        const YAML::Node& Index = doc['0'];
        Single single;
        Index[i] >> single;
        cout << single.matrix3f << endl;
        cout << single.vector3f << endl;
//    cout << single.obj_bb << endl;
        cout << single.obj_id << endl;
    }


    return 0;
}