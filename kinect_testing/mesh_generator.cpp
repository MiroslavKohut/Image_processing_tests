//
// Created by mirec on 2017.03.01..
//

//COMMON INCLUDES
#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>

//OPENCV2 INCLUDE DIRS
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

//PCL INCLUDE DIRS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>

#include "logging.h"

#define IMAGES_COUNT 150
#define MINIMUM_DISTANCE_MM 700


using namespace std;
using namespace cv;

typedef struct Target{
    float tx; //metres
    float ty;
    float tz;
    float rx; // radians
    float ry;
    float rz;
    float rw;
} Target;

typedef struct Target_origin{
    Eigen::Quaternionf orientation;
    Eigen::Vector4f origin;
} Target_origin;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

class Meshgen{

public:

    Meshgen();
    ~Meshgen();
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target target);
    void transformOriginPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target_origin target_origin);
    void generatePointCloud(const cv::Mat & rgbMat, const cv::Mat & depthMat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,int id);

private:
    const struct Target target_10 ={700,0,200,0,-0.707106781,0.707106781,0};
    const struct Target target_20 ={1049.999698851,-350.00005995,549.999355294,0.499999609,0.000000171,0.707107214,-0.499999778};
    const struct Target target_30 ={1400,-700,200,0,0,1,0};
    const struct Target target_40 ={700,-1400,200,0,0.707106781,0.707106781,0};
    const struct Target target_50 ={0,-700,200,0,1,0,0};
    const struct Target target_60 ={700,-600,900,0.5,0.5,0.5,-0.5};
    const struct Target target_70 ={1050,-1050,550,0.353553391,0.353553389,0.853553391,-0.14644661};
    const struct Target target_80 ={350,-1050,550,0.14644661,0.853553391,0.353553389,-0.353553391};


    int depth_idx_f;
};


Meshgen::Meshgen(){
    depth_idx_f = 0;

}
void Meshgen::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target target) {

    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();
    // Define a translation of 2.5 meters on the x axis.
    transform_x.rotate (Eigen::AngleAxisf (target.rx, Eigen::Vector3f::UnitX()));
    transform_y.rotate (Eigen::AngleAxisf (target.ry, Eigen::Vector3f::UnitY()));

    transform_z.translation() << target.tx, target.ty, target.tz;
    transform_z.rotate (Eigen::AngleAxisf (target.rz, Eigen::Vector3f::UnitZ()));


    //std::cout << transform_x.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_x);


    //std::cout << transform_y.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_y);

    // You can either apply transform_1 or transform_2; they are the same
    //std::cout << transform_z.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_z);
}

void Meshgen::transformOriginPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target_origin target_origin) {

    //TODO Implement trasnformation of pointcloud sensor origin;
    pointcloud->sensor_orientation_ = target_origin.orientation;
    pointcloud->sensor_origin_ = target_origin.origin;


}


void Meshgen::generatePointCloud(const cv::Mat & rgbMat, const cv::Mat & depthMat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,int id) {
    pointcloud->width = rgbMat.cols;
    pointcloud->height = rgbMat.rows;
    pointcloud->points.resize (pointcloud->height * pointcloud->width);

    float  focalLength = 525;
    float centerX = 0;
    float centerY = 0;
    float scalingFactor = 1;//5000.0;
    int depth_idx = 0;

    for (int v = 0; v < 480; ++v)
    {
        for (int u = 0; u < 640; ++u, ++depth_idx,this->depth_idx_f++)
        {

            pcl::PointXYZRGB & pt = pointcloud->points[depth_idx];
            pcl::PointXYZRGB & f_pt = final_cloud->points[this->depth_idx_f];

            pt.z = depthMat.at<uint16_t>(v,u) / scalingFactor ;

            if (pt.z > MINIMUM_DISTANCE_MM){
                depth_idx --;
                continue;
            }
            cout << pt.z << endl;
            pt.x = (static_cast<float> (u) - centerX) * pt.z / focalLength ;
            pt.y = (static_cast<float> (v) - centerY) * pt.z / focalLength ;
            pt.b = rgbMat.at<cv::Vec3b>(v,u)[0];
            pt.g = rgbMat.at<cv::Vec3b>(v,u)[1];
            pt.r = rgbMat.at<cv::Vec3b>(v,u)[2];


        }
    }
    //basic data transform
    Target target={0,0,0,-M_PI_2,0,-M_PI_2};
    this->transformPointCloud(pointcloud,target);
/*
    switch(id){
        case 10 :
            this->transformPointCloud(pointcloud,this->target_10);
            break;
        case 20  :
            this->transformPointCloud(pointcloud,this->target_20);
            break;
        case 30 :
            this->transformPointCloud(pointcloud,this->target_30);
            break;
        case 40  :
            this->transformPointCloud(pointcloud,this->target_40);
            break;
        case 50 :
            this->transformPointCloud(pointcloud,this->target_50);
            break;
        case 60  :
            this->transformPointCloud(pointcloud,this->target_60);
            break;
        case 70 :
            this->transformPointCloud(pointcloud,this->target_70);
            break;
        case 80  :
            this->transformPointCloud(pointcloud,this->target_80);
            break;
        default :;
    }*/

}

int main(int argc, char **argv) {

    string file_name;
    string read_file_name;

    pcl::PLYReader Reader;


    logging *data_load = new logging(1);
    Meshgen *mesh_generator = new Meshgen();

    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat depthMat(Size(640,480),CV_16UC1);

    //cout << "Zadajte id obrazka" << endl;
    //cin >> file_name;

    //if(!data_load->openDir("test"))
     //   return 0;


    //namedWindow("filtered_depth",CV_WINDOW_AUTOSIZE);
    //namedWindow("rgb_show",CV_WINDOW_AUTOSIZE);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    for (int i = 10;i<=80;i = i+10){

        Reader.read("/home/mirec/Documents/Capture_data_kinect/test/target_"+to_string(i)+".ply", *cloud);
        viewer.showCloud(cloud);

        //cvWaitKey(5000);
    }

    Reader.read("/home/mirec/Documents/Capture_data_kinect/test/target_"+to_string(10)+".ply", *cloud);
    pcl::io::savePLYFileBinary("novy_cloud.ply",*cloud);

        /*if(data_load->readImageDepth("depth_target_"+file_name,depthMat)) {

            depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
            cv::imshow("filtered_depth", depthf);
        }
        if(data_load->readImageDepth("rgb_target_"+file_name,rgbMat)) {

            depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
            cv::imshow("rgb_show", rgbMat);
        }*/

   // }
    cvWaitKey(10000000);

    return 0;
}
