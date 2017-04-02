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
    void transformOriginPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target target_origin);
    void generatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & final_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud);

    const struct Target target_10 ={0.700,0,0.200,0,0,0,0};
    const struct Target target_20 ={1.049999698851,-0.35000005995,0.549999355294,0,0,0,0};
    const struct Target target_30 ={1.400,-0.7,0.2,0,0,0,0};

    const struct Target target_40 ={0.700,-1.400,0.200,0,0,0,0}; // musim ydat kladne

    const struct Target target_50 ={0.0,-0.700,0.200,0,0,0,0};

    //const struct Target target_60 ={0.700,-0.600,0.900,0,0,0,0};

    const struct Target target_70 ={1.050,-1.050,0.550,0,0,0,0};
    const struct Target target_80 ={0.350,-1.050,0.550,0,0,0,0};


    int depth_idx_f;
};


Meshgen::Meshgen(){
    depth_idx_f = 0;

}
void Meshgen::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target target) {


    Target fixed_target;

    fixed_target.tx = target.tx;
    fixed_target.ty = target.tz;
    fixed_target.tz = -target.ty;

    Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_z = Eigen::Affine3f::Identity();

    Eigen::Affine3f transform_x_y_z = Eigen::Affine3f::Identity();

    // Define a translation of 2.5 meters on the x axis.
    //std::cout << transform_x_y_z.matrix() << std::endl;
    transform_x_y_z.translation() << fixed_target.tx, fixed_target.ty, fixed_target.tz;
    std::cout << transform_x_y_z.matrix() << std::endl;

    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_x_y_z);

    transform_x.rotate (Eigen::AngleAxisf (target.rx, Eigen::Vector3f::UnitX()));

    transform_y.rotate (Eigen::AngleAxisf (target.ry, Eigen::Vector3f::UnitY()));

    transform_z.rotate (Eigen::AngleAxisf (target.rz, Eigen::Vector3f::UnitZ()));


    //std::cout << transform_x.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_x);

    //std::cout << transform_y.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_y);
    // You can either apply transform_1 or transform_2; they are the same
    //std::cout << transform_z.matrix() << std::endl;
    pcl::transformPointCloud (*pointcloud, *pointcloud, transform_z);
}

void Meshgen::transformOriginPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud, Target target_origin) {

    //TODO Implement trasnformation of pointcloud sensor origin;
    Eigen::Quaternionf rotation(target_origin.rw,target_origin.rx,target_origin.ry,target_origin.rz);
    Eigen::Vector4f translation(target_origin.tx,target_origin.ty,target_origin.tz,0);

    cout << "tocim" << endl;
    pointcloud->sensor_orientation_ = rotation;
    pointcloud->sensor_origin_ = translation;
}


void Meshgen::generatePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & final_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud) {
    //pointcloud->width = rgbMat.cols;
    //pointcloud->height = rgbMat.rows;
    //pointcloud->points.resize (pointcloud->height * pointcloud->width);

    //const struct Target target_help ={0,0,0,0,1.57,0,0};

    //transformPointCloud(pointcloud,target_help);
    for (int i; i < pointcloud->points.size();i++){
        final_cloud->points.push_back(pointcloud->points[i]);
    }

/*
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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    for (int i = 10;i<=60;i = i+10){



        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        Reader.read("/home/mirec/Documents/Capture_data_kinect/test/target_"+to_string(i)+".ply", *cloud);
        Target target = {0, 0, 0, 0, 3.14, 0, 0};

       switch(i){
            case 10 :
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_10);
                break;
            case 20  :
                continue;
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_20);
                break;
            case 30 :
                target = {0, 0, 0, 0, -1.57, 0, 0};
                mesh_generator->transformPointCloud(cloud,target);
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_30);
                break;
            case 40  :
                target = {0, 0, 0, 0, 3.14, 0, 0};
                mesh_generator->transformPointCloud(cloud,target);
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_40);
                break;
            case 50 :
                target = {0, 0, 0, 0, 1.57, 0, 0};
                mesh_generator->transformPointCloud(cloud,target);
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_50);
                break;
            case 60  :
                continue;
                target = {0, 0, 0, -1.570, 0, 0, 0};
                mesh_generator->transformPointCloud(cloud,target);
                target ={0.700,-0.900,-0.6,0,0,0,0};
                mesh_generator->transformPointCloud(cloud,target);
                break;
            case 70 :
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_70);
                break;
            case 80  :
                mesh_generator->transformPointCloud(cloud,mesh_generator->target_80);
                break;
            default :;
        }
        //pcl::io::savePLYFileBinary("novy_cloud_"+to_string(i)+".ply",*cloud);
        mesh_generator->generatePointCloud(mesh_cloud,cloud);
        viewer.showCloud(mesh_cloud);
        //sleep(10);
    }
    pcl::io::savePLYFileBinary("novy_cloud.ply",*mesh_cloud);
    sleep(50);
    /*viewer.showCloud(mesh_cloud);
    sleep(10);
    cvWaitKey(5000000);*/

    //pcl::io::savePLYFileBinary("novy_cloud.ply",*mesh_cloud);

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
