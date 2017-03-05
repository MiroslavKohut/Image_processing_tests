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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "logging.h"

#define IMAGES_COUNT 150
#define MINIMUM_DISTANCE_MM 700


using namespace std;
using namespace cv;
class Meshgen{

public:

    Meshgen();
    ~Meshgen();
    void generatePointCloud(const cv::Mat & rgbMat, const cv::Mat & depthMat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud);

private:

};


void Meshgen::generatePointCloud(const cv::Mat & rgbMat, const cv::Mat & depthMat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud) {
    pointcloud->width = rgbMat.cols;
    pointcloud->height = rgbMat.rows;
    pointcloud->points.resize (pointcloud->height * pointcloud->width);

    float  focalLength = 525;
    float centerX = 319.5;
    float centerY = 239.5;
    float x_translation = -25.07450;
    float y_translation = 0.28102;
    float z_translation =0.79722;
    float scalingFactor = 1;//5000.0;
    int depth_idx = 0;

    for (int v = 0; v < 480; ++v)
    {
        for (int u = 0; u < 640; ++u, ++depth_idx)
        {

            pcl::PointXYZRGB & pt = pointcloud->points[depth_idx];

            pt.z = depthMat.at<uint16_t>(v,u) / scalingFactor ;

            if (pt.z > MINIMUM_DISTANCE_MM){
                depth_idx --;
                continue;
            }

            pt.x = (static_cast<float> (u) - centerX) * pt.z / focalLength ;
            pt.y = (static_cast<float> (v) - centerY) * pt.z / focalLength ;
            pt.b = rgbMat.at<cv::Vec3b>(v,u)[0];
            pt.g = rgbMat.at<cv::Vec3b>(v,u)[1];
            pt.r = rgbMat.at<cv::Vec3b>(v,u)[2];

        }
    }

}

Meshgen::Meshgen(){

}

int main(int argc, char **argv) {

    string file_name;
    string read_file_name;


    logging *data_load = new logging(1);
    Meshgen *mesh_generator = new Meshgen();

    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat depthMat(Size(640,480),CV_16UC1);

    namedWindow("filtered_depth",CV_WINDOW_AUTOSIZE);
    namedWindow("rgb_show",CV_WINDOW_AUTOSIZE);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    cout << "Zadajte id obrazka" << endl;
    cin >> file_name;

    if(!data_load->openDir("test"))
        return 0;

    if(data_load->readImageDepth("depth_target_"+file_name,depthMat)) {

        depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
        cv::imshow("filtered_depth", depthf);
    }
    if(data_load->readImageDepth("rgb_target_"+file_name,rgbMat)) {

        depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
        cv::imshow("rgb_show", rgbMat);
    }

    mesh_generator->generatePointCloud(rgbMat,depthMat,cloud);
    viewer.showCloud(cloud);

    cvWaitKey(10000000);

    return 0;
}
