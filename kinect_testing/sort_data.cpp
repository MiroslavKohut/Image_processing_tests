//
// Created by mirec on 2017.03.01..
//
#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "logging.h"

#define IMAGES_COUNT 150


using namespace std;
using namespace cv;
class Filter{

public:

    bool imagesMedian(vector<Mat> input_images, Mat output_image);

    Filter();
    ~Filter();

private:
    uint16_t calcMedian(vector<uint16_t> data);
};

Filter::Filter(){

}

bool Filter::imagesMedian(vector<Mat> input_images, Mat output_image) {

    for (int v = 0; v < 480; ++v) {
        for (int u = 0; u < 640; ++u) {
            std::vector<uint16_t> data_row;
            for (int i = 0; i < IMAGES_COUNT; i++) {
                uint16_t pixel = input_images[i].at<uint16_t>(v, u);
                data_row.push_back(pixel);
            }
            output_image.at<uint16_t>(v, u) = calcMedian(data_row);
        }
    }
    return true;
}

uint16_t Filter::calcMedian(vector<uint16_t > data)
{
    double median;
    size_t size = data.size();

    sort(data.begin(), data.end());

    if (size  % 2 == 0)
    {
        median = (data[size / 2 - 1] + data[size / 2]) / 2;
    }
    else
    {
        median = data[size / 2];
    }

    return median;
}


int main(int argc, char **argv) {

    string file_name;
    string read_file_name;
    vector<Mat> read_images;
    vector<Mat> read_rgb_images;

    logging *data_log = new logging(IMAGES_COUNT);
    logging *data_save = new logging(1);
    Filter *data_filter = new Filter;


    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat depthMat(Size(640,480),CV_16UC1);

    cout << "Zadajte nazov priecinku" << endl;
    namedWindow("depth",CV_WINDOW_AUTOSIZE);
    namedWindow("filtered_depth",CV_WINDOW_AUTOSIZE);

    cin >> file_name;
    if(!data_save->createDir(file_name))
        return 0;


    for (int i=10;i<=80;i=i+10){
        read_file_name = "target_"+to_string(i);
        if(!data_log->openDir(read_file_name))
            return 0;

        if(data_log->readImageDepth(read_file_name,read_images)) {
            cout << "file_name" << endl;

            data_filter->imagesMedian(read_images, depthMat);
            depthMat.convertTo(depthf, CV_8UC1, 255.0 / 2048.0);
            //cv::imshow("filtered_depth", depthf);

            //cout << "M = "<< endl << " "  << read_images[0]<< endl << endl;
            /*int i = 0;
            while (i != read_images.size()) {
                read_images[i].convertTo(depthf, CV_8UC1, 255.0/2048.0);
                cv::imshow("depth", depthf);
                i++;
                char k = cvWaitKey(100);
            }*/
            if(!data_save->saveImage(depthMat,"depth_target_"+to_string(i)+".png")){
                return 0;
            }
            if(!data_save->saveImage(depthf,"depth_viz_target_"+to_string(i)+".png")){
                return 0;
            }

        }

        if(data_log->readImageVideo(read_file_name,read_rgb_images)){
            if(!data_save->saveImage(read_rgb_images[0],"rgb_target_"+to_string(i)+".png")){
                return 0;
            }
        }

    }

    return 0;
}
