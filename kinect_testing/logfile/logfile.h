//
// Created by mirec on 2017.02.26..
//

#ifndef KINECT_TESTING_FILE_LOG_H
#define KINECT_TESTING_FILE_LOG_H

#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

using namespace std;
using namespace cv;

class logfile {

private:

public:

    logfile(string dir_name);

    bool saveImage(Mat input_image, string name);
    bool saveLog(string file_name);
    bool readImageVideo(string file_name, Mat & output_image);
    bool readImageDepth(string file_name, Mat & output_image);

private:
    string dir_path;

};


#endif //KINECT_TESTING_FILE_LOG_H
