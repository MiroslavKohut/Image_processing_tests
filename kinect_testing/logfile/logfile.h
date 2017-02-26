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

    logfile();
    ~logfile();

public:

    void createFile(string file_name);
    void saveImage(Mat);
    void saveLog(string file_name);
    Mat readImage(string file_name);


};


#endif //KINECT_TESTING_FILE_LOG_H
