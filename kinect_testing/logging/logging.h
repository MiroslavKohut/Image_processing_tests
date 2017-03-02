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

class logging {

private:

public:

    logging();
    ~logging();
    bool createDir(string dir_name);
    bool openDir(string dir_name);
    bool saveImage(Mat input_image, string name);
    bool saveLog(string images_count, string depth_name, string rgb_name);
    bool readImageVideo(string file_name, vector<Mat> &output_image_vec);
    bool readImageDepth(string file_name, vector<Mat> &output_image_vec);

private:
    string dir_path;
    std::ofstream *outfile;


};


#endif //KINECT_TESTING_FILE_LOG_H
