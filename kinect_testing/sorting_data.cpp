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

int main(int argc, char **argv) {

    string file_name;
    logging *data_log;
    data_log = new logging;

    bool die(false);
    string filename("snapshot");
    string suffix(".png");

    Mat depthMat(Size(640,480),CV_16UC1);
    Mat depthf (Size(640,480),CV_8UC1);
    Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
    Mat ownMat(Size(640,480),CV_8UC3,Scalar(0));

    vector<Mat> read_images;

    cout << "Zadajte nazov priecinku" << endl;
    cin >> file_name;

    if(!data_log->openDir(file_name))
        return 0;

    //namedWindow("rgb",CV_WINDOW_AUTOSIZE);
    namedWindow("depth",CV_WINDOW_AUTOSIZE);

    /*name_rgb = "rgb_image" + to_string(i) + ".png";
    name_edit = "depth_edit_image" + to_string(i) + ".png";
    name_depth = "depth_image" + to_string(i) + ".png";*/
    int i = 0;
    if(data_log->readImageDepth(file_name,read_images)) {
        cout << "citam" << endl;

        while (i != read_images.size()) {
            cv::imshow("depth", read_images[i]);
            i++;
            char k = cvWaitKey(100);
        }

    }


    //cv::imshow("rgb", rgbMat);
    //depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0); //konverzia iba pred zobrazovanim
    //opisat techniky ziskania hlbkoveho obrazu
    //Lukas Martis materialy na sposoby ziskavania hlbkoveho obrazu
    //cca 150 snimkov na potlacenie sumu
    //medianovy filter na potlacanie sumu hodnotu kazdeho pixela v obraze ziskam tak ze spravim sucet pixelu vo vsetkych obrazoch a spravim median
    // aby som vybral najvhodnejsie merania


    //tacv::imshow("depth",depthf);



    return 0;
}
