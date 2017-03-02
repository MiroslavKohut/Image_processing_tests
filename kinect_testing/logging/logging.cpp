//
// Created by mirec on 2017.02.26..
//

#include <sys/types.h>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>

#include "logging.h"

/*Exceptions*/
class myexception: public exception
{
    virtual const char* what() const throw()
    {
        return "Could not create directory";
    }
} myex;

/* Methods */

logging::logging() {
    this->dir_path = "";
}

logging::~logging(){


}

bool logging::createDir(string dir_name) {

    this->dir_path = "/home/mirec/Documents/Capture_data_kinect/" + dir_name;

    // TODO check directory existence
    struct stat sb;
    if (stat(dir_path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
    {
        cout << "Directory already exists"<< endl;
        throw myex;
    }
    else{

        string command  = "mkdir -p " + dir_path;

        const int dir_err = system(command.c_str());

        if (-1 == dir_err){
            cout << command << endl;
            throw myex;
        }
        else{
            cout << "Directory created" << endl;
            return true;
        }
    }
}

bool logging::openDir(string dir_name) {

    this->dir_path = "/home/mirec/Documents/Capture_data_kinect/" + dir_name;

    // TODO check directory existence
    struct stat sb;
    if (stat(dir_path.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
    {
        return true;
    }
    else{
        this->dir_path ="";
        return false;
    }
}

bool logging::saveImage(Mat input_image, string name) {


    if(dir_path != ""){
        stringstream file;

        file << this->dir_path << "/" << name;
        cout << file.str() <<endl;

        try {
            imwrite(file.str(), input_image);
            return true;
        }
        catch (cv::Exception& ex) {
            fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());
            return false;
        }
    }
    else
        return false;
}

bool logging::saveLog(string images_count, string depth_name, string rgb_name)  {

    if(dir_path != ""){


        outfile = new std::ofstream((dir_path + "/logging.txt").c_str());

        *outfile << images_count << std::endl;
        *outfile << depth_name << std::endl;
        *outfile << rgb_name << std::endl;

        outfile->close();
        delete outfile;
        return true;
    }
    else
        return false;
}


bool logging::readImageDepth(string file_name, vector<Mat> &output_image_vec) {


    if(this->openDir(file_name)){
        std::string path= this->dir_path + "/" + "depth_edit_image";

        for (int i =0;i<150;i++){

            Mat image = imread(path + to_string(i) + ".png", cv::IMREAD_UNCHANGED);
            output_image_vec.push_back(image);
        }
        return true;
    }
    else
        return false;

}

bool logging::readImageVideo(string file_name, vector<Mat> &output_image_vec) {

    if(this->openDir(file_name)){
        Mat output;
        std::string path= this->dir_path + "/" + "rgb_image";

        for (int i =0;i<150;i++){

            Mat image = imread(path + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);
            image.copyTo(output);
            output_image_vec.push_back(output);

        }
        return true;
    }
    else
        return false;
}

