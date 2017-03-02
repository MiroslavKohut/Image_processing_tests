#include <libfreenect.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include "logging.h"
#include <unistd.h>

using namespace cv;
using namespace std;


class myMutex {
	public:
		myMutex() {
			pthread_mutex_init( &m_mutex, NULL );
		}
		void lock() {
			pthread_mutex_lock( &m_mutex );
		}
		void unlock() {
			pthread_mutex_unlock( &m_mutex );
		}
	private:
		pthread_mutex_t m_mutex;
};


class MyFreenectDevice : public Freenect::FreenectDevice {
	public:
		MyFreenectDevice(freenect_context *_ctx, int _index)
	 		: Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(FREENECT_DEPTH_REGISTERED),
			m_buffer_rgb(FREENECT_VIDEO_RGB), m_gamma(2048), m_new_rgb_frame(false),
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1),
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)),
			ownMat(Size(640,480),CV_8UC3,Scalar(0)) {

			for( unsigned int i = 0 ; i < 2048 ; i++) {
				float v = i/2048.0;
				v = std::pow(v, 3)* 6;
				m_gamma[i] = v*6*256;
			}
		}

		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp) {
			//std::cout << "RGB callback" << std::endl;
			m_rgb_mutex.lock();
			uint8_t* rgb = static_cast<uint8_t*>(_rgb);
			rgbMat.data = rgb;
			m_new_rgb_frame = true;
			m_rgb_mutex.unlock();
		};

		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp) {
			//std::cout << "Depth callback" << std::endl;
			m_depth_mutex.lock();
			uint16_t* depth = static_cast<uint16_t*>(_depth);
			depthMat.data = (uchar*) depth;
			m_new_depth_frame = true;
			m_depth_mutex.unlock();
		}

		bool getVideo(Mat& output) {
			m_rgb_mutex.lock();
			if(m_new_rgb_frame) {
				cv::cvtColor(rgbMat, output, CV_RGB2BGR);
				m_new_rgb_frame = false;
				m_rgb_mutex.unlock();
				return true;
			} else {
				m_rgb_mutex.unlock();
				return false;
			}
		}

		bool getDepth(Mat& output) {
				m_depth_mutex.lock();
				if(m_new_depth_frame) {
					depthMat.copyTo(output);
					m_new_depth_frame = false;
					m_depth_mutex.unlock();
					return true;
				} else {
					m_depth_mutex.unlock();
					return false;
				}
			}
	private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		Mat depthMat;
		Mat rgbMat;
		Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};


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

    Freenect::Freenect freenect;
    MyFreenectDevice& device = freenect.createDevice<MyFreenectDevice>(0);

    cout << "Zadajte nazov priecinku" << endl;
    cin >> file_name;

    if(!data_log->createDir(file_name))
        return 0;

	// The next two lines must be changed as Freenect::Freenect
	// isn't a template but the method createDevice:
	// Freenect::Freenect<MyFreenectDevice> freenect;
	// MyFreenectDevice& device = freenect.createDevice(0);
	// by these two lines:



	namedWindow("rgb",CV_WINDOW_AUTOSIZE);
	namedWindow("depth",CV_WINDOW_AUTOSIZE);

    device.startVideo();
	device.startDepth();
    usleep(250000);
    device.stopVideo();
    device.stopDepth();

    device.startVideo();
    device.startDepth();
	usleep(2500000);

    string name_rgb;
    string name_depth;
    string name_edit;
    int i = 0;

    for (i; i<150;i++){

        device.getVideo(rgbMat);
        device.getDepth(depthMat);

        //cv::imshow("rgb", rgbMat);
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0); //konverzia iba pred zobrazovanim
        //opisat techniky ziskania hlbkoveho obrazu
        //Lukas Martis materialy na sposoby ziskavania hlbkoveho obrazu
        //cca 150 snimkov na potlacenie sumu
        //medianovy filter na potlacanie sumu hodnotu kazdeho pixela v obraze ziskam tak ze spravim sucet pixelu vo vsetkych obrazoch a spravim median
        // aby som vybral najvhodnejsie merania


        //tacv::imshow("depth",depthf);

        name_rgb = "rgb_image" + to_string(i) + ".png";
        name_edit = "depth_edit_image" + to_string(i) + ".png";
        name_depth = "depth_image" + to_string(i) + ".png";
        //TODO rovno ukladat depthMat a potom loadovat cez povodny load

        if(!data_log->saveImage(rgbMat,name_rgb)){
            return 0;
        }
		if(!data_log->saveImage(depthMat,name_depth)){
			return 0;
		}
        if(!data_log->saveImage(depthf,name_edit)){
            return 0;
        }
    }
    data_log->saveLog(to_string(i),name_depth,name_rgb);

	device.stopVideo();
	device.stopDepth();
    delete data_log;

	return 0;
}

