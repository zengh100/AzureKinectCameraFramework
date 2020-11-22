#pragma comment(lib, "k4a.lib")
// Azure Kinect SDK Library headers
#include <k4a/k4a.h>
#include <k4a/k4a.hpp>
#include <k4a/k4atypes.h>
#include <k4arecord/playback.h>

#include <stdio.h>
#include <stdlib.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/rgbd.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>

#include <glob.h>
#include <regex>

#include "AzureKinectCamera.h"
//Only available for C++ 17
//#include <filesystem>
//namespace fs = std::filesystem;

//#include "tensorflow/core/platform/default/logging.h"
//#include <glog/logging.h>

using namespace std::chrono;
//#include <opencv2/viz.hpp>

//#include <thread>
//using namespace std::chrono;
//using namespace cv;

using namespace std;
using namespace cv;

inline bool exists_file (const std::string& name) {
    ifstream f(name.c_str());
    return f.good();
}

vector<string> split(const string& str, const string& delim)
{
    vector<string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == string::npos) pos = str.length();
        string token = str.substr(prev, pos-prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
    return tokens;
}


std::vector<std::string> glob(const std::string& pattern) {
    using namespace std;

    // glob struct resides on the stack
    glob_t glob_result;
    memset(&glob_result, 0, sizeof(glob_result));

    // do the glob operation
    int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
    if(return_value != 0) {
        globfree(&glob_result);
        stringstream ss;
        ss << "glob() failed with return_value " << return_value << endl;
        throw std::runtime_error(ss.str());
    }

    // collect all the filenames into a std::list<std::string>
    vector<string> filenames;
    for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
        filenames.push_back(string(glob_result.gl_pathv[i]));
    }

    // cleanup
    globfree(&glob_result);

    // done
    return filenames;
}

double random_between_zeor_and_one()
{
	return ((double) rand() / (RAND_MAX));
}

/*
int TestVideoOpen_CPP() //Couldn't use cpp head file "k4arecord.hpp" is not installed yet 
{
	string video_filename = "../video/test.mkv";
    k4a::playback handle =  k4a::playback::open(video_filename);

    std::chrono::seconds s = std::chrono::duration_cast<std::chrono::seconds>(handle.get_recording_length());
    printf("duration of record: %li\n", s.count());

    k4a::capture cap;
    while(handle.get_next_capture(&cap))
    {
        k4a::image img = cap.get_depth_image();

        int rows = img.get_height_pixels();
        int cols = img.get_width_pixels();
        std::cout << "depth image size: " << img.get_size() << "\n";
        std::cout << "Timestamp: " << img.get_device_timestamp().count() << "\n";
        cv::Mat depthMat(rows, cols, CV_16UC1, (void *) img.get_buffer(), cv::Mat::AUTO_STEP);
        cv::imshow("Bla", depthMat);

        k4a::image img2 = cap.get_color_image();
        if(img2){
            int rows2 = img2.get_height_pixels();
            int cols2 = img2.get_width_pixels();
            std::cout << "color image size: " << img2.get_size() << "\n";
            std::cout << "Timestamp: " << img2.get_device_timestamp().count() << "\n";
            std::cout << "width: " << cols2 << "\n";
            std::cout << "heigth: " << rows2 << "\n";
            cv::Mat rawData( 1, img2.get_size(), CV_8UC1, (void*)img2.get_buffer() );
            cv::Mat decodedImage  =  cv::imdecode( rawData,  cv::IMREAD_UNCHANGED );
            cv::imshow("bla",decodedImage);
            cv::waitKey(1);
        }
    }
}
*/

int TestVideoOpen() 
{
	string video_filename = "../video/test.mkv";
	k4a_playback_t playback_handle = NULL;
	if (k4a_playback_open(video_filename.c_str(), &playback_handle) != K4A_RESULT_SUCCEEDED)
	{
		printf("Failed to open recorded file s%\n", video_filename.c_str());
		return 1;
	}
	uint64_t recording_length = k4a_playback_get_last_timestamp_usec(playback_handle);
	printf("Recording is %lld seconds long\n", recording_length / 1000000);

	//k4a_device_configuration_t config;
	k4a_record_configuration_t config;
	/* 
	below is the config stetting for Beam4
	config.camera_fps = K4A_FRAMES_PER_SECOND_30; //k4a_fps_t == 2
	config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED; //k4a_depth_mode_t ==2,  Depth captured at 640x576. Passive IR is also captured at 640x576.
	config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32; //k4a_image_format_t == 3
	config.color_resolution = K4A_COLOR_RESOLUTION_720P; //k4a_color_resolution_t == 1, 1280 x 720, 16:9 
	//config.color_resolution = K4A_COLOR_RESOLUTION_OFF;//Color camera will be turned off with this setting
	//config.color_resolution = K4A_COLOR_RESOLUTION_1536P; // 2048 x 1536   4:3

	// This means that we'll only get captures that have both color and
	// depth images, so we don't need to check if the capture contains
	// a particular type of image.
	//
	config.synchronized_images_only = true;
	*/
	//K4ARECORD_EXPORT k4a_result_t 
	k4a_playback_get_record_configuration(playback_handle, &config);
	std::cout << getK4A_FRAMES_PER_SECOND(config.camera_fps) 
	          << ", depth_mode=" << getK4A_DEPTH_MODE(config.depth_mode)
			  << ", color_format=" << getK4A_IMAGE_FORMAT(config.color_format)
			  << ", color_resolution=" << getK4A_COLOR_RESOLUTION(config.color_resolution) 
			  //<< ", synchronized_images_only=" << config.synchronized_images_only 
			  << ", imu_track_enabled=" << config.imu_track_enabled
			  << "\n"; 

	k4a_calibration_t calibration;
	//k4a::calibration calibration;
	k4a_result_t result_get_cal = k4a_playback_get_calibration(playback_handle, &calibration);
	if(result_get_cal == K4A_RESULT_SUCCEEDED)
		std::cout << "Succeeded to get calibration\n";
	if(result_get_cal == K4A_RESULT_FAILED)
		std::cout << "Failed to get calibration\n";
	/* for unknown reason, not able to get transformation from the calibration even though getting calibration was succeessful.
	k4a_transformation_t tf = k4a_transformation_create(&calibration);
    //k4a::transformation transformation;
	//transformation = k4a::transformation(calibration);
	// */

	//return 0;
	k4a_capture_t capture = NULL;
	k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
	std::cout << "Starting to read frames...\n";
	while (result == K4A_STREAM_RESULT_SUCCEEDED)
	{
		k4a_imu_sample_t imu;
		switch(k4a_playback_get_next_imu_sample(playback_handle, &imu))
		{
		case K4A_STREAM_RESULT_SUCCEEDED:
			std::cout << "imu.acc_sample.xyz: " << imu.acc_sample.xyz.x << ", " << imu.acc_sample.xyz.y << ", " <<  imu.acc_sample.xyz.z << "\n";
			float roll, pitch;
			getRollPitch(imu, roll, pitch);
			std::cout << "roll: " << roll*rad_to_deg << "deg\n";
			std::cout << "pitch: " << pitch*rad_to_deg  << "deg\n";
			break;
		case K4A_STREAM_RESULT_FAILED:
			std::cout << "Failed to read imu data\n";
			break;
		case K4A_STREAM_RESULT_EOF:
			std::cout << "Reach end of imu data\n";
			break;
		}

		result = k4a_playback_get_next_capture(playback_handle, &capture);
		if (result == K4A_STREAM_RESULT_SUCCEEDED)
		{
			// Process capture here
			k4a::capture cap(capture);
			k4a::image img = cap.get_depth_image();

			int rows = img.get_height_pixels();
			int cols = img.get_width_pixels();
			std::cout << "depth image rows: " << rows << "\n";
			std::cout << "depth image cols: " << cols << "\n";
			std::cout << "depth image size: " << img.get_size() << "\n";
			std::cout << "Timestamp: " << img.get_device_timestamp().count() << "\n";
			cv::Mat depthMat(rows, cols, CV_16UC1, (void *) img.get_buffer(), cv::Mat::AUTO_STEP);
			cv::imshow("Depth Image", depthMat);

			k4a::image ir_img = cap.get_ir_image();
			//cv::Mat irMat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
			cv::Mat irMat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
			convertIrImageToMat(ir_img, irMat);
			cv::imshow("Infrared Image", irMat);

			k4a::image img2 = cap.get_color_image();
			if(img2){
				int rows2 = img2.get_height_pixels();
				int cols2 = img2.get_width_pixels();
				std::cout << "color image size: " << img2.get_size() << "\n";
				std::cout << "Timestamp: " << img2.get_device_timestamp().count() << "\n";
				std::cout << "width: " << cols2 << "\n";
				std::cout << "heigth: " << rows2 << "\n";
				cv::Mat rawData( 1, img2.get_size(), CV_8UC1, (void*)img2.get_buffer() );
				cv::Mat decodedImage  =  cv::imdecode( rawData,  cv::IMREAD_UNCHANGED );
				cv::imshow("RGB Image",decodedImage);
			}			
			cv::waitKey(15);
			//k4a_capture_release(capture); //cap is local so it will take care of the release of capture
		}
		else if (result == K4A_STREAM_RESULT_EOF)
		{
			// End of file reached
			std::cout << "Reach end of stream\n";
			break;
		}
	}
	if (result == K4A_STREAM_RESULT_FAILED)
	{
		printf("Failed to read entire recording\n");
		return 1;
	}

	k4a_playback_close(playback_handle);	
	return 0;
}

//This class is to demo how to register your class member function to received data event from camera online/offline
class TestClass_CameraDataEventListener
{
public:
	void onDataEvent(float roll, float pitch, cv::Mat mat8UC1_infraredImg, k4a::image depth_image, k4a::image pointCloud, k4a::image color_image) 
	{
		std::cout << "roll = " << roll*rad_to_deg << " deg, pitch = " << pitch*rad_to_deg << " deg\n";
		std::cout << mat8UC1_infraredImg.cols << ", " << mat8UC1_infraredImg.rows << "\n";
		
		//add your code to process all data from the camera (online or offline)
		//your code
		
        //display the infrared image
		cv::imshow("Infrared Image", mat8UC1_infraredImg);
        
        //display color image
        if(color_image)
        {
            cv::Mat rawData( 1, color_image.get_size(), CV_8UC1, (void*)color_image.get_buffer() );
            cv::Mat decodedImage  =  cv::imdecode( rawData,  cv::IMREAD_UNCHANGED );
            cv::imshow("RGB Image",decodedImage);
        }
		cvWaitKey(1);//this is necessary to display images
	}
};

void TestClass_AzureKinectCamera(std::string video_filename)
{
    //create a cemra instance online(empty file name); otherwise it will load a video recorded from the camera
	AzureKinectCamera camera(video_filename);

    // create a class instance that will process the data from the camera.
	TestClass_CameraDataEventListener listener;
	
	// register your handling function to the data event
	camera.RegistrateDataEvent(boost::bind( &TestClass_CameraDataEventListener::onDataEvent, &listener, _1, _2, _3, _4, _5, _6) );
	camera.setExtractColorImage(true); // this will make sure that color image is passed too.

	//all set, ready to start camera/video
	camera.run();
} 

