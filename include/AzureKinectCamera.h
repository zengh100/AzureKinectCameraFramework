#ifndef AZURE_KINECT_CAMERA_H
#define AZURE_KINECT_CAMERA_H

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
#include "time2string.h"
#include "frame_rate_watcher.h"

//#include "BeamConfig.hpp"
#include <boost/signals2.hpp>
#include <thread>


//Only available for C++ 17
//#include <filesystem>
//namespace fs = std::filesystem;

//#include "tensorflow/core/platform/default/logging.h"
//#include <glog/logging.h>

using namespace std::chrono;
using namespace std;
using namespace cv;

const float rad_to_deg = 180.0f/3.14159265;
const float deg_to_rad = 3.14159265/180.0f;

std::string getK4A_FRAMES_PER_SECOND(k4a_fps_t fps);
std::string getK4A_DEPTH_MODE(k4a_depth_mode_t depth_mode);
std::string getK4A_IMAGE_FORMAT(k4a_image_format_t image_format);
std::string getK4A_COLOR_RESOLUTION(k4a_color_resolution_t color_resolution);

void getRollPitch(k4a_float3_t &acc, float &roll, float &pitch);
void getRollPitch(k4a_imu_sample_t &imu, float &roll, float &pitch);
void convertIrImageToMat(const k4a::image &irImage, cv::Mat &irMat, uint16_t min_value = 0, uint16_t max_value = 1000);

enum playbackMode 
{
    frame_by_frame = 0,
    continuously = 1
};

class AzureKinectCamera
{
public:
    // Shortcut typedefs (define event)
    typedef boost::signals2::signal<void(float, float, cv::Mat, k4a::image, k4a::image, k4a::image)>   data_event_t;
    typedef data_event_t::slot_type        data_event_slot;

    //define event emitter
    data_event_t data_event;

    //Two steps for an instance
    AzureKinectCamera();
    void openDevice(std::string offline_filepath="");

    //One step for an instance
    AzureKinectCamera(string offline_filepath);

    ~AzureKinectCamera();

    // void RegistrateClickEvent(registration_request slot);

    // void RegistratePrimaryHandFoundEvent(primary_hand_found_event_slot slot);

    void RegistrateDataEvent(data_event_slot slot);

    void run();
    void stop();
    //cv::Mat GetIR() {return mat8UC1;}
    //k4a::image GetPointCloud() { return point_cloud_image; }

    void convertDepthImageToMat(k4a_image_t &depthImage, cv::Mat &depthMat);
    void convertDepthImageToMat(const k4a::image &depthImage, cv::Mat &depthMat);
    void convertDepthImageToMatRGB(const k4a::image &depthImage, cv::Mat &depthMat);
    void displayDepthImage(cv::Mat &depthImage_16U);
    CvScalar blendTwoColors(CvScalar color1, CvScalar color2, float weight1);

    void Initialize();
    void play_one_frame_from_video(bool nextFrame);
    void setExtractColorImage(bool bExtract) {m_bExtractColorImage=bExtract;}
    void setDisplayImage(bool bDisplay) {m_bDisplayImage=bDisplay;}    

private:
    //Configure for camera
    k4a::device dev;
    k4a_device_configuration_t config;
    k4a::calibration calibration;
    k4a::transformation transformation;

    //Configure for video (offline file)
    string video_filename;
	k4a_playback_t playback_handle;
	k4a_record_configuration_t config_video;
    k4a_calibration_t calibration_video;
	//k4a::calibration calibration_video;
    //transformation for video not working yet

    float roll, pitch;

    double elapse_time = 0;
    k4a::image last_depthFrame;
    FrameRateWatcher fps_watcher;

    bool running;
    uint64_t frame_id;

    void initialize_camera();
    void initialize_video();
    bool initialized;
    bool m_online;
    void run_camera();
    void run_video();
    void run_video_old();

    bool m_bDisplayImage;

    //settings for offline mode
    bool m_bExtractColorImage;
};

#endif
