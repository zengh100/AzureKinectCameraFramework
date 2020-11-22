#include "AzureKinectCamera.h"

template<typename T>
inline void ConvertToGrayScaleImage(const T *imgDat, const int size, const int vmin, const int vmax, uint8_t *img)
{
	for (int i = 0; i < size; i++)
	{
		T v = imgDat[i];
		float colorValue = 0.0f;
		if (v <= vmin)
		{
			colorValue = 0.0f;
		}
		else if (v >= vmax)
		{
			colorValue = 1.0f;
		}
		else
		{
			colorValue = (float)(v - vmin) / (float)(vmax - vmin);
		}
		img[i] = (uint8_t)(colorValue * 255);
	}
}

void getRollPitch(k4a_float3_t &acc, float &roll, float &pitch)
{
	float x = acc.xyz.y;
	float y = acc.xyz.x;
	float z = acc.xyz.z;
	float sum = sqrt(x*x + y*y + z*z);
	x /= sum;
	y /= sum;
	z /= sum;
	pitch = asin(x);
	roll = atan2(-y, -z);
	//std::cout << "imu.acc_sample: " << x << ", "  << y << ", " << z << "\n";
	//std::cout << "Roll = " << roll *  rad_to_deg << " deg, pitch = " << pitch * rad_to_deg << " deg\n";
}

void getRollPitch(k4a_imu_sample_t &imu, float &roll, float &pitch)
{
	getRollPitch(imu.acc_sample, roll, pitch);
}

std::string getK4A_FRAMES_PER_SECOND(k4a_fps_t fps)
{
	switch(fps)
	{
    case K4A_FRAMES_PER_SECOND_5:     /**< 5 FPS */
		return "Camera FpS = 5";
		break;
    case K4A_FRAMES_PER_SECOND_15:    /**< 15 FPS */
	 	return "Camera FpS = 15";
		break;
    case K4A_FRAMES_PER_SECOND_30:    /**< 30 FPS */
	 	return "Camera FpS = 30";
		break;
	}
 	return "Camera FpS = unknown";
}

void convertIrImageToMat(const k4a::image &irImage, cv::Mat &irMat, uint16_t min_value, uint16_t max_value)
{
    if (irImage.handle() != NULL)
    {
        // you can check the format with this function
        k4a_image_format_t format = irImage.get_format(); // K4A_IMAGE_FORMAT_DEPTH16
        if (format != K4A_IMAGE_FORMAT_IR16)
            std::cerr << "Error on convertIrImageToMat(): the input isn't an ir image\n";

        int rows = irImage.get_height_pixels();
        int cols = irImage.get_width_pixels();

        // get raw buffer
        //const uint8_t* buffer = depthImage.get_buffer();
        uint8_t* buffer = k4a_image_get_buffer(irImage.handle());
        //convert to uint16_t to process the depth
        uint16_t *irImg = reinterpret_cast<uint16_t *>(buffer);
        std::vector<uint8_t> grayScaleImg(rows * cols);
        ConvertToGrayScaleImage(irImg, cols * rows, min_value, max_value, grayScaleImg.data());
        uint8_t *im = grayScaleImg.data();

        // convert the raw buffer to cv::Mat
        //depthMat = cv::Mat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
        irMat = cv::Mat(rows, cols, CV_8U, (void*)im, cv::Mat::AUTO_STEP);
    }
}

std::string getK4A_DEPTH_MODE(k4a_depth_mode_t depth_mode)
{
	switch(depth_mode)
	{
    case K4A_DEPTH_MODE_OFF:         /**< Depth sensor will be turned off with this setting. */
		return "K4A_DEPTH_MODE_OFF";
    case K4A_DEPTH_MODE_NFOV_2X2BINNED: /**< Depth captured at 320x288. Passive IR is also captured at 320x288. */
		return "K4A_DEPTH_MODE_NFOV_2X2BINNED";
    case K4A_DEPTH_MODE_NFOV_UNBINNED:  /**< Depth captured at 640x576. Passive IR is also captured at 640x576. */
		return "K4A_DEPTH_MODE_NFOV_UNBINNED";
    case K4A_DEPTH_MODE_WFOV_2X2BINNED: /**< Depth captured at 512x512. Passive IR is also captured at 512x512. */
		return "K4A_DEPTH_MODE_WFOV_2X2BINNED";
    case K4A_DEPTH_MODE_WFOV_UNBINNED:  /**< Depth captured at 1024x1024. Passive IR is also captured at 1024x1024. */
		return "K4A_DEPTH_MODE_WFOV_UNBINNED";
    case K4A_DEPTH_MODE_PASSIVE_IR:     /**< Passive IR only, captured at 1024x1024. */
		return "K4A_DEPTH_MODE_PASSIVE_IR";
	}
	return "K4A_DEPTH_MODE_UNKNOWN";
} 

std::string getK4A_IMAGE_FORMAT(k4a_image_format_t image_format)
{
	switch(image_format)
	{
    case K4A_IMAGE_FORMAT_COLOR_MJPG:         
		return "K4A_IMAGE_FORMAT_COLOR_MJPG";
    case K4A_IMAGE_FORMAT_COLOR_NV12:
		return "K4A_IMAGE_FORMAT_COLOR_NV12";
    case K4A_IMAGE_FORMAT_COLOR_YUY2:  
		return "K4A_IMAGE_FORMAT_COLOR_YUY2";
    case K4A_IMAGE_FORMAT_COLOR_BGRA32: 
		return "K4A_IMAGE_FORMAT_COLOR_BGRA32";
    case K4A_IMAGE_FORMAT_DEPTH16:  
		return "K4A_IMAGE_FORMAT_DEPTH16";
    case K4A_IMAGE_FORMAT_IR16:     
		return "K4A_IMAGE_FORMAT_IR16";
	}
	return "K4A_IMAGE_FORMAT_UMKNOWN";
} 

std::string getK4A_COLOR_RESOLUTION(k4a_color_resolution_t color_resolution)
{
	switch(color_resolution)
	{
    case K4A_COLOR_RESOLUTION_OFF:         
		return "K4A_COLOR_RESOLUTION_OFF";
    case K4A_COLOR_RESOLUTION_720P:
		return "K4A_COLOR_RESOLUTION_720P";
    case K4A_COLOR_RESOLUTION_1080P:
		return "K4A_COLOR_RESOLUTION_1080P";
    case K4A_COLOR_RESOLUTION_1440P:
		return "K4A_COLOR_RESOLUTION_1440P";
    case K4A_COLOR_RESOLUTION_1536P:  
		return "K4A_COLOR_RESOLUTION_1536P";
    case K4A_COLOR_RESOLUTION_2160P:     
		return "K4A_COLOR_RESOLUTION_2160P";
    case K4A_COLOR_RESOLUTION_3072P:     
		return "K4A_COLOR_RESOLUTION_3072P";
	}
	return "K4A_COLOR_RESOLUTION_UMKNOWN";
} 

AzureKinectCamera::AzureKinectCamera()
    : playback_handle (NULL)
    , m_bDisplayImage (false)
    , m_bExtractColorImage (true)
    , roll (0)
    , pitch (0)
    , m_online (true)
    , initialized (false)
{
    // following this, call openDevice() to instantiate the camera object. 
}

AzureKinectCamera::AzureKinectCamera(string offline_filepath)
    : playback_handle (NULL)
    , m_bDisplayImage (false)
    , m_bExtractColorImage (false)
    , roll (0)
    , pitch (0)
    , initialized (false)    
{
    openDevice(offline_filepath);
}

AzureKinectCamera::~AzureKinectCamera()
{
    if(m_online && playback_handle != NULL)
	    k4a_playback_close(playback_handle);	
}

void AzureKinectCamera::openDevice(std::string offline_filepath)
{
    video_filename = offline_filepath;
    m_online = video_filename.empty();
    Initialize();
}

void AzureKinectCamera::Initialize()
{
    frame_id = 0;
    if(m_online) 
        initialize_camera();
    else
        initialize_video();    
}

void AzureKinectCamera::initialize_camera()
{
    const uint32_t deviceCount = k4a::device::get_installed_count();
    if (deviceCount == 0)
    {
        printf("No k4a devices attached!\n");
        throw std::runtime_error("No Azure Kinect devices detected!");
    }

    // Start the device
    //
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    // original code
    // config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    // config.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    // config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    // config.color_resolution = K4A_COLOR_RESOLUTION_720P;
    // changed to
    // {
    config.camera_fps = K4A_FRAMES_PER_SECOND_30;
    config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = K4A_COLOR_RESOLUTION_720P; //1280 x 720  16:9 //K4A_COLOR_RESOLUTION_OFF;//
    //config.color_resolution = K4A_COLOR_RESOLUTION_1536P; /**< 2048 * 1536 4:3  */

    // }

    // This means that we'll only get captures that have both color and
    // depth images, so we don't need to check if the capture contains
    // a particular type of image.
    //
    config.synchronized_images_only = true;

    std::cout << "Started opening K4A device..." << std::endl;

    dev = k4a::device::open(K4A_DEVICE_DEFAULT);
    dev.start_cameras(&config);
    dev.start_imu();

    std::cout << "Finished opening K4A device." << std::endl;

    calibration = dev.get_calibration(config.depth_mode, config.color_resolution);
    transformation = k4a::transformation(calibration);
  	initialized = true;
}

void AzureKinectCamera::initialize_video()
{
    std::cout << "openning recorded file: " << video_filename << "\n";
	if (k4a_playback_open(video_filename.c_str(), &playback_handle) != K4A_RESULT_SUCCEEDED)
	{
		std::cout << "Failed to open recorded file: " << video_filename << "\n";
		return;
	}
	uint64_t recording_length = k4a_playback_get_last_timestamp_usec(playback_handle);
	if(recording_length < 5000000)
    	printf("Recording is %.01f seconds long\n", 1.0 * recording_length / 1000000);
    else
    	printf("Recording is %.0f seconds long\n", 1.0 * recording_length / 1000000);

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
	k4a_playback_get_record_configuration(playback_handle, &config_video);
	std::cout << getK4A_FRAMES_PER_SECOND(config_video.camera_fps) 
	          << ", depth_mode=" << getK4A_DEPTH_MODE(config_video.depth_mode)
			  << ", color_format=" << getK4A_IMAGE_FORMAT(config_video.color_format)
			  << ", color_resolution=" << getK4A_COLOR_RESOLUTION(config_video.color_resolution) 
			  //<< ", synchronized_images_only=" << config.synchronized_images_only 
			  << ", imu_track_enabled=" << config_video.imu_track_enabled
			  << "\n"; 

	k4a_result_t result_get_cal = k4a_playback_get_calibration(playback_handle, &calibration_video);
	if(result_get_cal == K4A_RESULT_SUCCEEDED)
		std::cout << "Succeeded to get calibration\n";
	if(result_get_cal == K4A_RESULT_FAILED)
		std::cout << "Failed to get calibration\n";
	//* for unknown reason, not able to get transformation from the calibration even though getting calibration was succeessful.
	k4a_transformation_t tf = k4a_transformation_create(&calibration_video);
    //k4a::transformation transformation;
	transformation = k4a::transformation(calibration_video);
	// */
	initialized = true;
}
void AzureKinectCamera::convertDepthImageToMat(k4a_image_t &depthImage, cv::Mat &depthMat)
{
    if (depthImage != NULL)
    {
        // you can check the format with this function
        k4a_image_format_t format = k4a_image_get_format(depthImage); // K4A_IMAGE_FORMAT_DEPTH16
        if (format != K4A_IMAGE_FORMAT_DEPTH16)
            std::cerr << "Error on convertDepthImageToMat(): the input isn't a depth image\n";

        int rows = k4a_image_get_height_pixels(depthImage);
        int cols = k4a_image_get_width_pixels(depthImage);

        // get raw buffer
        uint8_t* buffer = k4a_image_get_buffer(depthImage);

        //convert to uint16_t to process the depth
        uint16_t *irImg = reinterpret_cast<uint16_t *>(buffer);
        uint16_t min_depth = 500; //mm
        uint16_t max_depth = 2500; //mm
        for (int i = 0; i < rows*cols; i++)
        {
            uint16_t &d = *irImg;
            if (d < min_depth)
                d = 0;
            else
                if (d > max_depth) d = max_depth - min_depth;
                else
                    d -= min_depth;
            irImg++;
        }

        // convert the raw buffer to cv::Mat
        depthMat = cv::Mat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
        //k4a_image_release(depthImage);
    }
}

void AzureKinectCamera::convertDepthImageToMat(const k4a::image &depthImage, cv::Mat &depthMat)
{
    if (depthImage.handle() != NULL)
    {
        // you can check the format with this function
        k4a_image_format_t format = depthImage.get_format(); // K4A_IMAGE_FORMAT_DEPTH16
        if (format != K4A_IMAGE_FORMAT_DEPTH16)
            std::cerr << "Error on convertDepthImageToMat(): the input isn't a depth image\n";

        int rows = depthImage.get_height_pixels();
        int cols = depthImage.get_width_pixels();

        // get raw buffer
        //const uint8_t* buffer = depthImage.get_buffer();
        uint8_t* buffer = k4a_image_get_buffer(depthImage.handle());
        //convert to uint16_t to process the depth
        uint16_t *irImg = reinterpret_cast<uint16_t *>(buffer);
        std::vector<uint8_t> grayScaleImg(rows * cols);
        uint16_t min_depth = 500; //mm
        uint16_t max_depth = 2500; //mm
        ConvertToGrayScaleImage(irImg, cols * rows, min_depth, max_depth, grayScaleImg.data());
        uint8_t *im = grayScaleImg.data();

        // convert the raw buffer to cv::Mat
        //depthMat = cv::Mat(rows, cols, CV_16U, (void*)buffer, cv::Mat::AUTO_STEP);
        depthMat = cv::Mat(rows, cols, CV_8U, (void*)im, cv::Mat::AUTO_STEP);
    }
}

void AzureKinectCamera::convertDepthImageToMatRGB(const k4a::image &depthImage, cv::Mat &depthMat)
{
    if (depthImage.handle() != NULL)
    {
        // you can check the format with this function
        k4a_image_format_t format = depthImage.get_format(); // K4A_IMAGE_FORMAT_DEPTH16
        if (format != K4A_IMAGE_FORMAT_DEPTH16)
            std::cerr << "Error on convertDepthImageToMat(): the input isn't a depth image\n";

        int rows = depthImage.get_height_pixels();
        int cols = depthImage.get_width_pixels();

        // get raw buffer
        //const uint8_t* buffer = depthImage.get_buffer();
        uint8_t* buffer = k4a_image_get_buffer(depthImage.handle());
        depthMat.create(rows, cols, CV_8UC3);
        for (int row = 0; row < rows; row++)
        {
            cv::Vec3b *ptrMat = depthMat.ptr<cv::Vec3b>(row);
            int offset = row*cols * 2;
            for (int col = 0; col < cols; col++)
            {
                cv::Vec3b color;
                color[0] = buffer[offset + col * 2];
                color[1] = buffer[offset + col * 2 + 1];
                color[2] = 0;
                ptrMat[col] = color;
            }
        }
    }
}

void AzureKinectCamera::displayDepthImage(cv::Mat &depthImage_16U)
{
    //depthImage_16U
    cv::imshow("DepthImage", depthImage_16U);
}

CvScalar AzureKinectCamera::blendTwoColors(CvScalar color1, CvScalar color2, float weight1)
{
    CvScalar color;
    for(int i=0; i < 4; i++)
        color.val[i] = color1.val[i] * weight1 + color2.val[i] * (1.0 - weight1);
    return color;
}

/*
mm_3D_point_t getXyzFromPointCloudImage(k4a::image point_cloud_image, int col, int row)
{
    int width = point_cloud_image.get_width_pixels();
    int heigh = point_cloud_image.get_height_pixels();
    int16_t *point_cloud_image_data = (int16_t *)(void *)k4a_image_get_buffer(point_cloud_image.handle());
    int i = row * width + col;
    mm_3D_point_t point;
    point.xyz[0] = point_cloud_image_data[3 * i + 0];
    point.xyz[1] = point_cloud_image_data[3 * i + 1];
    point.xyz[2] = point_cloud_image_data[3 * i + 2];
    return point;
}
*/

/*
void AzureKinectCamera::RegistrateClickEvent(registration_request slot)
{
    //signalHandler.connect( boost::bind( &class::handleSignal, &class_object, _1) );
    signalHandler.connect( slot );      // Defined above
}

void AzureKinectCamera::RegistratePrimaryHandFoundEvent(primary_hand_found_event_slot slot)
{
    //primary_hand_found_event.connect( boost::bind( &class::handleSignal, &class_object, _1) );
    primary_hand_found_event.connect( slot );      // Defined above
}

void AzureKinectCamera::RegistratePrimaryHandLostEvent(primary_hand_lost_event_slot slot)
{
    //primary_hand_found_event.connect( boost::bind( &class::handleSignal, &class_object, _1) );
    primary_hand_lost_event.connect( slot );      // Defined above
}
*/
void AzureKinectCamera::stop()
{
    running = false;
}

void AzureKinectCamera::run()
{
    if(!initialized)
    {
        std::cout << "Error: camera or video is not intialized properly\n";
        return;
    }
    running = true;
    if(m_online)
        run_camera();
    else
        run_video();
}

void AzureKinectCamera::run_camera()
{
    while (running)
    {
        cv::Mat mat8UC1; // ir image
        k4a::image point_cloud_image = NULL;

        k4a::capture capture;
        k4a_imu_sample_t imu;
        dev.get_imu_sample(&imu, std::chrono::milliseconds(0));
        getRollPitch(imu, roll, pitch);
        if (dev.get_capture(&capture, std::chrono::milliseconds(0)))
        {
            const k4a::image depthImage = capture.get_depth_image();
            const k4a::image colorImage = capture.get_color_image();
            const k4a::image infraredImage = capture.get_ir_image();
            cv::Mat depthMat;
            convertDepthImageToMat(depthImage, depthMat);
            //convertDepthImageToMatRGB(depthImage, depthMat);

            //cv::Mat depthMat2 = k4a::get_mat(depthImage);
            //depthMat2.convertTo(CV_8U, -255/5000.0, 255);
            int depth_image_width_pixels = depthImage.get_width_pixels();
            int depth_image_height_pixels = depthImage.get_height_pixels();

            point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   depth_image_width_pixels,
                                                   depth_image_height_pixels,
                                                   depth_image_width_pixels * 3 * (int)sizeof(int16_t));//stride_bytes
            transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

            // Access the ir image
            int width = infraredImage.get_width_pixels();
            int height = infraredImage.get_height_pixels();
            int strides = infraredImage.get_stride_bytes();
            int rows = height;
            int cols = width;
            static bool run_once = true;
            if (run_once)
            {
                printf(" | IR16 res:%4dx%4d stride:%5d\n", height, width, strides);
                run_once = false;
            }

            elapse_time += fps_watcher.NotifyFrame();

            // get raw buffer
            //uint8_t* buffer = infraredImage.get_buffer(); //This doesn't allow.
            uint8_t* buffer = k4a_image_get_buffer(infraredImage.handle());
            uint16_t *irImg = reinterpret_cast<uint16_t *>(buffer);

            //
            std::vector<uint8_t> grayScaleImg(width * height);
            const int irMinValue = 0;
            const int irMaxValue = 1000;
            ConvertToGrayScaleImage(irImg, strides / 2 * height, irMinValue, irMaxValue, grayScaleImg.data());
            uint8_t *im = grayScaleImg.data();
            //cv::Mat mat8UC1(cv::Size(width, height), CV_8UC1, im);
            mat8UC1 = cv::Mat(cv::Size(width, height), CV_8UC1, im);
            data_event(roll, pitch, mat8UC1, depthImage, point_cloud_image, colorImage);

            if(m_bDisplayImage)
                cv::imshow("Infrared Image", mat8UC1);

            std::cout << "\nCamera FPS = " << fps_watcher.GetFramerate() << std::endl;

            //k4a_image_release(image_ir);
            if(m_bDisplayImage)
            {
                int key = cvWaitKey(1);
                if (key == 27) // exit if ESC key is pressed
                running = false;
            }
        }
        else
        {
            printf("Failed to get frames\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
    if (dev)
    {
        transformation.destroy();
        dev.stop_cameras();
        dev.stop_imu();
        dev.close();
    }
}

void AzureKinectCamera::run_video()
{
    //play_one_frame_from_video(true);
    run_video_old();
}

void AzureKinectCamera::run_video_old()
{
   	k4a_capture_t capture = NULL;
    running = true;

	k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
	std::cout << "Starting to read frames...\n";
    frame_id = 0;
	while (result == K4A_STREAM_RESULT_SUCCEEDED && running)
	{
        cv::Mat mat8UC1;
        k4a::image depth_image;
        k4a::image point_cloud_image;
        k4a::image color_image;

		k4a_imu_sample_t imu;
		switch(k4a_playback_get_next_imu_sample(playback_handle, &imu))
		{
		case K4A_STREAM_RESULT_SUCCEEDED:
			std::cout << "imu.acc_sample.xyz: " << imu.acc_sample.xyz.x << ", " << imu.acc_sample.xyz.y << ", " <<  imu.acc_sample.xyz.z << "\n";
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
			depth_image = cap.get_depth_image();

			int rows = depth_image.get_height_pixels();
			int cols = depth_image.get_width_pixels();
			std::cout << "depth image rows: " << rows << "\n";
			std::cout << "depth image cols: " << cols << "\n";
			std::cout << "depth image size: " << depth_image.get_size() << "\n";
			std::cout << "Timestamp: " << depth_image.get_device_timestamp().count() << "\n";
			cv::Mat depthMat(rows, cols, CV_16UC1, (void *) depth_image.get_buffer(), cv::Mat::AUTO_STEP);
            if(m_bDisplayImage)
			    cv::imshow("Depth Image", depthMat);

            int depth_image_width_pixels = depth_image.get_width_pixels();
            int depth_image_height_pixels = depth_image.get_height_pixels();

            point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                   depth_image_width_pixels,
                                                   depth_image_height_pixels,
                                                   depth_image_width_pixels * 3 * (int)sizeof(int16_t));//stride_bytes
            transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

			k4a::image ir_img = cap.get_ir_image();
            cv::Mat irMat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
			convertIrImageToMat(ir_img, irMat);
            mat8UC1 = irMat.clone();

			// mat8UC1 = cv::Mat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
			// convertIrImageToMat(ir_img, mat8UC1);

            if(m_bDisplayImage)
			    cv::imshow("Infrared Image", mat8UC1);
            if(m_bExtractColorImage)
            {                
                color_image = cap.get_color_image();
                if(color_image){
                    if(m_bDisplayImage)
                    {
                        int rows_color_image = color_image.get_height_pixels();
                        int cols_color_image = color_image.get_width_pixels();
                        std::cout << "color image size: " << color_image.get_size() << "\n";
                        std::cout << "Timestamp: " << color_image.get_device_timestamp().count() << "\n";
                        std::cout << "width: " << cols_color_image << "\n";
                        std::cout << "heigth: " << rows_color_image << "\n";
                        cv::Mat rawData( 1, color_image.get_size(), CV_8UC1, (void*)color_image.get_buffer() );
                        cv::Mat decodedImage  =  cv::imdecode( rawData,  cv::IMREAD_UNCHANGED );
                        cv::imshow("RGB Image",decodedImage);
                    }
                }			
            }            
            if(m_bDisplayImage)
			    cv::waitKey(1);
			//k4a_capture_release(capture); //cap is local so it will take care of the release of capture
		}
		else if (result == K4A_STREAM_RESULT_EOF)
		{
			// End of file reached
			std::cout << "Reach end of stream\n";
			break;
		}
        elapse_time += fps_watcher.NotifyFrame();
        //std::cout << "\n" << "Video: Frame# " << frame_id << ", FPS = " << fps_watcher.GetFramerate() << std::endl;
        std::cout << "\n" << "Video: FPS = " << fps_watcher.GetFramerate() << std::endl;

        data_event(roll, pitch, mat8UC1, depth_image, point_cloud_image, color_image);
        //frame_id++;
	}
	if (result == K4A_STREAM_RESULT_FAILED)
	{
		printf("Failed to read entire recording\n");
	}

	k4a_playback_close(playback_handle);	
}

void AzureKinectCamera::play_one_frame_from_video(bool nextFrame)
{
    if(!running) return;
   	k4a_capture_t capture = NULL;

	k4a_stream_result_t result = K4A_STREAM_RESULT_SUCCEEDED;
    if(nextFrame)    
        frame_id++;
    else
        frame_id--;
	std::cout << "------------------------------------------------------------\n";
	std::cout << "play_one_frame_from_video, frame_id=" << frame_id << "\n";

    cv::Mat mat8UC1;
    k4a::image depth_image;
    k4a::image point_cloud_image;
    k4a::image color_image;

    k4a_imu_sample_t imu;
    k4a_stream_result_t result_imu = K4A_STREAM_RESULT_FAILED;
    if(nextFrame)
        result_imu = k4a_playback_get_next_imu_sample(playback_handle, &imu);
    else
        result_imu = k4a_playback_get_previous_imu_sample(playback_handle, &imu);
    switch(result_imu)
    {
    case K4A_STREAM_RESULT_SUCCEEDED:
        getRollPitch(imu, roll, pitch);
        //std::cout << "imu.acc_sample.xyz: " << imu.acc_sample.xyz.x << ", " << imu.acc_sample.xyz.y << ", " <<  imu.acc_sample.xyz.z << "\n";
        //std::cout << "roll: " << roll*rad_to_deg << "deg\n";
        //std::cout << "pitch: " << pitch*rad_to_deg  << "deg\n";
        break;
    case K4A_STREAM_RESULT_FAILED:
        std::cout << "Failed to read imu data\n";
        break;
    case K4A_STREAM_RESULT_EOF:
        std::cout << "Reach end of imu data\n";
        break;
    }
    if(nextFrame)
        result = k4a_playback_get_next_capture(playback_handle, &capture);
    else
        result = k4a_playback_get_previous_capture(playback_handle, &capture);
    if (result == K4A_STREAM_RESULT_SUCCEEDED)
    {
        // Process capture here
        k4a::capture cap(capture);
        depth_image = cap.get_depth_image();

        int rows = depth_image.get_height_pixels();
        int cols = depth_image.get_width_pixels();
        // std::cout << "depth image rows: " << rows << "\n";
        // std::cout << "depth image cols: " << cols << "\n";
        // std::cout << "depth image size: " << depth_image.get_size() << "\n";
        std::cout << "Timestamp: " << depth_image.get_device_timestamp().count() << "\n";
        cv::Mat depthMat(rows, cols, CV_16UC1, (void *) depth_image.get_buffer(), cv::Mat::AUTO_STEP);
        if(m_bDisplayImage)
            cv::imshow("Depth Image", depthMat);

        int depth_image_width_pixels = depth_image.get_width_pixels();
        int depth_image_height_pixels = depth_image.get_height_pixels();

        point_cloud_image = k4a::image::create(K4A_IMAGE_FORMAT_CUSTOM,
                                                depth_image_width_pixels,
                                                depth_image_height_pixels,
                                                depth_image_width_pixels * 3 * (int)sizeof(int16_t));//stride_bytes
        transformation.depth_image_to_point_cloud(depth_image, K4A_CALIBRATION_TYPE_DEPTH, &point_cloud_image);

        k4a::image ir_img = cap.get_ir_image();
        cv::Mat irMat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
        convertIrImageToMat(ir_img, irMat);
        mat8UC1 = irMat.clone();

        // mat8UC1 = cv::Mat(rows, cols, CV_16UC1, (void *) ir_img.get_buffer(), cv::Mat::AUTO_STEP);
        // convertIrImageToMat(ir_img, mat8UC1);

        if(m_bDisplayImage)
            cv::imshow("Infrared Image", mat8UC1);
        if(m_bExtractColorImage)
        {                
            color_image = cap.get_color_image();
            if(color_image){
                if(m_bDisplayImage)
                {
                    int rows_color_image = color_image.get_height_pixels();
                    int cols_color_image = color_image.get_width_pixels();
                    std::cout << "color image size: " << color_image.get_size() << "\n";
                    std::cout << "Timestamp: " << color_image.get_device_timestamp().count() << "\n";
                    std::cout << "width: " << cols_color_image << "\n";
                    std::cout << "heigth: " << rows_color_image << "\n";
                    cv::Mat rawData( 1, color_image.get_size(), CV_8UC1, (void*)color_image.get_buffer() );
                    cv::Mat decodedImage  =  cv::imdecode( rawData,  cv::IMREAD_UNCHANGED );
                    cv::imshow("RGB Image",decodedImage);
                }
            }			
        }            
        if(m_bDisplayImage)
            cv::waitKey(1);
        //k4a_capture_release(capture); //cap is local so it will take care of the release of capture
    }
    else if (result == K4A_STREAM_RESULT_EOF)
    {
        // End of file reached
        std::cout << "Reach end of stream\n";
    }
    elapse_time += fps_watcher.NotifyFrame();
    //std::cout << "\n" << "Video: Frame# " << frame_id << ", FPS = " << fps_watcher.GetFramerate() << std::endl;
    std::cout << "\n" << "Video: FPS = " << fps_watcher.GetFramerate() << std::endl;

    if (result == K4A_STREAM_RESULT_SUCCEEDED)
        data_event(roll, pitch, mat8UC1, depth_image, point_cloud_image, color_image);

	if (result == K4A_STREAM_RESULT_FAILED)
	{
		printf("Failed to read entire recording\n");
	}
}

void AzureKinectCamera::RegistrateDataEvent(data_event_slot slot)
{
    data_event.connect( slot );
}
