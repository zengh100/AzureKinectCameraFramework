# Azure Kinect Camera Framework for OpenCV
A framework to read, display (in OpenCV), and processing sensor data (imu, infrared, depth, 3D point cloud, and color image) from Microsoft Azure Kinect camera.
```
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

```
