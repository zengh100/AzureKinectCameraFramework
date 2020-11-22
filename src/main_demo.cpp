#include <thread>
#include "test.hpp"

int main(int argc, char** argv)
{
    //TestVideoOpen();
    std::string video_filename = ""; //empty will open camera
    if(argc > 1)
        video_filename = argv[1];
    TestClass_AzureKinectCamera(video_filename);
	std::cout << "Program exited\n";
    return 0;
}
