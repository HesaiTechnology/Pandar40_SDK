# Build
**only supports Ubuntu14 platform**
### 1. [install opencv](https://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html)

### 2. install dependencies:
```
sudo apt-get install libboost-all-dev libyaml-cpp-dev
```
### 3. install pcl:
```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```
### 4.
```
mkdir build
cd build
cmake ..
make
```

# Usage
### sample:
```
#include "pandoraSDK.h"

void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
    // do something with matp
    // cv::imwrite("test.jpg", *matp);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
    // do something with cld
    // pcl::io::savePCDFileASCII("test.pcd", *cld);
}



int main(int argc, char **argv)
{
    PandoraSDK psdk(std::string("172.31.2.165"), 9870, 8080, 0, std::string("intrinsic.yaml"), std::string("correction.csv"), cameraCallback, lidarCallback);
    // PandoraSDK psdk(std::string("172.31.2.165"), 9870, cameraCallback, lidarCallback); // use default port, intrinsic, correction
    psdk.start(); // start task
    sleep(5);
    psdk.stop();  //  stop task
    sleep(5);
    psdk.start(); // start task again
    while(true)
    {
        sleep(100);
    }
```