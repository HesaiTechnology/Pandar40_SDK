#include "pandoraSDK.h"

#include <opencv2/highgui/highgui.hpp>

class PandoraCallback
{
public:
  PandoraCallback()
  {
  }

  void cameraCallback(cv::Mat mat, double timestamp, int pic_id)
  {
  }

  void lidarCallback(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)
  {
  }
private:
};




void cameraCallback(cv::Mat mat, double timestamp, int pic_id)
{

}

void lidarCallback(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)
{
}


int main(int argc, char **argv)
{
  // PandoraCallback pback;
  boost::function<void(cv::Mat, double timestamp, int pic_id)> cb = &cameraCallback;
  boost::function<void(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)> lb = &lidarCallback;
  PandoraSDK psdk(std::string("172.31.3.165"), 9870, 8080, 0, std::string("intrinsic.ymal"), std::string("extrinsic.ymal"), std::string("correction.csv"), cb, lb);
  psdk.start();
}