#include "pandoraSDK.h"

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


// cv::namedWindow("camera");
int imageNo = 0;
FILE* cameraTimestampFile = fopen("camera-timestamp.txt", "w");

void cameraCallback(cv::Mat mat, double timestamp, int pic_id)
{
  // Mat myMat = imread("t.png");
  // imshow("camera", myMat);
  // std::cout<<myMat<<std::endl;
  // cv::imwrite(boost::to_string(++imageNo) + ".jpg", mat);
  std::cout<<"camera: "<<timestamp<<", pic: "<< pic_id<<std::endl;
  fprintf(cameraTimestampFile, "%d,%f\n", pic_id, timestamp);
  fflush(cameraTimestampFile);
}

int lidarPacketCounter = 0;
void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
  // char pcdFileName[256];
  // sprintf(pcdFileName, "%d.pcd", ++lidarPacketCounter);
  // pcl::io::savePCDFileASCII(pcdFileName, *cld);
  std::cout<<"lidar: "<<timestamp<<std::endl;
}



int main(int argc, char **argv)
{
  PandoraSDK psdk(std::string("172.31.2.165"), 9870, 8080, 0, std::string("intrinsic.yaml"), std::string("extrinsic.yaml"), std::string("correction.csv"), cameraCallback, lidarCallback);
  psdk.start();
  while(true)
  {
    sleep(100);
  }
}