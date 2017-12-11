#include "pandoraSDK.h"

int imageNo = 0;
int imageNoForSave = 0;
int lidarNo = 0;
int lidarNoForSave = 0;
FILE* cameraTimestampFile = fopen("camera-timestamp.txt", "w");
FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");

void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  // Mat myMat = imread("t.png");
  // imshow("camera", myMat);
  // std::cout<<myMat<<std::endl;
  if(++imageNo == 10000)
  {
    cv::imwrite(boost::to_string(++imageNoForSave) + ".jpg", *matp);
    imageNo = 0;
  }
  // std::cout<<"camera: "<<timestamp<<", pic: "<< pic_id<<std::endl;
  if (pic_id == 0)
  {
    fprintf(cameraTimestampFile, "%d,%f\n", pic_id, timestamp);
    // fflush(cameraTimestampFile);
  }
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
  // fprintf(lidarTimestampFile, "%f\n", timestamp);
  // fflush(lidarTimestampFile);
  if (++lidarNo == 100)
  {
    char pcdFileName[256];
    sprintf(pcdFileName, "%d.pcd", ++lidarNoForSave);
    pcl::io::savePCDFileASCII(pcdFileName, *cld);
    lidarNo = 0;
  }

  // std::cout<<"lidar: "<<timestamp<<std::endl;
}



int main(int argc, char **argv)
{
  // PandoraSDK psdk(std::string("172.31.2.165"), 9870, 8080, 0, std::string("intrinsic.yaml"), std::string("correction.csv"), cameraCallback, lidarCallback);
  // PandoraSDK psdk(std::string("172.31.2.165"), 9870, 8080, 0, std::string(""), std::string(""), cameraCallback, lidarCallback);
  PandoraSDK psdk(std::string("172.31.2.165"), 9870, cameraCallback, lidarCallback);
  psdk.start();
  while(true)
  {
    sleep(100);
  }
  // while(true)
  // {
  //   sleep(5);
  //   printf("stop\n");
  //   psdk.stop();
  //   sleep(5);
  //   printf("start\n");
  //   psdk.start();
  // }
}