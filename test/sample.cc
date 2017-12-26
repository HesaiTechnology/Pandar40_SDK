#include "pandoraSDK.h"

int imageNo = 0;
unsigned long imageNoForSave = 0;
unsigned int modnum = 0;
int lidarNo = 0;
unsigned long lidarNoForSave = 0;
double pandoraToSysTimeGap = 0;

void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{

  if(++imageNo > 50  && modnum < 4)
  { 
    ++modnum;
    cv::imwrite(boost::to_string(++imageNoForSave) + "-" + boost::to_string(pic_id) + ".jpg", *matp);
    if (modnum == 4)
    {
      imageNo = 0;
      modnum = 0;
    }
  }

}

void gpsCallback(unsigned int timestamp)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  pandoraToSysTimeGap = ts.tv_sec + (double)ts.tv_usec / 1000000 - timestamp;
  printf("gap: %f\n", pandoraToSysTimeGap);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{

  if (++lidarNo == 100)
  {
    char pcdFileName[256];
    sprintf(pcdFileName, "%d.pcd", ++lidarNoForSave);
    pcl::io::savePCDFileASCII(pcdFileName, *cld);
    lidarNo = 0;
  }

}

int main(int argc, char **argv)
{

  PandoraSDK psdk(std::string("192.168.20.51"), 9870, 8080, 10110, 0, std::string(""), std::string(""), cameraCallback, lidarCallback, gpsCallback);
  // PandoraSDK psdk(std::string("192.168.20.51"), 9870, cameraCallback, lidarCallback); //use default udpport, gpsport, intrinsic, start_angle, correction
  psdk.start();
  while(true)
  {
    sleep(100);
  }

}