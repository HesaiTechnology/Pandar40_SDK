#include "hesaiLidarSDK.h"

int imageNo = 0;
unsigned long imageNoForSave = 0;
unsigned int modnum = 0;
int lidarNo = 0;
unsigned long lidarNoForSave = 0;
FILE* cameraTimestampFile = fopen("camera-timestamp.txt", "w");
FILE* lidarTimestampFile = fopen("lidar-timestamp.txt", "w");
double pandoraToSysTimeGap = 0;

void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  // Mat myMat = imread("t.png");
  // imshow("camera", myMat);
  // std::cout<<myMat<<std::endl;
  // if(++imageNo > 50  && modnum < 4)
  // { 
  //   ++modnum;
  //   cv::imwrite(boost::to_string(++imageNoForSave) + "-" + boost::to_string(pic_id) + ".jpg", *matp);
  //   if (modnum == 4)
  //   {
  //     imageNo = 0;
  //     modnum = 0;
  //   }
  // }
  // if(pic_id == 0)
  // {
  //   cv::imwrite(boost::to_string(++imageNoForSave) + "-" + boost::to_string(pic_id) + ".bmp", *matp);
  // }
  struct timeval ts;
  gettimeofday(&ts, NULL);
  // fprintf(cameraTimestampFile, "%d,%f\n", pic_id, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
  fprintf(cameraTimestampFile, "%d,%lf,%lf\n", pic_id, timestamp, (double)ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
  // printf("%d , %f , %lf , %f \n" ,pic_id , timestamp , (double)ts.tv_sec + (double)ts.tv_usec / 1000000 ,pandoraToSysTimeGap);
  // if (pic_id != 0)
  // {
  //   fprintf(cameraTimestampFile, "%d,%f\n", pic_id, timestamp);
  //   fflush(cameraTimestampFile);
  // }
  // printf("cameraid: %d, timestamp: %lf\n", pic_id, timestamp);
}


void gpsCallback(int timestamp)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  pandoraToSysTimeGap = (double)ts.tv_sec + ( (double)ts.tv_usec / 1000000.0  ) - (double)timestamp;
  // printf("gps: %d, gap: %f\n", timestamp, pandoraToSysTimeGap);
}

void cameraCallbackForDelay(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  fprintf(cameraTimestampFile, "%d,%f\n", pic_id, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - timestamp);
  // fflush(cameraTimestampFile);
}

void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp)
{
  struct timeval ts;
  gettimeofday(&ts, NULL);
  fprintf(lidarTimestampFile, "%f,%f\n", cld->points[0].timestamp, ts.tv_sec + (double)ts.tv_usec / 1000000  -  pandoraToSysTimeGap - cld->points[0].timestamp);
  // if (++lidarNo == 100)
  // {
  //   char pcdFileName[256];
  //   sprintf(pcdFileName, "%d.pcd", ++lidarNoForSave);
  //   pcl::io::savePCDFileASCII(pcdFileName, *cld);
  //   lidarNo = 0;
  // }

  // printf("lidar: %lf\n",cld->points[0].timestamp);
  // for(int i = 0; i < cld->points.size();i++)
  //   printf("i: %f\n", cld->points[i].intensity);
}

int main(int argc, char **argv)
{
  // PandoraSDK psdk(std::string("172.31.2.165"), 9870, 8080, 10110, 0,
  // std::string("intrinsic.yaml"), std::string("correction.csv"), 
  // cameraCallback, lidarCallback, gpsCallback);
  HesaiLidarSDK psdk(std::string("192.168.20.51"), 9870, 2368, 10110, 0,
    std::string("calibration.yml"), std::string("correction.csv"),
    cameraCallback, lidarCallback, gpsCallback,
    1, 40, 0);

  // HesaiLidarSDK psdk(2368, 10110, std::string("correction.csv"), lidarCallback, gpsCallback, 1, 40, 0);
  // PandoraSDK psdk(std::string("192.168.20.51"), 9870, cameraCallbackForDelay, lidarCallback);
  // HesaiLidarSDK psdk(std::string("/media/yy/Data/pcap/alibaba/lane_line.pcap"), std::string(""), 0, 40, 0, lidarCallback);
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