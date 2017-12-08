#ifndef __PANDORA_SDK_H
#define __PANDORA_SDK_H

#include <string>
#include <boost/function.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pthread.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <semaphore.h>

#include "yaml-cpp/yaml.h"

#include "pandora_client.h"
#include "point_types.h"
#include "input.h"
#include "rawdata.h"

using namespace cv;


typedef struct gps_s_
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
}gps_s;

typedef struct HS_LIDAR_L40_GPS_PACKET_s{
    unsigned short flag;
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short second;
    unsigned short minute;
    unsigned short hour;
    unsigned int fineTime;
//    unsigned char unused[496];
}HS_LIDAR_L40_GPS_Packet;

#define HS_LIDAR_L40_GPS_PACKET_SIZE (512)
#define HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_DAY_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE (2)
#define HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE (2)
#define HS_LIDAR_L40_GPS_ITEM_NUM (7)


const int CAMERA_NUM = 5;


class PandoraSDK
{
public:
	PandoraSDK(std::string pandoraIP,
						 int cameraPort,
						 int lidarRecvPort,
						 int startAngle,
						 std::string intrinsicFile,
						 std::string extrinsicFile,
						 std::string lidarCorrectionFile,
						 boost::function<void(cv::Mat, double timestamp, int pic_id)> cameraCallback,
						 boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	~PandoraSDK()
	{

	};
	int start();
	void stop();
	void destoryLidarThread();
	void setupCameraClient();
	void setupLidarClient();
	void lidarTask();
	void loadIntrinsics(std::string& intrinsicFile);
	void cameraProcessGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);
	void lidarProcessGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);
	void processLiarPacket();
	void pushLiDARData(PandarPacket packet);
	double getCameraLastGPSSecond();

	std::vector<Mat> getMapxList();
	std::vector<Mat> getMapyList();
	void userCameraCallback(cv::Mat mat, double timestamp, int pic_id);

private:
	pthread_t lidarThread;
	boost::thread lidarRecvThread;
	boost::thread lidarProcessThread;
	bool continueLidarThread;
	bool continueProcessLidarPacket;
	pthread_mutex_t gpsLock;
	std::string ip;
	int cport;
	int lport;
	int lidarRotationStartAngle;
	void* pandoraCameraClient;
	HS_LIDAR_L40_GPS_Packet hesaiGps;

	unsigned int lastGPSSecond;
	// std::vector<Mat> mapxList;
  // std::vector<Mat> mapyList;
	boost::shared_ptr<pandar_pointcloud::Input> input;
	// pandar_pointcloud::InputSocket input;
	boost::function<void(cv::Mat, double timestamp, int pic_id)> userCameraCallbackPara;
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> userLidarCallbackPara;
	// boost::shared_ptr<pandar_rawdata::RawData> data_;
	boost::shared_ptr<pandar_rawdata::RawData> data_;
	unsigned int cameraTimestamp[CAMERA_NUM];
	time_t gps1;
  gps_struct_t gps2;
  time_t gps1Cam[CAMERA_NUM];
	gps_struct_t gps2Cam[CAMERA_NUM];
	double lidarLastGPSSecond;
	double cameraLastGPSSecond;
	double localGPSOffset;
	pthread_mutex_t lidarLock;
  sem_t lidarSem;
  std::list<PandarPacket> LiDARDataSet;
};

#endif