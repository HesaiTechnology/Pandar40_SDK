#ifndef __PANDORA_SDK_IN_H
#define __PANDORA_SDK_IN_H

#include <pthread.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <semaphore.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui_c.h>
#include "pandora_client.h"
#include "input.h"
#include "rawdata.h"


using namespace cv;

typedef struct HS_LIDAR_L40_GPS_PACKET_s{
    unsigned short flag;
    unsigned short year;
    unsigned short month;
    unsigned short day;
    unsigned short second;
    unsigned short minute;
    unsigned short hour;
    unsigned int fineTime;
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

#define PandoraSDK_DEFAULT_LIDAR_RECV_PORT 8080
#define PandoraSDK_DEFAULT_GPS_RECV_PORT 10110

#define PandoraSDK_DEFAULT_START_ANGLE 0.0
#define PandoraSDK_CAMERA_NUM  5
#define PandoraSDK_IMAGE_WIDTH 1280
#define PandoraSDK_IMAGE_HEIGHT 720
// 10000 us  = 100 ms = 0.1s per packet
#define PandoraSDK_PCAP_TIME_INTERVAL 100000


enum PandoraUseMode
{
	PandoraUseMode_onlyLidar, 		// 仅使用lidar
	PandoraUseMode_readPcap, 			// 仅使用lidar读取pcap
	PandoraUseMode_lidarAndCamera //使用lidar和camera
};


class PandoraSDK_internal
{
public:
	PandoraSDK_internal(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		const unsigned short lidarRecvPort,
		const unsigned short gpsRecvPort,
		const double startAngle,
		const std::string intrinsicFile,
		const std::string lidarCorrectionFile,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
		boost::function<void(double timestamp)> gpsCallback);
	
	PandoraSDK_internal(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	
	PandoraSDK_internal(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const double startAngle,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);

	~PandoraSDK_internal();
	int start();
	void stop();
	void pushPicture(PandoraPic *pic);

private:
	void init(	
			const std::string pandoraIP,
			const unsigned short pandoraCameraPort,
			const unsigned short lidarRecvPort,
			const unsigned short gpsRecvPort,
			const double startAngle,
			const std::string intrinsicFile,
			const std::string lidarCorrectionFile,
			boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
			boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
			boost::function<void(double timestamp)> gpsCallback);
	void setupCameraClient();
	void setupLidarClient();
	void setupReadPcap();
	void lidarRecvTask();
	void readPcapFile();
	void processGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);
	void processLiarPacket();
	void processPic();
	void pushLiDARData(PandarPacket packet);
	bool loadIntrinsics(const std::string& intrinsicFile);

	PandoraUseMode useMode;
	pthread_mutex_t lidarLock, picLock, lidarGpsLock, cameraGpsLock;
  sem_t lidarSem, picSem;
	boost::thread lidarRecvThread;
	boost::thread lidarProcessThread;
	boost::thread processPicThread;
	boost::thread readPacpThread;
	bool continueLidarRecvThread;
	bool continueProcessLidarPacket;
	bool continueProcessPic;
	bool continueReadPcap;
	bool needRemapPicMat;
	std::string ip;
	unsigned short cport;
	unsigned short lport;
	int lidarRotationStartAngle;
	void* pandoraCameraClient;
	HS_LIDAR_L40_GPS_Packet hesaiGps;

  std::list<PandarPacket> lidarPacketList;
	std::list<PandoraPic*> pictureList;

	std::vector<Mat> mapxList;
  std::vector<Mat> mapyList;
	boost::shared_ptr<pandar_pointcloud::Input> input;
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> userCameraCallback;
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> userLidarCallback;
	boost::function<void(double timestamp)> userGpsCallback;
	boost::shared_ptr<pandar_rawdata::RawData> data_;
	time_t gps1;
  	GPS_STRUCT_T gps2;
  	time_t gps1Cam[PandoraSDK_CAMERA_NUM];
	GPS_STRUCT_T gps2Cam[PandoraSDK_CAMERA_NUM];
	unsigned int lidarLastGPSSecond;
	unsigned int cameraLastGPSSecond;
	unsigned int cameraTimestamp[PandoraSDK_CAMERA_NUM];
};

#endif