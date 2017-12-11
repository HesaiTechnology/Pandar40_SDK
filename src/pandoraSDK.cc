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
#include "input.h"
#include "rawdata.h"
#include "pandoraSDK.h"
#include "utilities.cc"

using namespace cv;

typedef struct GPS_S_
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
}PS_GPS_T;

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
#define PandoraSDK_DEFAULT_START_ANGLE 0.0
#define PandoraSDK_CAMERA_NUM  5
#define PandoraSDK_IMAGE_WIDTH 1280
#define PandoraSDK_IMAGE_HEIGHT 720

Size PandoraSDK_IMAGE_SIZE(PandoraSDK_IMAGE_WIDTH, PandoraSDK_IMAGE_HEIGHT);

int HS_L40_GPS_Parse(HS_LIDAR_L40_GPS_Packet *packet, const unsigned char *recvbuf)
{
	int index = 0;
	packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
	index += HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE;
	packet->year = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE;
	packet->month = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE;
	packet->day = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_DAY_SIZE;
	packet->second = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE;
	packet->minute = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE;
	packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10 + 8;
	index += HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE;
	packet->fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
										 ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
	return 0;
}

class PandoraSDK_intern
{
public:
	PandoraSDK_intern(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		const unsigned short lidarRecvPort,
		const double startAngle,
		const std::string intrinsicFile,
		const std::string lidarCorrectionFile,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	
	PandoraSDK_intern(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	
	~PandoraSDK_intern();
	int start();
	void stop();
	void pushPicture(PandoraPic *pic);

private:
	void init(	
			const std::string pandoraIP,
			const unsigned short pandoraCameraPort,
			const unsigned short lidarRecvPort,
			const double startAngle,
			const std::string intrinsicFile,
			const std::string lidarCorrectionFile,
			boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
			boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback);
	void setupCameraClient();
	void setupLidarClient();
	void lidarRecvTask();
	void lidarProcessGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);
	void processLiarPacket();
	void processPic();
	void pushLiDARData(PandarPacket packet);
	bool loadIntrinsics(const std::string& intrinsicFile);

	pthread_mutex_t lidarLock, picLock, lidarGpsLock, cameraGpsLock;
  	sem_t lidarSem, picSem;
	boost::thread lidarRecvThread;
	boost::thread lidarProcessThread;
	boost::thread processPicThread;
	bool continueLidarRecvThread;
	bool continueProcessLidarPacket;
	bool continueProcessPic;
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
	boost::shared_ptr<pandar_rawdata::RawData> data_;
	time_t gps1;
  	GPS_STRUCT_T gps2;
  	time_t gps1Cam[PandoraSDK_CAMERA_NUM];
	GPS_STRUCT_T gps2Cam[PandoraSDK_CAMERA_NUM];
	unsigned int lidarLastGPSSecond;
	unsigned int cameraLastGPSSecond;
	unsigned int cameraTimestamp[PandoraSDK_CAMERA_NUM];
};


PandoraSDK::PandoraSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new PandoraSDK_intern(
		pandoraIP,
		pandoraCameraPort,
		lidarRecvPort,
		startAngle,
		intrinsicFile,
		lidarCorrectionFile,
		cameraCallback,
		lidarCallback);
}

PandoraSDK::PandoraSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new PandoraSDK_intern(
		pandoraIP,
		pandoraCameraPort,
		cameraCallback,
		lidarCallback);
}
PandoraSDK::~PandoraSDK()
{

}

int PandoraSDK::start()
{
	psi->start();
}
void PandoraSDK::stop()
{
	psi->stop();
}




static int cameraClientCallback(void *handle, int cmd, void *param, void *userp)
{
	PandoraPic* pic = (PandoraPic*)param;
	PandoraSDK_intern* pSDK = (PandoraSDK_intern*)userp;
	pSDK->pushPicture(pic);
	return 0;
}

void PandoraSDK_intern::init(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback
)
{
	ip = pandoraIP;
	cport = pandoraCameraPort;
	userCameraCallback = cameraCallback;
	userLidarCallback = lidarCallback;
	lidarRotationStartAngle = static_cast<int>(startAngle * 100);
	sem_init(&lidarSem, 0, 0);
	sem_init(&picSem, 0, 0);
	pthread_mutex_init(&lidarLock, NULL);
	pthread_mutex_init(&picLock, NULL);
	pthread_mutex_init(&lidarGpsLock, NULL);
	pthread_mutex_init(&cameraGpsLock, NULL);

	input.reset(new pandar_pointcloud::InputSocket(lidarRecvPort));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile));
	data_->setup();

	if(intrinsicFile.empty())
	{
		needRemapPicMat = false;
	}
	else
	{
		if (loadIntrinsics(intrinsicFile))
			needRemapPicMat = true;
		else
			needRemapPicMat = false;
	}
}

PandoraSDK_intern::PandoraSDK_intern(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	init(pandoraIP,pandoraCameraPort,lidarRecvPort,startAngle,intrinsicFile,lidarCorrectionFile,cameraCallback,lidarCallback);
}


PandoraSDK_intern::PandoraSDK_intern(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	init(
		pandoraIP,
		pandoraCameraPort,
		PandoraSDK_DEFAULT_LIDAR_RECV_PORT,
		PandoraSDK_DEFAULT_START_ANGLE,
		std::string(""),
		std::string(""),
		cameraCallback,
		lidarCallback);
}

PandoraSDK_intern::~PandoraSDK_intern()
{
	stop();
}

bool PandoraSDK_intern::loadIntrinsics(const std::string &intrinsicFile)
{
	std::vector<cv::Mat> cameraKList;
	std::vector<cv::Mat> cameraDList;
	if (load_camera_intrinsics(intrinsicFile.c_str(), cameraKList, cameraDList))
		return false;
	for (int i = 0; i < PandoraSDK_CAMERA_NUM; i++)
	{
		Mat mapx = Mat(PandoraSDK_IMAGE_SIZE, CV_32FC1);
		Mat mapy = Mat(PandoraSDK_IMAGE_SIZE, CV_32FC1);
		Mat R = Mat::eye(3, 3, CV_32F);
		initUndistortRectifyMap(cameraKList[i], cameraDList[i], R, cameraKList[i], PandoraSDK_IMAGE_SIZE, CV_32FC1, mapx, mapy);
		mapxList.push_back(mapx);
		mapyList.push_back(mapy);
	}
	return true;
}

int PandoraSDK_intern::start()
{	
	continueLidarRecvThread = true;
	continueProcessLidarPacket = true;
	continueProcessPic = true;
	setupCameraClient();
	setupLidarClient();
	return 0;
}

void PandoraSDK_intern::setupCameraClient()
{
	pandoraCameraClient = PandoraClientNew(ip.c_str(), cport, cameraClientCallback, this);
	processPicThread = boost::thread(boost::bind(&PandoraSDK_intern::processPic, this));
}

void PandoraSDK_intern::setupLidarClient()
{
	lidarProcessThread = boost::thread(boost::bind(&PandoraSDK_intern::processLiarPacket, this));
	lidarRecvThread = boost::thread(boost::bind(&PandoraSDK_intern::lidarRecvTask, this));
}


void PandoraSDK_intern::stop()
{
	if (pandoraCameraClient)
		PandoraCLientDestroy(pandoraCameraClient);
	continueProcessPic = false;
	continueLidarRecvThread = false;
	continueProcessLidarPacket = false;
	processPicThread.join();
	lidarRecvThread.join();
	lidarProcessThread.join();
}

void PandoraSDK_intern::pushPicture(PandoraPic *pic)
{
	pthread_mutex_lock(&picLock);
  	pictureList.push_back(pic);
  	pthread_mutex_unlock(&picLock);
  	sem_post(&picSem);
}

void PandoraSDK_intern::processPic()
{
	while(continueProcessPic)
	{
		struct timespec ts;
		if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("processPic, get time error\n");
		}

		ts.tv_sec += 1;
		if (sem_timedwait(&picSem, &ts) == -1)
		{
			printf("no pic\n");
			continue;
		}

		pthread_mutex_lock(&picLock);
		PandoraPic *pic = pictureList.front();
		pictureList.pop_front();
		pthread_mutex_unlock(&picLock);
		
		if (pic == NULL)
		{
			printf("pic is NULL\n");
			return;
		}
		pthread_mutex_lock(&cameraGpsLock);
		if(pic->header.timestamp < 500000 && gps2Cam[pic->header.pic_id].used == 0)
		{
			gps1Cam[pic->header.pic_id] = gps2Cam[pic->header.pic_id].gps;
			gps2Cam[pic->header.pic_id].used =1;
			if (pic->header.pic_id == 0)
			{
				std::cout<<"new gps: "<<gps2Cam[pic->header.pic_id].gps<<std::endl;
			}
		}
		else
		{
			if(pic->header.timestamp < cameraTimestamp[pic->header.pic_id])
			{
				gps1Cam[pic->header.pic_id] += ((cameraTimestamp[pic->header.pic_id] - 100) /1000000) +  1;
				if (pic->header.pic_id == 0)
					std::cout<<"gps round: "<<pic->header.timestamp<<", "<<cameraTimestamp[pic->header.pic_id]<<", "<<gps1Cam[pic->header.pic_id]<<std::endl;
			}
		}

		cameraTimestamp[pic->header.pic_id] = pic->header.timestamp;

		double timestamp = (double)gps1Cam[pic->header.pic_id] + ((double)pic->header.timestamp)/1000000;
		if (pic->header.pic_id == 0)
			std::cout<<timestamp<<std::endl;
		pthread_mutex_unlock(&cameraGpsLock);
		boost::shared_ptr<cv::Mat> cvMatPic(new cv::Mat());
		switch (pic->header.pic_id)
		{
			case 0:
			{
				conv_yuv422_to_mat(*cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
				if (needRemapPicMat)
					remap(cvMatPic->clone(), *cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);
				break;
			}
			case 1:
			case 2:
			case 3:
			case 4:
			{
				conv_yuv400_to_mat(*cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
				if (needRemapPicMat)
					remap(cvMatPic->clone(), *cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);
				break;
			}
			default:
			{
				free(pic->yuv);
				free(pic);
				printf("wrong pic id\n");
				return;
			}
		}
		userCameraCallback(cvMatPic, timestamp, pic->header.pic_id);
		free(pic->yuv);
		free(pic);
	}
}

void PandoraSDK_intern::lidarRecvTask()
{
	while (continueLidarRecvThread)
	{
		PandarPacket pkt;
		int rc = input->getPacket(&pkt, 0); // todo
		if (rc == 1)
		{
			printf("something wrong with input.getPacket\n");
			continue;
		}
		if (rc == 2)
		{
			// gps packet;
			HS_L40_GPS_Parse(&hesaiGps, &pkt.data[0]);
			lidarProcessGps(hesaiGps);
			continue;
		}
		// data packet
		pushLiDARData(pkt);
	}
}

void PandoraSDK_intern::processLiarPacket()
{
	double lastTimestamp = 0.0f;
	int frame_id = 0;
	struct timespec ts;

	while(continueProcessLidarPacket)
	{
		boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
		if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("get time error\n");
		}

		ts.tv_sec += 1;
		if (sem_timedwait(&lidarSem, &ts) == -1)
		{
			std::cout << "no lidarPacket" << std::endl;
			continue;
		}
		pthread_mutex_lock(&lidarLock);
		PandarPacket packet = lidarPacketList.front();
		lidarPacketList.pop_front();
		pthread_mutex_unlock(&lidarLock);
		outMsg->header.frame_id = "pandar";
		outMsg->height = 1;

		double firstStamp = 0.0f;
		pthread_mutex_lock(&lidarGpsLock);
		int ret = data_->unpack(packet, *outMsg, gps1, gps2, firstStamp, lidarRotationStartAngle);
		pthread_mutex_unlock(&lidarGpsLock);
		if (ret == 1)
		{
			if (lastTimestamp != 0.0f)
			{
				if (lastTimestamp > firstStamp)
				{
					printf("error, lastTimestamp > currentTimestamp\n");
				}
			}
			lastTimestamp = firstStamp;
			userLidarCallback(outMsg, firstStamp);
		}
	}
}

void PandoraSDK_intern::pushLiDARData(PandarPacket packet)
{
	pthread_mutex_lock(&lidarLock);
	lidarPacketList.push_back(packet);
	if (lidarPacketList.size() > 6)
	{
		sem_post(&lidarSem);
	}
	pthread_mutex_unlock(&lidarLock);
}

void PandoraSDK_intern::lidarProcessGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
{
	struct tm t;
	t.tm_sec = gpsMsg.second;
	t.tm_min = gpsMsg.minute;
	t.tm_hour = gpsMsg.hour;
	t.tm_mday = gpsMsg.day;
	t.tm_mon = gpsMsg.month - 1;
	t.tm_year = gpsMsg.year + 2000 - 1900;
	t.tm_isdst = 0;
	if (lidarLastGPSSecond != (mktime(&t) + 1))
	{
		lidarLastGPSSecond = mktime(&t) + 1;
		pthread_mutex_lock(&lidarGpsLock);
		gps2.gps = lidarLastGPSSecond;
		gps2.used = 0;
		pthread_mutex_unlock(&lidarGpsLock);
		pthread_mutex_lock(&cameraGpsLock);
		for (int i = 0; i < PandoraSDK_CAMERA_NUM; ++i)
		{
			gps2Cam[i].gps = lidarLastGPSSecond;
			gps2Cam[i].used = 0;
		}
		pthread_mutex_unlock(&cameraGpsLock);
	}

	// if (cameraLastGPSSecond != (mktime(&t) + 1))
	// {

	// 	cameraLastGPSSecond = mktime(&t) + 1;
	// 	for (int i = 0; i < PandoraSDK_CAMERA_NUM; ++i)
	// 	{
	// 		/* code */
	// 		gps2Cam[i].gps = cameraLastGPSSecond;
	// 		gps2Cam[i].used = 0;
	// 	}
	// }
}
