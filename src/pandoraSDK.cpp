#include "pandoraSDK.h"


int HS_L40_GPS_Parse(HS_LIDAR_L40_GPS_Packet *packet , const unsigned char* recvbuf)
{
    int index = 0;
    packet->flag = (recvbuf[index] & 0xff)|((recvbuf[index + 1] & 0xff)<< 8);
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
    packet->fineTime = (recvbuf[index]& 0xff)| (recvbuf[index + 1]& 0xff) << 8 |
                       ((recvbuf[index + 2 ]& 0xff) << 16) | ((recvbuf[index + 3]& 0xff) << 24);
    return 0;
}



PandoraSDK::PandoraSDK(std::string pandoraIP,
											 int cameraPort,
											 int lidarRecvPort,
											 int startAngle,
											 std::string intrinsicFile,
											 std::string extrinsicFile,
											 std::string lidarCorrectionFile,
											 boost::function<void(cv::Mat mat, double timestamp, int pic_id)> cameraCallback,
											 boost::function<void(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)> lidarCallback)
{
	localGPSOffset = 0.0f;
	continueLidarThread = true;
	lidarRotationStartAngle = startAngle;
	ip = pandoraIP;
	cport = cameraPort;
	lport = lidarRecvPort;
	userCameraCallbackPara = cameraCallback;
	userLidarCallbackPara = lidarCallback;
	// data_ = new pandar_rawdata::RawData(lidarCorrectionFile);
	data_ = new pandar_rawdata::RawData(lidarCorrectionFile);
	start();
}

void PandoraSDK::loadIntrinsics(std::string& intrinsicFile)
{
	std::vector<cv::Mat> camera_k_list;
  std::vector<cv::Mat> camera_d_list;
	load_camera_intrinsics(intrinsicFile.c_str(), camera_k_list, camera_d_list);
  for (int i = 0; i < CAMERA_NUM; i++)
  {
    Mat mapx = Mat(IMAGE_SIZE, CV_32FC1);
    Mat mapy = Mat(IMAGE_SIZE, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(camera_k_list[i],camera_d_list[i],R,camera_k_list[i],IMAGE_SIZE,CV_32FC1,mapx,mapy);
    mapxList.push_back(mapx);
    mapyList.push_back(mapy);
  }
}

void PandoraSDK::setupCameraClient()
{
	pandoraClient = PandoraClientNew(ip.c_str(), cport, PandoraSDK::cameraClientCallback, this);
}

void PandoraSDK::setupLidarClient()
{
	// int ret = pthread_create(&lidarThread, NULL, (void*)lidarTask, (void*)this);
	lidarBoostThread = boost::thread(boost::bind(&PandoraSDK::lidarTask, this));
}

int PandoraSDK::start()
{
	setupCameraClient();
	setupLidarClient();
	return 0;
}

void PandoraSDK::stop()
{
	PandoraCLientDestroy(pandoraClient);
	destoryLidarThread();
}

void PandoraSDK::destoryLidarThread()
{
	continueLidarThread = false;
	lidarBoostThread.join();
}

void PandoraSDK::lidarTask()
{
	input = pandar_pointcloud::InputSocket(lport);
	double lastTimestamp = 0.0f;
	PPointCloud::Ptr outMsg(new PPointCloud());
	int frame_id = 0;
	while(continueLidarThread)
	{
		PandarPacket* pkt;
		int rc = input.getPacket(pkt, 0); // todo
		if (rc == 2)
		{
			// gps packet;
			HS_L40_GPS_Parse(&hesaiGps, &pkt->data[0]);
			cameraProcessGps(hesaiGps);
			lidarProcessGps(hesaiGps);
			continue;
		}

		outMsg->header.frame_id = "pandar";
		outMsg->height = 1;
		double firstStamp = 0.0f;
		int ret = data_->unpack(*pkt, *outMsg, gps1, gps2, firstStamp, lidarRotationStartAngle);

		if (ret == 1)
		{
			if (lastTimestamp != 0.0f)
			{
				if (lastTimestamp > firstStamp)
				{
					perror("error, lastTimestamp large than currentTimestamp\n");
				}
			}

			lastTimestamp = firstStamp;
			// pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
			userLidarCallback(outMsg, outMsg->header.stamp);
			memset(pkt, 0, sizeof(*pkt)); // initialize to zeros
			// output_.publish(outMsg);
			outMsg->clear();
		}
	}
}

void PandoraSDK::lidarProcessGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
{
   
    struct tm t;
    t.tm_sec = gpsMsg.second;
    t.tm_min = gpsMsg.minute;
    t.tm_hour = gpsMsg.hour;
    t.tm_mday = gpsMsg.day;
    t.tm_mon = gpsMsg.month - 1;
    t.tm_year = gpsMsg.year + 2000 - 1900;
    t.tm_isdst = 0;  
    if(lidarLastGPSSecond != (mktime(&t) + 1))
    {
        lidarLastGPSSecond = (mktime(&t) + 1);
        gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
        gps2.used = 0;
    }
}



void PandoraSDK::cameraProcessGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
{
  struct tm t;
  t.tm_sec = gpsMsg.second;
  t.tm_min = gpsMsg.minute;
  t.tm_hour = gpsMsg.hour;
  t.tm_mday = gpsMsg.day;
  t.tm_mon = gpsMsg.month - 1;
  t.tm_year = gpsMsg.year + 2000 - 1900;
  t.tm_isdst = 0;  

  if(cameraLastGPSSecond != (mktime(&t) + 1))
  {

    cameraLastGPSSecond = mktime(&t) + 1;
    for (int i = 0; i < CAMERA_NUM; ++i)
    {
      /* code */
      gps2Cam[i].gps = mktime(&t) + 1;// the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
      gps2Cam[i].used = 0;
    }
  }

  if (localGPSOffset == 0.0f)
  {
    /* code */
		struct timeval timeNow;
		gettimeofday(&timeNow, NULL);
    // localGPSOffset = ros::Time::now().toSec() - (double)gps2Cam[0].gps;
		localGPSOffset = timeNow.tv_sec + timeNow.tv_usec / 1000  - (double)gps2Cam[0].gps;
    printf("offset %f " , localGPSOffset);
  }  
}

std::vector<Mat> PandoraSDK::getMapxList()
{
	return mapxList;
}

std::vector<Mat> PandoraSDK::getMapyList()
{
	return mapyList;
}


void PandoraSDK::userCameraCallback(cv::Mat mat, double timestamp, int pic_id)
{
	userCameraCallbackPara(mat, timestamp, pic_id);
}

void PandoraSDK::userLidarCallback(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)
{
	userLidarCallbackPara(cld, timestamp);
}

static int cameraClientCallback(void *handle, int cmd, void *param, void *userp)
{
	PandoraPic *pic = (PandoraPic *)param;
	PandoraSDK *pSDK = (PandoraSDK *)userp;
	cv::Mat cvMatPic;
	std::vector<cv::Mat> mapxList = pSDK->getMapxList();
	std::vector<cv::Mat> mapyList = pSDK->getMapyList();
	switch (pic->header.pic_id)
	{
		case 0:
		{
			conv_yuv422_to_mat(cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
			remap(cvMatPic.clone(), cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);
			break;
		}
		case 1:
		case 2:
		case 3:
		case 4:
		{
			conv_yuv400_to_mat(cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
			remap(cvMatPic.clone(), cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);
			break;
		}
		default:
		{
			free(pic->yuv);
			free(pic);
			printf("wrong pic id\n");
			return -1;
		}
	}
	free(pic->yuv);
	free(pic);

	// pthread_mutex_lock(&gpsLock);
	// double timestamp = lastTimestamp + pic->header.timestamp;
	// pthread_mutex_unlock(&gpsLock);
	double timestamp = 0;
	pSDK->userCameraCallback(cvMatPic, timestamp, pic->header.pic_id);

	return 0;
}