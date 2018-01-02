
# include "pandoraSDK_IN.h"
#include "utilities.h"
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


static int cameraClientCallback(void *handle, int cmd, void *param, void *userp)
{
	PandoraPic* pic = (PandoraPic*)param;
	PandoraSDK_internal* pSDK = (PandoraSDK_internal*)userp;
	pSDK->pushPicture(pic);
	return 0;
}

void PandoraSDK_internal::init(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback
)
{
	ip = pandoraIP;
	cport = pandoraCameraPort;
	userCameraCallback = cameraCallback;
	userLidarCallback = lidarCallback;
	userGpsCallback = gpsCallback;
	lidarRotationStartAngle = static_cast<int>(startAngle * 100);

	gps1 = 0;
	gps2.gps = 0;
	for(int i = 0; i < PandoraSDK_CAMERA_NUM; ++i)
	{
		gps1Cam[i] = 0;
		gps2Cam[i].gps = 0;
		cameraTimestamp[i] = 0;
	}

	if (ip.empty())
		useMode = PandoraUseMode_onlyLidar;
	else
		useMode = PandoraUseMode_lidarAndCamera;
	sem_init(&lidarSem, 0, 0);
	sem_init(&picSem, 0, 0);
	pthread_mutex_init(&lidarLock, NULL);
	pthread_mutex_init(&picLock, NULL);
	pthread_mutex_init(&lidarGpsLock, NULL);
	pthread_mutex_init(&cameraGpsLock, NULL);

	lidarRecvThread = 0;
	lidarProcessThread = 0;
	processPicThread = 0;
	readPacpThread = 0;
	pandoraCameraClient = NULL;

	input.reset(new pandar_pointcloud::Input(lidarRecvPort, gpsRecvPort));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile));

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

PandoraSDK_internal::PandoraSDK_internal(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(double timestamp)> gpsCallback)

{
	init(
		pandoraIP,
		pandoraCameraPort,
		lidarRecvPort,
		gpsRecvPort,
		startAngle,
		intrinsicFile,
		lidarCorrectionFile,
		cameraCallback,
		lidarCallback,
		gpsCallback);
}


PandoraSDK_internal::PandoraSDK_internal(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	init(
		pandoraIP,
		pandoraCameraPort,
		PandoraSDK_DEFAULT_LIDAR_RECV_PORT,
		PandoraSDK_DEFAULT_GPS_RECV_PORT,
		PandoraSDK_DEFAULT_START_ANGLE,
		std::string(""),
		std::string(""),
		cameraCallback,
		lidarCallback,
		NULL);
}

PandoraSDK_internal::PandoraSDK_internal(
	const std::string pcapPath,
	const std::string lidarCorrectionFile,
	const double startAngle,
	boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
{
	lidarRotationStartAngle = static_cast<int>(startAngle * PandoraSDK_PCAP_TIME_INTERVAL);
	userLidarCallback = lidarCallback;
	useMode = PandoraUseMode_readPcap;
	input.reset(new pandar_pointcloud::Input(pcapPath));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile));

}

PandoraSDK_internal::~PandoraSDK_internal()
{
	stop();
}

bool PandoraSDK_internal::loadIntrinsics(const std::string &intrinsicFile)
{
	std::vector<cv::Mat> cameraKList;
	std::vector<cv::Mat> cameraDList;
	if (!loadCameraIntrinsics(intrinsicFile.c_str(), cameraKList, cameraDList))
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

int PandoraSDK_internal::start()
{	
	stop();
	switch(useMode)
	{
		case PandoraUseMode_readPcap:
		{
			setupReadPcap();
			break;
		}

		case PandoraUseMode_onlyLidar:
		{
			setupLidarClient();
			break;
		}
		case PandoraUseMode_lidarAndCamera:
		{
			setupCameraClient();
			setupLidarClient();
			break;
		}
		default:
		{
			printf("wrong useMode\n");
			return -1;
		}
	}

	return 0;
}

void PandoraSDK_internal::setupReadPcap()
{
	continueReadPcap = true;
	readPacpThread = new boost::thread(boost::bind(&PandoraSDK_internal::readPcapFile, this));
	continueProcessLidarPacket = true;
	lidarProcessThread = new boost::thread(boost::bind(&PandoraSDK_internal::processLiarPacket, this));
}

void PandoraSDK_internal::setupCameraClient()
{
	continueProcessPic = true;
	pandoraCameraClient = PandoraClientNew(ip.c_str(), cport, cameraClientCallback, this);
	processPicThread = new boost::thread(boost::bind(&PandoraSDK_internal::processPic, this));
}

void PandoraSDK_internal::setupLidarClient()
{
	continueLidarRecvThread = true;
	continueProcessLidarPacket = true;
	lidarProcessThread = new boost::thread(boost::bind(&PandoraSDK_internal::processLiarPacket, this));
	lidarRecvThread = new boost::thread(boost::bind(&PandoraSDK_internal::lidarRecvTask, this));
}


void PandoraSDK_internal::readPcapFile()
{
	while(continueReadPcap)
	{
		PandarPacket pkt;
		int rc = input->getPacketFromPcap(&pkt); // todo
		if (rc == -1)
		{
			printf("something wrong with input.getPacket\n");
			continue;
		}
		if (rc == 1)
		{
			// gps packet;
			HS_L40_GPS_Parse(&hesaiGps, &pkt.data[0]);
			processGps(hesaiGps);
			continue;
		}
		// data packet
		pushLiDARData(pkt);
		// 10 ms for read
		usleep(PandoraSDK_PCAP_TIME_INTERVAL - 10000);
	}
}

void PandoraSDK_internal::stop()
{
	continueLidarRecvThread = false;
	continueProcessLidarPacket = false;
	continueProcessPic = false;
	continueReadPcap = false;

	if(lidarProcessThread)
	{
		lidarProcessThread->join();
		delete lidarProcessThread;
		lidarProcessThread = 0;
	}
	if(lidarRecvThread)
	{
		lidarRecvThread->join();
		delete lidarRecvThread;
		lidarRecvThread = 0;
	}
	if (pandoraCameraClient)
	{
		PandoraClientDestroy(pandoraCameraClient);
		pandoraCameraClient = NULL;
	}
	if(processPicThread)
	{
		processPicThread->join();
		delete processPicThread;
		processPicThread = 0;
	}
	if(readPacpThread)
	{
		readPacpThread->join();
		readPacpThread = 0;
	}

}

void PandoraSDK_internal::pushPicture(PandoraPic *pic)
{
	pthread_mutex_lock(&picLock);
  pictureList.push_back(pic);
  pthread_mutex_unlock(&picLock);
  sem_post(&picSem);
}

void PandoraSDK_internal::processPic()
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
		}
		else
		{
			if(pic->header.timestamp < cameraTimestamp[pic->header.pic_id])
			{
				gps1Cam[pic->header.pic_id] += ((cameraTimestamp[pic->header.pic_id] - 100) /1000000) +  1;
				printf("there is a round, current: %d, last: %d\n", pic->header.timestamp, cameraTimestamp[pic->header.pic_id]);
			}
		}

		cameraTimestamp[pic->header.pic_id] = pic->header.timestamp;
		double timestamp = (double)gps1Cam[pic->header.pic_id] + ((double)pic->header.timestamp)/1000000;
		pthread_mutex_unlock(&cameraGpsLock);

		boost::shared_ptr<cv::Mat> cvMatPic(new cv::Mat());
		switch (pic->header.pic_id)
		{
			case 0:
			{
				yuv422ToCvmat(*cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
				if(needRemapPicMat)
					remap(cvMatPic->clone(), *cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);
				break;
			}
			case 1:
			case 2:
			case 3:
			case 4:
			{
#if 0
				// conv_yuv400_to_mat(*cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
				std::vector<uint8_t> cvInputArray;
  			cvInputArray.assign((char*)pic->yuv, (char*)pic->yuv + pic->header.len);
  			*cvMatPic = cv::imdecode(cvInputArray, CV_LOAD_IMAGE_COLOR);
#else
				unsigned char *bmp;
				unsigned long bmpSize;
				// struct timeval ts_s, ts_e;
				// gettimeofday(&ts_s, NULL);
				decompressJpeg((unsigned char*)pic->yuv, pic->header.len, bmp, bmpSize);

				*cvMatPic = cv::Mat(PandoraSDK_IMAGE_HEIGHT, PandoraSDK_IMAGE_WIDTH, CV_8UC3, bmp).clone();
				if(needRemapPicMat)
					remap(cvMatPic->clone(), *cvMatPic, mapxList[pic->header.pic_id], mapyList[pic->header.pic_id], INTER_LINEAR);

				// gettimeofday(&ts_e, NULL);
				// printf("diff: %d\n", ts_e.tv_usec - ts_s.tv_usec);
				free(bmp);
#endif
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
		pic->yuv = NULL;
		pic = NULL;
	}
}

void PandoraSDK_internal::lidarRecvTask()
{
	while (continueLidarRecvThread)
	{
		PandarPacket pkt;
		int rc = input->getPacket(&pkt, 0);
		if (rc == -1)
		{
			continue;
		}
		if (rc == 1)
		{
			// gps packet;
			HS_L40_GPS_Parse(&hesaiGps, &pkt.data[0]);
			processGps(hesaiGps);
			continue;
		}
		pushLiDARData(pkt);
	}
}

void PandoraSDK_internal::processLiarPacket()
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

void PandoraSDK_internal::pushLiDARData(PandarPacket packet)
{
	pthread_mutex_lock(&lidarLock);
	lidarPacketList.push_back(packet);
	if (lidarPacketList.size() > 6)
	{
		sem_post(&lidarSem);
	}
	pthread_mutex_unlock(&lidarLock);
}

void PandoraSDK_internal::processGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
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
	if (userGpsCallback)
		userGpsCallback(lidarLastGPSSecond);

}
