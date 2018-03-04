
#include "hesaiLidarSDK_IN.h"
#include "utilities.h"
Size HesaiLidarSDK_IMAGE_SIZE(HesaiLidarSDK_IMAGE_WIDTH, HesaiLidarSDK_IMAGE_HEIGHT);

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
	packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE;
	packet->fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
										 ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
	return 0;
}

static int cameraClientCallback(void *handle, int cmd, void *param, void *userp)
{
	PandoraPic *pic = (PandoraPic *)param;
	HesaiLidarSDK_internal *pSDK = (HesaiLidarSDK_internal *)userp;
	pSDK->pushPicture(pic);
	return 0;
}

void HesaiLidarSDK_internal::init(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const unsigned int laserReturnType,
		const unsigned int laserCount,
		const unsigned int pclDataType,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
{
	useMode = PandoraUseMode_readPcap;
	input.reset(new pandar_pointcloud::Input(pcapPath, laserReturnType));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile, laserReturnType, laserCount, pclDataType));
	readPacpThread = 0;
	lidarProcessThread = 0;
	lidarRotationStartAngle = 0;
	gps1 = 0;
	gps2.gps = 0;
	gps2.used = 1;
	gps2.usedHour = 1;
	userLidarCallback = lidarCallback;
	for (int i = 0; i < HesaiLidarSDK_CAMERA_NUM; ++i)
	{
		gps1Cam[i] = 0;
		gps2Cam[i].gps = 0;
		cameraTimestamp[i] = 0;
	}

	sem_init(&lidarSem, 0, 0);
	pthread_mutex_init(&lidarLock, NULL);
	pthread_mutex_init(&lidarGpsLock, NULL);
}

void HesaiLidarSDK_internal::init(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		const unsigned short lidarRecvPort,
		const unsigned short gpsRecvPort,
		const double startAngle,
		const std::string intrinsicFile,
		const std::string lidarCorrectionFile,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
		boost::function<void(unsigned int timestamp)> gpsCallback,
		const unsigned int laserReturnType,
		const unsigned int laserCount,
		const unsigned int pclDataType)
{
	ip = pandoraIP;
	cport = pandoraCameraPort;
	userCameraCallback = cameraCallback;
	userLidarCallback = lidarCallback;
	userGpsCallback = gpsCallback;
	lidarRotationStartAngle = static_cast<int>(startAngle * 100);

	gps1 = 0;
	gps2.gps = 0;
	gps2.used = 1;
	gps2.usedHour = 1;
	for (int i = 0; i < HesaiLidarSDK_CAMERA_NUM; ++i)
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
	pandoraCameraClient = NULL;

	input.reset(new pandar_pointcloud::Input(lidarRecvPort, gpsRecvPort));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile, laserReturnType, laserCount, pclDataType));

	if (intrinsicFile.empty())
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

HesaiLidarSDK_internal::HesaiLidarSDK_internal(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		const unsigned short lidarRecvPort,
		const unsigned short gpsRecvPort,
		const double startAngle,
		const std::string intrinsicFile,
		const std::string lidarCorrectionFile,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
		boost::function<void(double timestamp)> gpsCallback,
		const unsigned int laserReturnType,
		const unsigned int laserCount,
		const unsigned int pclDataType)

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
			gpsCallback,
			laserReturnType, laserCount, pclDataType);
}

HesaiLidarSDK_internal::HesaiLidarSDK_internal(
		const std::string pandoraIP,
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	init(
			pandoraIP,
			pandoraCameraPort,
			HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT,
			HesaiLidarSDK_DEFAULT_GPS_RECV_PORT,
			HesaiLidarSDK_DEFAULT_START_ANGLE,
			std::string(""),
			std::string(""),
			cameraCallback,
			lidarCallback,
			NULL, 0, 40, 0);
}

HesaiLidarSDK_internal::HesaiLidarSDK_internal(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const unsigned int laserReturnType,
		const unsigned int laserCount,
		const unsigned int pclDataType,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
{
		init(
			pcapPath,
			lidarCorrectionFile,
			laserReturnType,
			laserCount,
			pclDataType,
			lidarCallback);
}

	HesaiLidarSDK_internal::~HesaiLidarSDK_internal()
	{
		stop();
	}

	bool HesaiLidarSDK_internal::loadIntrinsics(const std::string &intrinsicFile)
	{
		std::vector<cv::Mat> cameraKList;
		std::vector<cv::Mat> cameraDList;
		if (!loadCameraIntrinsics(intrinsicFile.c_str(), cameraKList, cameraDList))
			return false;
		for (int i = 0; i < HesaiLidarSDK_CAMERA_NUM; i++)
		{
			Mat mapx = Mat(HesaiLidarSDK_IMAGE_SIZE, CV_32FC1);
			Mat mapy = Mat(HesaiLidarSDK_IMAGE_SIZE, CV_32FC1);
			Mat R = Mat::eye(3, 3, CV_32F);
			initUndistortRectifyMap(cameraKList[i], cameraDList[i], R, cameraKList[i], HesaiLidarSDK_IMAGE_SIZE, CV_32FC1, mapx, mapy);
			mapxList.push_back(mapx);
			mapyList.push_back(mapy);
		}
		return true;
	}

	int HesaiLidarSDK_internal::start()
	{
		stop();
		switch (useMode)
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

	void HesaiLidarSDK_internal::setupReadPcap()
	{
		continueReadPcap = true;
		readPacpThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::readPcapFile, this));
		continueProcessLidarPacket = true;
		lidarProcessThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::processLiarPacket, this));
	}

	void HesaiLidarSDK_internal::setupCameraClient()
	{
		continueProcessPic = true;
		pandoraCameraClient = PandoraClientNew(ip.c_str(), cport, cameraClientCallback, this);
		processPicThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::processPic, this));
	}

	void HesaiLidarSDK_internal::setupLidarClient()
	{
		continueLidarRecvThread = true;
		continueProcessLidarPacket = true;
		lidarProcessThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::processLiarPacket, this));
		lidarRecvThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::lidarRecvTask, this));
	}

	void HesaiLidarSDK_internal::readPcapFile()
	{
		while (continueReadPcap)
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
			// usleep(HesaiLidarSDK_PCAP_TIME_INTERVAL - 10000);
		}
	}

	void HesaiLidarSDK_internal::stop()
	{
		continueLidarRecvThread = false;
		continueProcessLidarPacket = false;
		continueProcessPic = false;
		continueReadPcap = false;

		if (lidarProcessThread)
		{
			lidarProcessThread->join();
			delete lidarProcessThread;
			lidarProcessThread = 0;
		}
		if (lidarRecvThread)
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
		if (processPicThread)
		{
			processPicThread->join();
			delete processPicThread;
			processPicThread = 0;
		}
		if (readPacpThread)
		{
			readPacpThread->join();
			readPacpThread = 0;
		}
	}

	void HesaiLidarSDK_internal::pushPicture(PandoraPic *pic)
	{
		pthread_mutex_lock(&picLock);
		pictureList.push_back(pic);
		pthread_mutex_unlock(&picLock);
		sem_post(&picSem);
	}

	void HesaiLidarSDK_internal::processPic()
	{
		while (continueProcessPic)
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
			if (pic->header.timestamp < 500000 && gps2Cam[pic->header.pic_id].used == 0)
			{
				gps1Cam[pic->header.pic_id] = gps2Cam[pic->header.pic_id].gps;
				gps2Cam[pic->header.pic_id].used = 1;
			}
			else
			{
				if (pic->header.timestamp < cameraTimestamp[pic->header.pic_id])
				{
					gps1Cam[pic->header.pic_id] += ((cameraTimestamp[pic->header.pic_id] - 100) / 1000000) + 1;
					printf("there is a round in pic, current: %d, last: %d\n", pic->header.timestamp, cameraTimestamp[pic->header.pic_id]);
				}
			}

			cameraTimestamp[pic->header.pic_id] = pic->header.timestamp;
			// double timestamp = (double)gps1Cam[pic->header.pic_id] + ((double)pic->header.timestamp)/1000000;
			struct tm t;
			t.tm_sec = pic->header.UTC_Time.UTC_Second;
			t.tm_min = pic->header.UTC_Time.UTC_Minute;
			t.tm_hour = pic->header.UTC_Time.UTC_Hour;
			t.tm_mday = pic->header.UTC_Time.UTC_Day;
			t.tm_mon = pic->header.UTC_Time.UTC_Month-1;
			t.tm_year = pic->header.UTC_Time.UTC_Year + 2000 - 1900;
			t.tm_isdst = 0;
			double timestamp = mktime(&t) + pic->header.timestamp / 1000000.0;
			pthread_mutex_unlock(&cameraGpsLock);

			boost::shared_ptr<cv::Mat> cvMatPic(new cv::Mat());
			switch (pic->header.pic_id)
			{
			case 0:
			{
				yuv422ToCvmat(*cvMatPic, pic->yuv, pic->header.width, pic->header.height, 8);
				if (needRemapPicMat)
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
				decompressJpeg((unsigned char *)pic->yuv, pic->header.len, bmp, bmpSize);

				*cvMatPic = cv::Mat(HesaiLidarSDK_IMAGE_HEIGHT, HesaiLidarSDK_IMAGE_WIDTH, CV_8UC3, bmp).clone();
				if (needRemapPicMat)
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
			if (userCameraCallback)
			{
				// Add 2 seconds to correct the timestamp, Reason? No reason. liuxingwei@hesaitech.com
				userCameraCallback(cvMatPic, timestamp + 2, pic->header.pic_id);
			}
			free(pic->yuv);
			free(pic);
			pic->yuv = NULL;
			pic = NULL;
		}
	}

	unsigned int lastPacketTimestamp = 0;
	FILE *lidarPacketTimestampFile = fopen("lidar-packet-timestamp.txt", "w");
	int timestampIndexInPacket = 1234;
	void internalAnalysisPacket(const PandarPacket &pkt)
	{
		const uint8_t *buff = &pkt.data[0];
		lastPacketTimestamp = (buff[timestampIndexInPacket] & 0xff) | (buff[timestampIndexInPacket + 1] & 0xff) << 8 |
													((buff[timestampIndexInPacket + 2] & 0xff) << 16) | ((buff[timestampIndexInPacket + 3] & 0xff) << 24);
		fprintf(lidarPacketTimestampFile, "%d\n", lastPacketTimestamp);
	}
	FILE *gpsFile = fopen("lidar-gps-timestamp.txt", "w");
	void HesaiLidarSDK_internal::internalFuncForGPS(const unsigned int &gpsstamp)
	{
		// printf("internalFunc\n");
		struct timeval ts;
		gettimeofday(&ts, NULL);
		double pandoraToSysTimeGap = ts.tv_sec + (double)ts.tv_usec / 1000000 - gpsstamp;
		fprintf(gpsFile, "%d,%f,%d,%d,%d,%d,%d,%d\n", gpsstamp, pandoraToSysTimeGap, lastPacketTimestamp, cameraTimestamp[0], cameraTimestamp[1], cameraTimestamp[2], cameraTimestamp[3], cameraTimestamp[4]);
	}

	void HesaiLidarSDK_internal::lidarRecvTask()
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
			// internalAnalysisPacket(pkt);
		}
	}

	void HesaiLidarSDK_internal::processLiarPacket()
	{
		double lastTimestamp = 0.0f;
		struct timespec ts;

		while (continueProcessLidarPacket)
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

			pthread_mutex_lock(&lidarGpsLock);
			// printf("gps: %d\n", gps1);
			int ret = data_->unpack(packet, *outMsg, gps1, gps2, lidarRotationStartAngle);
			pthread_mutex_unlock(&lidarGpsLock);
			if (ret == 1)
			{
				lastTimestamp = outMsg->points[0].timestamp;
				if(userLidarCallback)
					userLidarCallback(outMsg, lastTimestamp);
			}
		}
	}

	void HesaiLidarSDK_internal::pushLiDARData(PandarPacket packet)
	{
		pthread_mutex_lock(&lidarLock);
		lidarPacketList.push_back(packet);
		if (lidarPacketList.size() > 6)
		{
			sem_post(&lidarSem);
		}
		pthread_mutex_unlock(&lidarLock);
	}

	void HesaiLidarSDK_internal::processGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
	{
		struct tm t;
		t.tm_sec = gpsMsg.second;
		t.tm_min = gpsMsg.minute;
		// t.tm_sec = 0;
		// t.tm_min = 0;

		t.tm_hour = gpsMsg.hour;
		t.tm_mday = gpsMsg.day;
		t.tm_mon = gpsMsg.month - 1;
		t.tm_year = gpsMsg.year + 2000 - 1900;
		t.tm_isdst = 0;

		if (lidarLastGPSSecond != (mktime(&t) + 1))
		{
			// Send the last GPS Time when the PPS occurs
			lidarLastGPSSecond = mktime(&t) + 1;
			pthread_mutex_lock(&lidarGpsLock);
			gps2.gps = lidarLastGPSSecond;
			gps2.used = 0;
			if (gps2.t.tm_hour != t.tm_hour)
			{
				gps2.usedHour = 0;
			}

			memcpy(&gps2.t, &t, sizeof(struct tm));
			gps2.t.tm_min = 0;
			gps2.t.tm_sec = 0;

			pthread_mutex_unlock(&lidarGpsLock);

			pthread_mutex_lock(&cameraGpsLock);
			for (int i = 0; i < HesaiLidarSDK_CAMERA_NUM; ++i)
			{
				gps2Cam[i].gps = lidarLastGPSSecond;
				gps2Cam[i].used = 0;
			}
			pthread_mutex_unlock(&cameraGpsLock);
		}
		// internalFuncForGPS(lidarLastGPSSecond);
		if (userGpsCallback)
			userGpsCallback(lidarLastGPSSecond);
	}
