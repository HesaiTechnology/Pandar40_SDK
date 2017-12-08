#include "pandoraSDK.h"

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

void conv_yuv400_to_mat(cv::Mat &dst, void *pYUV400, int nWidth, int nHeight, int bit_depth)
{
	IplImage *yimg;

	if (!pYUV400)
	{
		return;
	}

	if (bit_depth == 8)
	{
		yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_8U, 1);
	}
	else
	{
		yimg = cvCreateImageHeader(cvSize(nWidth, nHeight), IPL_DEPTH_16U, 1);
	}

	cvSetData(yimg, (unsigned char *)pYUV400, nWidth);

	dst = cv::cvarrToMat(yimg);
	cvReleaseImageHeader(&yimg);
}

void YUVToRGB(const int &iY, const int &iU, const int &iV, int &iR, int &iG, int &iB)
{
	assert(&iR != NULL && &iG != NULL && &iB != NULL);

	iR = iY + 1.13983 * (iV - 128);
	iG = iY - 0.39465 * (iU - 128) - 0.58060 * (iV - 128);
	iB = iY + 2.03211 * (iU - 128);

	iR = iR > 255 ? 255 : iR;
	iR = iR < 0 ? 0 : iR;

	iG = iG > 255 ? 255 : iG;
	iG = iG < 0 ? 0 : iG;

	iB = iB > 255 ? 255 : iB;
	iB = iB < 0 ? 0 : iB;
}

void yuv422_to_rgb24(unsigned char *uyvy422, unsigned char *rgb24, int width, int height)
{
	int iR, iG, iB;
	int iY0, iY1, iU, iV;
	int i = 0;
	int j = 0;
	for (i = 0; i < width * height * 2; i += 4)
	{
		iU = uyvy422[i + 0];
		iY0 = uyvy422[i + 1];
		iV = uyvy422[i + 2];
		iY1 = uyvy422[i + 3];

		YUVToRGB(iY0, iU, iV, iR, iG, iB);
		rgb24[j++] = iR;
		rgb24[j++] = iG;
		rgb24[j++] = iB;
		YUVToRGB(iY1, iU, iV, iR, iG, iB);
		rgb24[j++] = iR;
		rgb24[j++] = iG;
		rgb24[j++] = iB;
	}
}

// #define PIC_MAX_WIDTH (1920)
// #define PIC_MAX_HEIGHT (1080)

void conv_yuv422_to_mat(cv::Mat &dst, void *pYUV422, int nWidth, int nHeight, int bit_depth)
{

	if (!pYUV422)
	{
		return;
	}

	// static unsigned char rgb24_buffer[PIC_MAX_WIDTH*PIC_MAX_HEIGHT*3]; // todo, temporarily
	unsigned char *rgb24_buffer = new unsigned char[nWidth * nHeight * 3];
	yuv422_to_rgb24((unsigned char *)pYUV422, rgb24_buffer, nWidth, nHeight);
	dst = cv::Mat(nHeight, nWidth, CV_8UC3, rgb24_buffer).clone();
	delete[] rgb24_buffer;
}

bool load_camera_intrinsics(const std::string &filename, std::vector<cv::Mat> &camera_k_list, std::vector<cv::Mat> &camera_d_list)
{

	if ((access(filename.c_str(), 0)) == -1)
	{
		return false;
	}
	YAML::Node yn = YAML::LoadFile(filename);
	std::string camera_id;
	cv::Mat camera_k, camera_d;
	for (int i = 0; i < 5; i++)
	{
		// camera_id = std::to_string(i);
		camera_id = boost::lexical_cast<std::string>(i);

		if (yn[camera_id]["K"].IsDefined())
		{
			camera_k = cv::Mat::zeros(3, 3, CV_64FC1);
			for (int i = 0; i < yn[camera_id]["K"].size(); ++i)
			{
				camera_k.at<double>(i) = yn[camera_id]["K"][i].as<double>();
			}
			camera_k_list.push_back(camera_k);
		}
		else
		{
			return false;
		}
		if (yn[camera_id]["D"].IsDefined())
		{
			camera_d = cv::Mat::zeros(yn[camera_id]["D"].size(), 1, CV_64FC1);
			for (int i = 0; i < yn[camera_id]["D"].size(); ++i)
			{
				camera_d.at<double>(i) = yn[camera_id]["D"][i].as<double>();
			}
			camera_d_list.push_back(camera_d);
		}
		else
		{
			camera_d = cv::Mat::zeros(5, 1, CV_64FC1);
			return false;
		}
	}
}

const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 720;
Size IMAGE_SIZE(IMAGE_WIDTH, IMAGE_HEIGHT);
std::vector<Mat> mapxList;
std::vector<Mat> mapyList;


int cvCounter = 0;
static int cameraClientCallback(void *handle, int cmd, void *param, void *userp)
{
	PandoraPic* pic = (PandoraPic*)param;
	PandoraSDK* pSDK = (PandoraSDK*)userp;
	cv::Mat cvMatPic;
	// std::vector<cv::Mat> mapxList = pSDK->getMapxList();
	// std::vector<cv::Mat> mapyList = pSDK->getMapyList();
	// printf("pic_id: %d\n", pic->header.pic_id);
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

	// pthread_mutex_lock(&gpsLock);
	// double timestamp = lastTimestamp + pic->header.timestamp;
	// pthread_mutex_unlock(&gpsLock);
	double timestamp = pSDK->getCameraLastGPSSecond() + pic->header.timestamp / 1000000.0;
	// std::cout<<"row"<<cvMatPic.height<<std::endl;
	// char cvName[256];
	// sprintf(cvName, "%d.jpg", ++cvCounter);
	// imwrite(cvName, cvMatPic);
	pSDK->userCameraCallback(cvMatPic, timestamp, pic->header.pic_id);
	free(pic->yuv);
	free(pic);

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
	continueProcessLidarPacket = true;
	lidarRotationStartAngle = startAngle;
	ip = pandoraIP;
	cport = cameraPort;

	sem_init(&lidarSem, 0, 0);
	pthread_mutex_init(&lidarLock, NULL);

	input.reset(new pandar_pointcloud::InputSocket(lidarRecvPort));
	userCameraCallbackPara = cameraCallback;
	userLidarCallbackPara = lidarCallback;
	loadIntrinsics(intrinsicFile);
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile));
	data_->setup();
}

void PandoraSDK::loadIntrinsics(std::string &intrinsicFile)
{
	std::vector<cv::Mat> camera_k_list;
	std::vector<cv::Mat> camera_d_list;
	load_camera_intrinsics(intrinsicFile.c_str(), camera_k_list, camera_d_list);
	for (int i = 0; i < CAMERA_NUM; i++)
	{
		Mat mapx = Mat(IMAGE_SIZE, CV_32FC1);
		Mat mapy = Mat(IMAGE_SIZE, CV_32FC1);
		Mat R = Mat::eye(3, 3, CV_32F);
		initUndistortRectifyMap(camera_k_list[i], camera_d_list[i], R, camera_k_list[i], IMAGE_SIZE, CV_32FC1, mapx, mapy);
		mapxList.push_back(mapx);
		mapyList.push_back(mapy);
	}
}

void PandoraSDK::setupCameraClient()
{
	pandoraCameraClient = PandoraClientNew(ip.c_str(), cport, cameraClientCallback, this);
}

void PandoraSDK::setupLidarClient()
{
	// int ret = pthread_create(&lidarThread, NULL, (void*)lidarTask, (void*)this);
	lidarProcessThread = boost::thread(boost::bind(&PandoraSDK::processLiarPacket, this));
	lidarRecvThread = boost::thread(boost::bind(&PandoraSDK::lidarTask, this));
}

int PandoraSDK::start()
{
	setupCameraClient();
	setupLidarClient();
	return 0;
}

void PandoraSDK::stop()
{
	PandoraCLientDestroy(pandoraCameraClient);
	destoryLidarThread();
}

void PandoraSDK::destoryLidarThread()
{
	continueLidarThread = false;
	continueProcessLidarPacket = false;
	lidarRecvThread.join();
	lidarProcessThread.join();
}

void PandoraSDK::lidarTask()
{
	// input = pandar_pointcloud::InputSocket(lport);
	double lastTimestamp = 0.0f;
	int frame_id = 0;
	while (continueLidarThread)
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
			// cameraProcessGps(hesaiGps);
			lidarProcessGps(hesaiGps);
			continue;
		}
		pushLiDARData(pkt);
	}
}

int counter = 0;
void PandoraSDK::processLiarPacket()
{
	double lastTimestamp = 0.0f;

	int frame_id = 0;
	struct timespec ts;

	while(continueProcessLidarPacket)
	{
		boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
		if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("get time error");
		}

		ts.tv_sec += 1;
		if (sem_timedwait(&lidarSem, &ts) == -1)
		{
			std::cout << "no lidarPacket" << std::endl;
			continue;
		}
		pthread_mutex_lock(&lidarLock);
		PandarPacket packet = LiDARDataSet.front();
		// printf("process size: %d\n", LiDARDataSet.size());
		LiDARDataSet.pop_front();
		pthread_mutex_unlock(&lidarLock);
		outMsg->header.frame_id = "pandar";
		outMsg->height = 1;

		double firstStamp = 0.0f;
		int ret = data_->unpack(packet, *outMsg, gps1, gps2, firstStamp, lidarRotationStartAngle);
		if (ret == 1)
		{
			if (lastTimestamp != 0.0f)
			{
				if (lastTimestamp > firstStamp)
				{
					printf("errrrrrrrrr");
				}
			}

			lastTimestamp = firstStamp;
			// struct timeval tss;
			// gettimeofday(&tss, NULL);
			// pcl_conversions::toPCL(ros::Time(firstStamp), outMsg->header.stamp);
			// outMsg->header.stamp = (double)tss.tv_sec + (double)tss.tv_usec / 1000.0; 
			printf("pointsize: %d\n", outMsg->points.size());
			// char pcdFileName[256];
  		// sprintf(pcdFileName, "%d.pcd", ++counter);
  		// pcl::io::savePCDFileASCII(pcdFileName, *outMsg);
			userLidarCallbackPara(outMsg, firstStamp);
			// outMsg->clear();
		}
	}
}

void PandoraSDK::pushLiDARData(PandarPacket packet)
{
	pthread_mutex_lock(&lidarLock);
	LiDARDataSet.push_back(packet);
	// printf("size: %d\n", LiDARDataSet.size());
	if (LiDARDataSet.size() > 6)
	{
		sem_post(&lidarSem);
	}
	pthread_mutex_unlock(&lidarLock);
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
	if (lidarLastGPSSecond != (mktime(&t) + 1))
	{
		lidarLastGPSSecond = (mktime(&t) + 1);
		gps2.gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
		gps2.used = 0;
	}

	if (cameraLastGPSSecond != (mktime(&t) + 1))
	{

		cameraLastGPSSecond = mktime(&t) + 1;
		for (int i = 0; i < CAMERA_NUM; ++i)
		{
			/* code */
			gps2Cam[i].gps = mktime(&t) + 1; // the gps always is the last gps, the newest GPS data is after the PPS(Serial port transmition speed...)
			gps2Cam[i].used = 0;
		}
	}

	if (localGPSOffset == 0.0f)
	{
		/* code */
		struct timeval timeNow;
		gettimeofday(&timeNow, NULL);
		// localGPSOffset = ros::Time::now().toSec() - (double)gps2Cam[0].gps;
		localGPSOffset = timeNow.tv_sec + timeNow.tv_usec / 1000 - (double)gps2Cam[0].gps;
		printf("offset %f ", localGPSOffset);
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

double PandoraSDK::getCameraLastGPSSecond()
{
	return cameraLastGPSSecond;
}

void PandoraSDK::userCameraCallback(cv::Mat mat, double timestamp, int pic_id)
{
	userCameraCallbackPara(mat, timestamp, pic_id);
}