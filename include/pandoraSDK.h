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
const int IMAGE_WIDTH = 1280;
const int IMAGE_HEIGHT = 720;
Size IMAGE_SIZE(IMAGE_WIDTH, IMAGE_HEIGHT);

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
						 boost::function<void(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)> lidarCallback);
	~PandoraSDK();
	int start();
	void stop();
	void setupCameraClient();
	void setupLidarClient();
	void lidarTask();
	void loadIntrinsics(std::string& intrinsicFile);
	void cameraProcessGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);
	void lidarProcessGps(HS_LIDAR_L40_GPS_Packet& gpsMsg);

	static int cameraClientCallback(void *handle, int cmd, void *param, void *userp);
	std::vector<Mat> getMapxList();
	std::vector<Mat> getMapyList();
	void userCameraCallback(cv::Mat mat, double timestamp, int pic_id);
	void userLidarCallback(pcl::PointCloud<PPoint>::Ptr cld, double timestamp);

private:
	pthread_t lidarThread;
	bool continueLidarThread;
	pthread_mutex_t gpsLock;
	std::string ip;
	int cport;
	int lport;
	int lidarRotationStartAngle;
	PandoraClient* pandoraClient;
	HS_LIDAR_L40_GPS_Packet hesaiGps;
  time_t gps1;
  gps_struct_t gps2;
	unsigned int lastGPSSecond;
	std::vector<Mat> mapxList;
  std::vector<Mat> mapyList;
	pandar_pointcloud::InputSocket input;
	boost::function<void(cv::Mat, double timestamp, int pic_id)> userCameraCallbackPara;
	boost::function<void(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)> userLidarCallbackPara;
	// boost::shared_ptr<pandar_rawdata::RawData> data_;
	pandar_rawdata::RawData* data_;
	unsigned int cameraTimestamp[CAMERA_NUM];
  time_t gps1Cam[CAMERA_NUM];
	gps_struct_t gps2Cam[CAMERA_NUM];
	double lidarLastGPSSecond;
	double cameraLastGPSSecond;
	double localGPSOffset;
};

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

#endif