#include "hesaiLidarSDK.h"
#include "hesaiLidarSDK_IN.h"

HesaiLidarSDK::HesaiLidarSDK(
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
	psi = new HesaiLidarSDK_internal(
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

HesaiLidarSDK::HesaiLidarSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new HesaiLidarSDK_internal(
		pandoraIP,
		pandoraCameraPort,
		cameraCallback,
		lidarCallback);
}

HesaiLidarSDK::HesaiLidarSDK(
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback,
	const unsigned int laserReturnType,
	const unsigned int laserCount,
	const unsigned int pclDataType)
{
	psi = new HesaiLidarSDK_internal(
		std::string(""),
		9870,
		lidarRecvPort,
		gpsRecvPort,
		0,
		std::string(""),
		lidarCorrectionFile,
		NULL,
		lidarCallback,
		gpsCallback,
		laserReturnType, laserCount, pclDataType);
}

HesaiLidarSDK::HesaiLidarSDK(
		const std::string pcapPath,
		const std::string lidarCorrectionFile,
		const unsigned int laserReturnType,
		const unsigned int laserCount,
		const unsigned int pclDataType,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
{
	psi = new HesaiLidarSDK_internal(
		pcapPath,
		lidarCorrectionFile,
		laserReturnType, laserCount, pclDataType,
		lidarCallback);
}

HesaiLidarSDK::~HesaiLidarSDK()
{
	delete psi;
}

int HesaiLidarSDK::start()
{
	psi->start();
}
void HesaiLidarSDK::stop()
{
	psi->stop();
}
