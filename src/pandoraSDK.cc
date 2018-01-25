#include "pandoraSDK.h"
#include "pandoraSDK_IN.h"

PandoraSDK::PandoraSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const double startAngle,
	const std::string intrinsicFile,
	const std::string lidarCorrectionFile,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback)
{
	psi = new PandoraSDK_internal(
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

PandoraSDK::PandoraSDK(
	const std::string pandoraIP,
	const unsigned short pandoraCameraPort,
	boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
	boost::function<void(boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new PandoraSDK_internal(
		pandoraIP,
		pandoraCameraPort,
		cameraCallback,
		lidarCallback);
}

// PandoraSDK::PandoraSDK(
// 		const std::string pcapPath,
// 		const std::string lidarCorrectionFile,
// 		const double startAngle,
// 		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback)
// {
// 	psi = new PandoraSDK_internal(
// 		pcapPath,
// 		lidarCorrectionFile,
// 		startAngle,
// 		lidarCallback);
// }

PandoraSDK::~PandoraSDK()
{
	delete psi;
}

int PandoraSDK::start()
{
	psi->start();
}
void PandoraSDK::stop()
{
	psi->stop();
}
