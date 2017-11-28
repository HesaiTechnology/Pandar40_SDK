#ifndef PANDAORA_SDK
#define PANDAORA_SDK
#include<string>
#include<boost/function.hpp>
#include<opencv>

class PandoraSDK{
public:
	PandoraSDK(std::string pandoraIP ,  
			   int cameraPort , 
			   int lidarRecvPort , 
			   boost::function<void (cv::Mat , double timestamp , int pic_id)> camera_callBack, 
			   boost::function<void (pcl::PointCloud<PPoint>::Ptr cld , double timestamp)> lidar_callBack, 
			    );
	~PandoraSDK();
	int start();
	void stop();
private:
	
};
#endif