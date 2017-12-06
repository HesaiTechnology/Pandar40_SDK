#include <ros/ros.h>
#include <pandoraSDK>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class PandoraCallback
{
public:
  PandoraCallback(image_transport::ImageTransport it)
  {

    pub[0] = it.advertise("pandora_camera/pandora_camera0", 1);
    pub[1] = it.advertise("pandora_camera/pandora_camera1", 1);
    pub[2] = it.advertise("pandora_camera/pandora_camera2", 1);
    pub[3] = it.advertise("pandora_camera/pandora_camera3", 1);
    pub[4] = it.advertise("pandora_camera/pandora_camera4", 1);

    lidarPub = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);

  }

  void cameraCallback(cv::Mat mat, double timestamp, int pic_id)
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat).toImageMsg();
    pub[pic_id].publish(msg);
  }

  void lidarCallback(pcl::PointCloud<PPoint>::Ptr cld, double timestamp)
  {
    pcl_conversions::toPCL(ros::Time(timestamp), outMsg->header.stamp);
    lidarPub.publish(cld);
  }
private:
    image_transport::Publisher pub[5];
    ros::Publisher lidarPub;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandora");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  image_transport::ImageTransport it(nh);
  PandoraCallback pback(it);
  PandoraSDK psdk(std::string("172.31.3.165"), 9870, "", "", "", pback.cameraCallback, pback.lidarCallback);
  while (nh.ok())
  {
    ros::spinOnce();
  }
}