# pandoraSDK使用说明:
**注意:仅支持64位ubuntu16.04和64位ubuntu14.04**
## 1.pandoraSDK目录树如下:
```
pandoraSDK
├── lib  
│   ├── 14.04
│   │   └── libpandoraSDK.so
│   └── 16.04
│       └── libpandoraSDK.so
├── pandoraSDK.h
└── point_types.h
```
使用时,需要include `pandoraSDK.h`和`point_types.h`这两个头文件,构建项目时, 根据不同的ubuntu版本链接不同的 libpandoraSDK.so
## 2. 依赖环境:

### install dependencies:
```
sudo apt-get install libcv-dev libpcap-dev libpcl-dev libboost-all-dev libyaml-cpp-dev libjpeg-dev
```

## 3. 接口说明:
### (1)pandoraSDK有两个构造函数, 每个参数所代表的意义如注释:
```
	PandoraSDK(
		const std::string pandoraIP, 														// pandora的ip
		const unsigned short pandoraCameraPort,		   						// pandora的port
		const unsigned short lidarRecvPort,     								// lidar的数据接收端口， 默认为8080
		const unsigned short gpsRecvPort,												// gps数据接收端口， 默认为10110
		const double startAngle,		  													// lidar的旋转起始角度，默认为0,单位是度
		const std::string intrinsicFile, 												// 摄像头的内参文件路径，为空时，输出的图像是未经过矫正的
		const std::string lidarCorrectionFile, 									// lidar的标定文件路径，为空时，将使用默认参数
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback, 	// 摄像头的callback函数
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback, 				// lidar的callback函数
		boost::function<void(double timestamp)> gpsCallback);		// gps数据的callback函数，可为NULL.timestamp为当前gps的时间戳.
```
```
	PandoraSDK(
		const std::string pandoraIP, //此时,lidar的数据接收端口默认为8080, gps数据接收端口默认为10110, lidar的旋转起始角度默认为0, 输出的图像是未经过矫正的, lidar的标定参数使用默认值
		const unsigned short pandoraCameraPort,
		boost::function<void(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)> cameraCallback,
		boost::function<void(boost::shared_ptr<PPointCloud> pcloudp, double timestamp)> lidarCallback);
```
### (2)panodoraSDK提供两个方法:
```
	int start(); //开始数据接收和传输任务,成功时返回0,否则返回-1
	void stop(); //停止数据接收和传输任务
```

## 4. Sample Code:
### 请参照sample文件