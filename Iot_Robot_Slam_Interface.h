#ifndef IOT_ROBOT_SLAM_INTERFACE
#define IOT_ROBOT_SLAM_INTERFACE

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

class Iot_Robot_Slam_Interface{
public:
	Iot_Robot_Slam_Interface(){}
	~Iot_Robot_Slam_Interface(){}

	virtual void init() = 0;
	virtual void runoneframe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame) = 0; // This is the actual process of slam
	virtual void uninit() = 0;

};


#endif