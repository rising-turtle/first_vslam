#ifndef IOT_ROBOT_RGBDSLAM_INTERFACE_H
#define IOT_ROBOT_RGBDSLAM_INTERFACE_H

#include "Iot_Robot_Slam_Interface.h"

#include <iostream>
#include <string>
#include <boost/shared_ptr.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include "graph_manager.h"
#include "CPose3D.h"
#include "Viewer.h"

class Iot_Robot_rgdbslam_Interface : public Iot_Robot_Slam_Interface
{
public:
	Iot_Robot_rgdbslam_Interface(const string& detectorType, const string& descriptorType, const string& descriptorMatcherType){
		setDetector(detectorType);
		setExtractor(descriptorType);
		setMatcher(descriptorMatcherType);
	}
	~Iot_Robot_rgdbslam_Interface(){}

	// overload init() uninit() runoneframe()
	void init(){}
	void uninit(){}
	void runoneframe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame);

	// parameters for feature extractor
	void setDetector(const string& detectorType){
		m_detector = cv::FeatureDetector::create(detectorType);
	};
	void setExtractor(const string& descriptorType){
		m_extractor = cv::DescriptorExtractor::create(descriptorType);
	};
	void setMatcher(const string& descriptorMatcherType){
		m_matcher = cv::DescriptorMatcher::create(descriptorMatcherType);
	};

	////Interfaces of opencv SIFT/SURF/FAST/STAR/GFTT algorithm
	//cv::FeatureDetector* createDetector( const string& detectorType );
	//cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );
	Node* createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, const cv::Mat& depth = cv::Mat());
	bool computeNodePose(Node* node_ptr, double* xyz, double* rpy);

	// whether this pose is good: not noise nor too small
	bool IsBigTra(CPose3D & pose);

	// split CPointCloud into image and depth_image metadata
	void getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
		unsigned char* rgbbuf,
		unsigned char* depthbuf
		);

	//OpenCV variables 
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;

public:
	// Record pose queue
	vector<CPose3D> robotpath;

	// Map builder
	boost::shared_ptr<MapBuilder> pMapbuilder;

private:
	//
	GraphManager m_graph_mgr;

};



#endif