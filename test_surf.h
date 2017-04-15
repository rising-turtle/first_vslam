#pragma once

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
#include "CLogfile.h"
//#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;

typedef struct _StKinectFrame{
	typedef unsigned long STTimeStamp;
	boost::shared_ptr<openni_wrapper::DepthImage> depth_image;
	boost::shared_ptr<openni_wrapper::Image> image;
	boost::shared_ptr<pointcloud_type> pcl;
	double	timestamp;
	void genTimestamp(){
		timestamp = pcl::getTime();
	}
	pcl::PointXYZRGB getPointByVImage(unsigned x, unsigned y);
	pcl::PointXYZRGB getPointByDImage(unsigned x, unsigned y);
} StKinectFrame;

class CSlam{
	virtual bool getKinectFrame(StKinectFrame& kinectFrame)=0;
};

//class CPointMap : public PointCloud<pcl::PointXYZRGB>{
//	virtual bool loadFromKinectFrame(StKinectFrame& kinectFrame)=0;
//};

//class CEigenMap : public CPointMap{
//	bool loadFromKinectFrame(StKinectFrame& kinectFrame){
//		//Get XYZGRB from rgb image and depth image
//	};
////	std::vector<pcl::surface::NVector> m_NVectorList;
//};

class CVisualSlam: public CSlam{

public:
	//
	bool getKinectFrame(StKinectFrame& kinectFrame){
		m_kinectFrame = kinectFrame; 
		return true;
	};

	CVisualSlam(const string& detectorType, const string& descriptorType, const string& descriptorMatcherType){
		setDetector(detectorType);
		setExtractor(descriptorType);
		setMatcher(descriptorMatcherType);
	};

	void setDetector(const string& detectorType){
		if( !detectorType.compare( "SURF" ) ) {
			m_detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(),
				global_adjuster_min_keypoints,
				global_adjuster_max_keypoints,
				global_surf_adjuster_max_iterations);
		}else if(!detectorType.compare( "FAST")){
			m_detector = new cv::DynamicAdaptedFeatureDetector (new cv::FastAdjuster(20,true), 
				global_adjuster_min_keypoints,
				global_adjuster_max_keypoints,
				global_fast_adjuster_max_iterations);
		}else
			m_detector = cv::FeatureDetector::create(detectorType);
	};
	/*void setDetector(const string& detectorType){
		if( !detectorType.compare( "SURF" ) ) {
			m_detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(),
				global_adjuster_min_keypoints,
				global_adjuster_max_keypoints,
				global_surf_adjuster_max_iterations);
		}else
			m_detector = cv::FeatureDetector::create(detectorType);
	};*/

	void setExtractor(const string& descriptorType){
		m_extractor = cv::DescriptorExtractor::create(descriptorType);
	};
	void setMatcher(const string& descriptorMatcherType){
		m_matcher = cv::DescriptorMatcher::create(descriptorMatcherType);
	};

	// display transformation matrix
	void displayResult(boost::shared_ptr<CPose3D> & pose);
	void run();        // execute without Hogman structure
	void runHogman();  // with Hogman
	void runHogmanExperiment(); // Hogman refresh all Cell Map
	void runFeature(); // Only display Feature Points

	// Extract visual feature points
	//void extractVisualFeatures(const cv::Mat& visual , const cv::Mat& detection_mask , 
	//	std::vector<cv::KeyPoint>& feature_locations_2d);

	////Interfaces of opencv SIFT/SURF/FAST/STAR/GFTT algorithm
	//cv::FeatureDetector* createDetector( const string& detectorType );
	//cv::DescriptorExtractor* createDescriptorExtractor( const string& descriptorType );
	Node* createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, const cv::Mat& depth = cv::Mat());
	bool computeNodePose(Node* node_ptr, double* xyz, double* rpy);

	// get Images and Depth from pointXYZRGB
	void getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
		unsigned char* rgbbuf,
		unsigned char* depthbuf
		);


	// add matched features into pre-frame and cur-frame
	void addFeatureToImage(Node* preNode, unsigned char* l_rgb_buff,Node* pNode, unsigned char* rgb_buff,MatchingResult& mr);

	// add feature to image
	void addFeatureToImage(Node* pNode, unsigned char* rgb_buff);
	void getFileName(int frame_num, std::string & filename);
	// save rgb_buff into bmp
	void saveImgeToBmp(std::string& filename,int frame_num, unsigned char* rgb_buff);
	// convert to opencv image format
	void convertToIplImage(boost::shared_ptr<IplImage> pImage,unsigned char * rgb_buff);
	// change sequence of rgb_flow
	void fromrgbtobgr(unsigned char* rgb_buff, int len);

	// whether pose is good
	bool IsBigTra(CPose3D & pose);
	
	// up limitation for noisy translation
	bool IsNoiseLocation(CPose3D& lastpose,CPose3D& currentpose );

	// search all robotpath to obtain a valid match
	int SearchAllPath(Node* new_node,boost::shared_ptr<CPose3D>& fpose);

	// just for test file
	void test();

	//OpenCV variables 
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;

public:
	// Record pose queue
	vector<CPose3D> robotpath;
	
	// This is for recording slam-time-consumed
	CLogfile mylogfile;
	
	// Map builder
	//boost::shared_ptr<MapBuilder> pMapbuilder;

private:
	//Kinect data variables
	StKinectFrame m_kinectFrame;
	//
	GraphManager m_graph_mgr;
};

