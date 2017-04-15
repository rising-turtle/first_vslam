#include "test_surf.h"
#include "Openni.h"
#include <boost/array.hpp>
#include <boost/shared_array.hpp>
#include <boost/dynamic_bitset.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include "pcl/visualization/pcl_visualizer.h"

#include <pcl/surface/mls.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include "pcl/features/normal_3d.h"

#include "IoTRobot_NetServer.h"
#include "AreaStore.h"
#include "CSession.h"
#include "CMyNode.h"
#include "CMyGraphManager.h"
#include "FPFHNode.h"

#include "TestSet.h"

#define pi 3.141592654
#define R2D(r) ((r)/pi * 180)
#define D2R(d) ((d)*pi/180)

#define features_threshold 50	// if a new frame has less than this,then drop it!
#define search_range 2			// back search units

//#define NETSERVER
//
//using namespace cv;
// Reference to openni_listener in rgbdslam application
//FeatureDetector* CVisualSlam::createDetector( const string& detectorType ) {
//	FeatureDetector* fd = 0;
//    if( !detectorType.compare( "FAST" ) ) {
//        //fd = new FastFeatureDetector( 20/*threshold*/, true/*nonmax_suppression*/ );
//        fd = new DynamicAdaptedFeatureDetector (new FastAdjuster(20,true));
//
////												params->get<int>("adjuster_min_keypoints"),
////												params->get<int>("adjuster_max_keypoints"),
////												params->get<int>("fast_adjuster_max_iterations"));
//    }
//    else if( !detectorType.compare( "STAR" ) ) {
//        fd = new StarFeatureDetector( 16/*max_size*/, 5/*response_threshold*/, 10/*line_threshold_projected*/,
//                                      8/*line_threshold_binarized*/, 5/*suppress_nonmax_size*/ );
//    }
//    else if( !detectorType.compare( "SIFT" ) ) {
//        fd = new SiftFeatureDetector(SIFT::DetectorParams::GET_DEFAULT_THRESHOLD(),
//                                     SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());
//    }
//    else if( !detectorType.compare( "SURF" ) ) {
//		fd = new DynamicAdaptedFeatureDetector(new SurfAdjuster());
////        										params->get<int>("adjuster_min_keypoints"),
////												params->get<int>("adjuster_max_keypoints"),
////												params->get<int>("surf_adjuster_max_iterations"));
//    }
//    else if( !detectorType.compare( "MSER" ) ) {
//        fd = new MserFeatureDetector( 1/*delta*/, 60/*min_area*/, 14400/*_max_area*/, 0.35f/*max_variation*/,
//                0.2/*min_diversity*/, 200/*max_evolution*/, 1.01/*area_threshold*/, 0.003/*min_margin*/,
//                5/*edge_blur_size*/ );
//    }
//    else if( !detectorType.compare( "GFTT" ) ) {
//        fd = new GoodFeaturesToTrackDetector( 200/*maxCorners*/, 0.001/*qualityLevel*/, 1./*minDistance*/,
//                                              5/*int _blockSize*/, true/*useHarrisDetector*/, 0.04/*k*/ );
//    }
//    else {
//      fd = createDetector("SURF"); //recursive call with correct parameter
//    }
//    return fd;
//}

//DescriptorExtractor* CVisualSlam::createDescriptorExtractor( const string& descriptorType ) {
//    DescriptorExtractor* extractor = 0;
//    if( !descriptorType.compare( "SIFT" ) ) {
//        extractor = new SiftDescriptorExtractor();/*( double magnification=SIFT::DescriptorParams::GET_DEFAULT_MAGNIFICATION(), bool isNormalize=true, bool recalculateAngles=true, int nOctaves=SIFT::CommonParams::DEFAULT_NOCTAVES, int nOctaveLayers=SIFT::CommonParams::DEFAULT_NOCTAVE_LAYERS, int firstOctave=SIFT::CommonParams::DEFAULT_FIRST_OCTAVE, int angleMode=SIFT::CommonParams::FIRST_ANGLE )*/
//    }
//    else if( !descriptorType.compare( "SURF" ) ) {
//        extractor = new SurfDescriptorExtractor();/*( int nOctaves=4, int nOctaveLayers=2, bool extended=false )*/
//    }
//    else {
//      extractor = createDescriptorExtractor("SURF");
//    }
//    return extractor;
//}

//void CVisualSlam::extractVisualFeatures(const cv::Mat& visual , const cv::Mat& detection_mask , std::vector<cv::KeyPoint>& feature_locations_2d){
//	m_detector->detect( visual, feature_locations_2d, detection_mask);// fill 2d locations	
//	cv::Mat feature_descriptors_;
//	m_extractor->compute(visual, feature_locations_2d, feature_descriptors_);
//}

Node* CVisualSlam::createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, 
									 const cv::Mat& depth){
	Node* node_ptr = new Node(visual, m_detector, m_extractor, m_matcher,
		point_cloud, depth);
	return node_ptr;
}

bool CVisualSlam::IsNoiseLocation(CPose3D& lastpose,CPose3D& currentpose )
{
	double uplimit_t = 0.4;
	double lxyz[3],cxyz[3];
	currentpose.getXYZ(cxyz);
	lastpose.getXYZ(lxyz);
	if(fabs(lxyz[0] - cxyz[0]) > uplimit_t || fabs(lxyz[1] - cxyz[1]) > uplimit_t || fabs(lxyz[2] - cxyz[2]) > uplimit_t)
		return true;
	return false;
}

bool CVisualSlam::computeNodePose(Node* node_ptr, double* xyz, double* rpy){

	if(m_graph_mgr.addNode2(node_ptr))
	{
		xyz[0] = m_graph_mgr.latest_pose[0];
		xyz[1] = m_graph_mgr.latest_pose[1];
		xyz[2] = m_graph_mgr.latest_pose[2];

		rpy[0] = m_graph_mgr.latest_pose[3];
		rpy[1] = m_graph_mgr.latest_pose[4];
		rpy[2] = m_graph_mgr.latest_pose[5];

		return true;
	}
	else
		return false;
}


void CVisualSlam::displayResult(boost::shared_ptr<CPose3D> & pose){
	
	Eigen::Matrix4f final_transformation;
	pose->getHomogeneousMatrix(final_transformation);

	// Print the rotation matrix and translation vector
	Eigen::Matrix3f rotation = final_transformation.block<3,3>(0, 0);
	Eigen::Vector3f translation = final_transformation.block<3,1>(0, 3);

	printf("value by align");
	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));
}
void showxyzrpy(double xyz[3],double rpy[3]){

	cout<<"(x,y,z)" <<"("<<xyz[0]<<","<<xyz[1]<<","<<xyz[2]<<")" \
		<<"(r,p,y)" <<"("<<R2D(rpy[0])<<","<<R2D(rpy[1])<<","<<R2D(rpy[2])<<")"<<endl;
}

void CVisualSlam::getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
							   unsigned char* rgbbuf,
							   unsigned char* depthbuf
							   )
{
	unsigned short * pdepth =(unsigned short*) (depthbuf);
	unsigned char  * pimage = rgbbuf;  
	unsigned int totalnum = point_cloud->width * point_cloud->height;

	float bad_point = std::numeric_limits<float>::quiet_NaN();


	for(size_t i=0;i<totalnum;i++){
		
		pcl::PointXYZRGB& pt = point_cloud->points[i];
		// get rgb-info 
		*pimage = pt.r;
		pimage++;
		*pimage = pt.g;
		pimage++;
		*pimage = pt.b;
		pimage++;

		// get depth-info
		if(pt.x == bad_point && pt.y == bad_point && pt.z == bad_point){
			*pdepth = 0;
		}
		else
		{
			*pdepth = pt.z * 1000.0f;
		}
		pdepth ++;
	}
}

// add feature to image
void CVisualSlam::addFeatureToImage(Node* pNode, unsigned char* rgb_buff){
	
	int index =0;
	cv::Point2f p2d;
	for(unsigned int i = 0; i < pNode->feature_locations_2d_.size(); i++){
		p2d = pNode->feature_locations_2d_[i].pt;
		if (p2d.x >= pNode->pc_col.width  || p2d.x < 0 ||
			p2d.y >= pNode->pc_col.height || p2d.y < 0 ||
			_isnan(p2d.x) || _isnan(p2d.y)){ //TODO: Unclear why points should be outside the image or be NaN
				continue;
		}

		// get location of feature point
		index = (int)p2d.y * pNode->pc_col.width + (int)p2d.x;
		index *= 3;
		// set this point black
		rgb_buff[index] = 0;
		rgb_buff[index+1] = 0;
		rgb_buff[index+2] =0;
	}
}

// add matched features into pre-frame and cur-frame
void CVisualSlam::addFeatureToImage(Node* preNode, unsigned char* l_rgb_buff,Node* pNode, unsigned char* rgb_buff,MatchingResult& mr){
	cv::Point2f lpt,cpt; // point at last frame and current frame
	int index ;
	for(std::vector<cv::DMatch>::iterator it = mr.inlier_matches.begin();
			it!= mr.inlier_matches.end(); it++)
		{
			 cpt = pNode->feature_locations_2d_[(*it).queryIdx].pt; // point at current-frame
			 lpt = preNode->feature_locations_2d_[(*it).trainIdx].pt; // point at pre-frame
			 // check validity
			 if (cpt.x >= pNode->pc_col.width  || cpt.x < 0 ||
				 cpt.y >= pNode->pc_col.height || cpt.y < 0 ||
				 _isnan(cpt.x) || _isnan(cpt.y)){ //TODO: Unclear why points should be outside the image or be NaN
					 continue;
			 }
			 if (lpt.x >= preNode->pc_col.width  || lpt.x < 0 ||
				 lpt.y >= preNode->pc_col.height || lpt.y < 0 ||
				 _isnan(lpt.x) || _isnan(lpt.y)){ //TODO: Unclear why points should be outside the image or be NaN
					 continue;
			 }
			 // get location of feature point at current frame
			 index = (int)cpt.y * pNode->pc_col.width + (int)cpt.x;
			 index *= 3;
			 // set this point black
			 rgb_buff[index] = 0;
			 rgb_buff[index+1] = 0;
			 rgb_buff[index+2] =0;

			 // get location of feature point at previous frame
			 index = (int)lpt.y * preNode->pc_col.width + (int)lpt.x;
			 index *= 3;
			 // set this point black
			 l_rgb_buff[index] = 0;
			 l_rgb_buff[index+1] = 0;
			 l_rgb_buff[index+2] =0;
	}

}
void CVisualSlam::getFileName(int frame_num, std::string & filename){
	static string path_name("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\Nodefeatures\\");
	static string bmp(".bmp");
	static string file_name("frame");
	char num_str[10];
	itoa(frame_num,num_str,10);
	string tmp(num_str);
	// clear already name
	filename.clear();
	filename = path_name + file_name + tmp + bmp;
	return ;
}
// convert to opencv image format
void CVisualSlam::convertToIplImage(boost::shared_ptr<IplImage> pImage,unsigned char * rgb_buff){
	
	pImage->width = 640;
	pImage->height = 480;
	pImage->widthStep = pImage->width * 3;
	pImage->imageSize = pImage->height * pImage->widthStep;
	pImage->dataOrder = 0; // interleaved
	
	pImage->ID = 0;
	pImage->nChannels = 3;
	pImage->depth = IPL_DEPTH_8U;
	pImage->origin = 0; // windows style top-left
	
	// must be null
	pImage->maskROI = NULL; 
	pImage->roi = NULL;
	pImage->imageData = (char*)rgb_buff;
	pImage->nSize = sizeof(*(pImage.get()));

	//pImage->imageDataOrigin = (char*)rgb_buff;
}
// change sequence of rgb_flow
void CVisualSlam::fromrgbtobgr(unsigned char* rgb_buff, int len){
	 unsigned char tmp;
	 unsigned char* pr = rgb_buff;
	 for(int i=0;i<len; i+=3)
	 { 
		 // from rgb to bgr 
		 tmp = *(pr+i);
		 *(pr+i) = *(pr+i+2);
		 *(pr+i+2) = tmp;
	 }
	return ;
}
// save rgb_buff into bmp
void CVisualSlam::saveImgeToBmp(std::string& filename,int frame_num, unsigned char* rgb_buff){	
	fromrgbtobgr(rgb_buff,640*480*3);
	getFileName(frame_num,filename);
	boost::shared_ptr<IplImage> pImage(new IplImage);
	convertToIplImage(pImage,rgb_buff);
	//cvSaveImage(filename.c_str(),rgb_buff);
	cvSaveImage(filename.c_str(),pImage.get());
}

// whether pose is good
bool CVisualSlam::IsBigTra(CPose3D & pose){
	if(fabs(pose.m_coords[0]) > 2 || \
		fabs(pose.m_coords[1]) > 2 || \
		fabs(pose.m_coords[2]) > 2 || \
		fabs(R2D(pose.yaw)) > 50 || \
		fabs(R2D(pose.pitch)) > 50 || \
		fabs(R2D(pose.roll)) > 50 )
			return false;
	if(fabs(pose.m_coords[0]) > 0.06 || \
		fabs(pose.m_coords[1]) > 0.06 || \
		fabs(pose.m_coords[2]) > 0.06 || \
		fabs(R2D(pose.yaw)) > 5 || \
		fabs(R2D(pose.pitch)) > 5 || \
		fabs(R2D(pose.roll)) > 5 )
		return true;
	return false;
}

// search all robotpath to obtain a valid match
int CVisualSlam::SearchAllPath(Node* new_node,boost::shared_ptr<CPose3D>& fpose){
	// threshold for matched pairs
	static int valid_numnber_of_matches = 30;
	// search robot_path to find most similar Node
	for(size_t i=1 ;i< search_range/*= m_graph_mgr.graph_.size()*/; i++)
	{
		Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-i];
		MatchingResult mr = new_node->matchNodePair(pre_node);
		if(mr.inlier_matches.size() >= valid_numnber_of_matches) // this is valid match
		{
			boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); // relative Pose to previous frames
			fpose = pose;
			return i;
		}
			
	}
	// failed to find a similar Node
	return -1;
}

bool g_bDataRdy=false;
bool g_bProData=false;
unsigned char *g_RGB;
unsigned short *g_Depth;
void CallBack_SLAM(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext)
{

}

void CallBack_SLAM_IMU(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext)
{
//	if (!g_bProData)
	{
		memcpy(g_RGB,pucRGB,640*480*3);
		memcpy(g_Depth,pusDepth,640*480*2);
	//	g_RGB=pucRGB;
	//	g_Depth=pusDepth;
		g_bDataRdy=1;
	}

}
#define CLNUI_DEPTH_TABLE_MASK 2048

double g_dDepthTable[CLNUI_DEPTH_TABLE_MASK];
void IoT_GenerateConvertTable()
{
	int i,nIdx;
	for (i=0;i<CLNUI_DEPTH_TABLE_MASK;i++)
	{
		nIdx=i%(CLNUI_DEPTH_TABLE_MASK-1);
		g_dDepthTable[i]=1.0f / (nIdx * (-0.0030711016) + 3.3309495161);
	}
}


void IoT_ConvertToXYZRGBPointCloud_CLNUI(unsigned char*pucRGB,unsigned short *pusDepth,
										 boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
{
	cloud->header.frame_id = "/openni_rgb_optical_frame";;
	cloud->height = 480;
	cloud->width = 640;
	cloud->is_dense = false;
	int nIdx;

	cloud->points.resize(cloud->height * cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	register const XnDepthPixel* depth_map = pusDepth;
	register int color_idx = 0, depth_idx = 0;


	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == CLNUI_DEPTH_TABLE_MASK-1 )
			{
				pt.x = pt.y = pt.z = bad_point;
				pt.r=pucRGB[color_idx];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx+2];
			}
			else
			{
				pt.z=g_dDepthTable[pusDepth[depth_idx]];
				pt.x=0.001904*u *pt.z;
				pt.y=0.001904*v *pt.z;
				pt.r=pucRGB[color_idx+2];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx];

			}
		}
	}
}



void IoT_ConvertToXYZRGBPointCloud(unsigned char*pucRGB,unsigned short *pusDepth,
								   boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > cloud)
{
//	cloud->header.frame_id = "/openni_rgb_optical_frame";;
	cloud->height = 480;
	cloud->width = 640;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);
	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);
	register const XnDepthPixel* depth_map = pusDepth;
	register int color_idx = 0, depth_idx = 0;


	float bad_point = std::numeric_limits<float>::quiet_NaN();

	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, color_idx += 3, ++depth_idx)
		{
			pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth_map[depth_idx] == 0 )
			{
				pt.x = pt.y = pt.z = bad_point;
				pt.r=pucRGB[color_idx];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx+2];
			}
			else
			{
				pt.z=pusDepth[depth_idx] * 0.001f;
				pt.x=0.001904*u *pt.z;
				pt.y=0.001904*v *pt.z;
				pt.r=pucRGB[color_idx];
				pt.g=pucRGB[color_idx+1];
				pt.b=pucRGB[color_idx+2];

			}
		}
	}
}
IoTRobot_NetServer g_CNetServer;
bool g_bNetRu=false;
UINT ThreadServer(LPVOID lpParam)
{
	g_CNetServer.NetServer_Run();
//	g_bNetRu=true;
	return 0;
}
void CVisualSlam::runHogmanExperiment(){

#ifdef NETSERVER
	IoT_NetCallBackSet stCallbackSet;
	memset(&stCallbackSet,0,sizeof(IoT_NetCallBackSet));
	stCallbackSet.cbSlam_IMU=CallBack_SLAM_IMU;
	stCallbackSet.cbSlam=CallBack_SLAM;
	g_Depth=new unsigned short[640*480];
	g_RGB=new unsigned char[640*480*3];
	LPDWORD ID=0;
	g_CNetServer.NetServer_Init(stCallbackSet);
	IoT_GenerateConvertTable();
	
//	g_CNetServer.NetServer_Run();


	IoT_NetConf stMetConf;
	stMetConf.stSLAMConf.cCompression=0;
	stMetConf.stSLAMConf.cFrequency=25;
	stMetConf.stSLAMConf.cResolution=0;
	stMetConf.stSLAMConf.cType=2;

	stMetConf.stVisulizationConf.cCompression=0;
	stMetConf.stVisulizationConf.cFrequency=60;
	stMetConf.stVisulizationConf.cResolution=0;
	stMetConf.stVisulizationConf.cType=0;
CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)ThreadServer,NULL,0,ID);
	Sleep(1000);
	while (g_CNetServer.NetServer_Conf(0,stMetConf)!=0)
	{
		Sleep(30);
	}
#endif

	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//point_cloud = new <>

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

#ifdef NETSERVER
#else
	// 1. Initialize OpenNI Grabber and Mapbuilder
	SimpleOpenNIViewer v;
	v.startframe();
	v.start();
#endif
	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();


	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	double start_t_slam, end_t_slam;  // record SLAM time-consuming
	char buf[100]; // for receive char* from sprintf

	// check every step of SLAM
	double data_start_t, data_end_t, fd_start_t,fd_end_t;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pGlobalMap(new pcl::PointCloud<pcl::PointXYZRGB>);

	while(1){
#ifdef NETSERVER
		while(1)
#else
		while(v.getCloudPoint( point_cloud))
#endif
		{
#ifdef NETSERVER
			g_CNetServer.NetServer_GetOneSlamFrame(0);

			while(!g_bDataRdy)
			{
				Sleep(5);
			}
			//IoT_ConvertToXYZRGBPointCloud(g_RGB,g_Depth,point_cloud);
			IoT_ConvertToXYZRGBPointCloud_CLNUI(g_RGB,g_Depth,point_cloud);
			g_bDataRdy=false;
			g_bProData=true;
#endif
			// record start time of SLAM
			start_t_slam = ::GetTickCount();

			// initialize log parameters
			gl_pf_gs=0; // graph_size
			gl_pf_fe=0;	// time for Feature Extraction
			gl_pf_fm=0; // time for Feature Match
			gl_pf_id=0; // time for Inliers Detection
			gl_pf_me=0; // time for Motion Estimation
			gl_pf_slam=0; // time for total Slam


			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);


			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			// 3. Create node to extract feature and wrap the image and feature data
			fd_start_t = ::GetTickCount();
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);
			gl_pf_fe = ::GetTickCount()-fd_start_t;

			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}

			// 4. Insert node into node-graph to compute the pose of the node
			double xyz[3]={0,0,0};
			double rpy[3]={0,0,0};

			if(computeNodePose(node_ptr,xyz,rpy)){

				// Record current pose info
				CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);
				boost::shared_ptr<CPose3D> final_pose(new CPose3D(pose));
				robotpath.push_back(pose);

				std::map<int, Node* >::reverse_iterator vertex_iter_graph = m_graph_mgr.graph_.rbegin();

				int id_of_node = vertex_iter_graph->second->id_;
				cout<<"!!!Set id_of_node: "<<id_of_node<<endl;

				gl_pf_slam = ::GetTickCount() - start_t_slam;

				mylogfile.writeid(frame_num++); // id of frame
				double graphsize = (double)m_graph_mgr.graph_.size();
				mylogfile.writeintolog(graphsize); // graph size
				mylogfile.writeintolog(gl_pf_fe); // FE
				mylogfile.writeintolog(gl_pf_fm); // FM
				mylogfile.writeintolog(gl_pf_id); // ID
				mylogfile.writeintolog(gl_pf_me); // ME
				mylogfile.writeintolog(gl_pf_slam,true); // Total Slam

				// save this frame into files for debug
			/*	string filename;
				static int num_frame = 0;
				addFeatureToImage(node_ptr,rgb_buffer);
				saveImgeToBmp(filename,++num_frame,rgb_buffer);*/

				pMapbuilder->cloudRegistration(final_pose,point_cloud);

				// Send point_cloud and pose to Mapbuilder
				while(!pMapbuilder->setGlobalMapForExperiment(point_cloud,pose,id_of_node)){
					boost::this_thread::yield();
				}
				
				// delete pc in Node
				//node_ptr->pc_col.

				static int num_of_nodes = -1;
				if(++num_of_nodes > 20){
					num_of_nodes = -1;
					cout<<"!!! Clean the Cells in Mapbuilder!!!"<<endl;
					// update all path node's position
					// transformation all point cloud from Nodes
					// Merge all pc into a global_pc
					std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = m_graph_mgr.optimizer_->vertices().begin();
					std::map<int, Node* >::iterator vertex_iter_graph = m_graph_mgr.graph_.begin();
					pcl::PointCloud<point_type>::Ptr pCloud;

					//pGlobalMap.reset(new pcl::PointCloud<pcl::PointXYZRGB>); // clear pGlobalMap
					for(vector<CPose3D>::iterator node_pose=robotpath.begin(); node_pose!=robotpath.end() && vertex_iter_graph!=m_graph_mgr.graph_.end();\
						node_pose++,vertex_iter++,vertex_iter_graph++)
					{
						AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
						double x1=v_to_del->transformation.translation().x();
						double y1=v_to_del->transformation.translation().y();
						double z1=v_to_del->transformation.translation().z();
						_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
						double roll = rpy.roll();
						double pitch = rpy.pitch();
						double yaw = rpy.yaw();
						node_pose->setrpy(roll,pitch,yaw);
						node_pose->setxyz(x1,y1,z1);
						boost::shared_ptr<CPose3D> updated_pose(new CPose3D(*node_pose));
						//*(pCloud.get()) = vertex_iter_graph->second()->pc_col;
						//pMapbuilder->cloudRegistration(updated_pose,&((vertex_iter_graph->second)->pc_col));
						//pMapbuilder->MergeLMapwithGMap(pGlobalMap.get(),&((vertex_iter_graph->second)->pc_col));
						pMapbuilder->transformPointsofNode(vertex_iter->second->id(),*updated_pose);
/*
						if(vertex_iter_graph->second->pc_col.points.size()!=0)
						{
							cout<<"!!!Clear memory allocated by Node "<<vertex_iter_graph->second->id_<<endl;
							vertex_iter_graph->second->pc_col.points.swap(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >());
						}
						*/
						//pMapbuilder->MergeLMapwithGMap(pGlobalMap.get(),&((vertex_iter->second)->pc_col));
					}
						// redraw Cells
						pMapbuilder->CleanCells();
					//	pMapbuilder->RefreshCells();
				}	
			}
			else{
				cout<<"Abandon this node!"<<endl;
				delete node_ptr;
				continue;
			}	
			g_bProData=false;
		}
	}
}
void CVisualSlam::runHogman(){
	
	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// 1. Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
	//v.startframe();
	v.start();

	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();


	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	double start_t_slam, end_t_slam;  // record SLAM time-consuming
	char buf[100]; // for receive char* from sprintf

	// check every step of SLAM
	double data_start_t, data_end_t, fd_start_t,fd_end_t;

	while(1){
		while(v.getCloudPoint( point_cloud))
		{

			// record start time of SLAM
			start_t_slam = ::GetTickCount();

			// initialize log parameters
			gl_pf_gs=0; // graph_size
			gl_pf_fe=0;	// time for Feature Extraction
			gl_pf_fm=0; // time for Feature Match
			gl_pf_id=0; // time for Inliers Detection
			gl_pf_me=0; // time for Motion Estimation
			gl_pf_slam=0; // time for total Slam

			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);


			fd_start_t = ::GetTickCount();
			// 3. Create node to extract feature and wrap the image and feature data
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);
			// record time cost by FE
			gl_pf_fe = ::GetTickCount() - fd_start_t; 

			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}

			// 4. Insert node into node-graph to compute the pose of the node
			double xyz[3]={0,0,0};
			double rpy[3]={0,0,0};
			
			if(computeNodePose(node_ptr,xyz,rpy)){
				static int num_of_node=-1;
				if(num_of_node++>10)
				{
					cout<<"!!! Clean the Cells in Mapbuilder!!!"<<endl;
					pMapbuilder->CleanCells();
					num_of_node = 0;
				}

					// Record current pose info
					CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);
									
					if(robotpath.size()==0 || !IsNoiseLocation(*robotpath.rbegin(),pose)){
						//cout<<"final _pose in RobotPath!"<<endl;					
						boost::shared_ptr<CPose3D> final_pose(new CPose3D(pose));
						robotpath.push_back(pose);
						
						//cout<<"3 after SLAM!"<<endl;
						//pose.output(std::cout);

						// Logfile for slam_process
						end_t_slam = ::GetTickCount(); // end of SLAM for a valid frame
						gl_pf_slam = end_t_slam - start_t_slam;
						
						mylogfile.writeid(frame_num++); // id of frame
						double graphsize = (double)m_graph_mgr.graph_.size();
						//mylogfile.writeintolog(graphsize); // graph size
						//mylogfile.writeintolog(gl_pf_fe); // FE
						//mylogfile.writeintolog(gl_pf_fm); // FM
						//mylogfile.writeintolog(gl_pf_id); // ID
						//mylogfile.writeintolog(gl_pf_me); // ME
						//mylogfile.writeintolog(gl_pf_slam,true); // Total Slam
	
						// Send point_cloud and pose to Mapbuilder
						while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose)){
							boost::this_thread::yield();
						}
				}
					// noisy translation
					else{
						// delete last noisy frame
						m_graph_mgr.deleteLastFrame();
						cout<<"delete noisy plane!"<<endl;
						delete node_ptr;						
						continue;
					}
			}
			else{
				cout<<"Abandon this node!"<<endl;
				delete node_ptr;
				continue;
			}		
		}
	}
		
}
void CVisualSlam::runFeature()
{
	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// 1. Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
	//v.startframe();
	v.start();

	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();


	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	char buf[100]; // for receive char* from sprintf

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pFPs(new pcl::PointCloud<pcl::PointXYZRGB>);

	while(1){
		while(v.getCloudPoint(point_cloud))
		{
			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			// 3. Create node to extract feature and wrap the image and feature data
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);

			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}

			// 4. Insert node into node-graph to compute the pose of the node
			double xyz[3]={0,0,0};
			double rpy[3]={0,0,0};

			if(computeNodePose(node_ptr,xyz,rpy)){
				static int num_of_node=-1;

				// Record current pose info
				CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);

				if(robotpath.size()==0 || !IsNoiseLocation(*robotpath.rbegin(),pose)){
					
					m_pFPs->points.clear();
					std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& FPs=node_ptr->feature_locations_3d_;
					for(size_t i=0;i<FPs.size();i++)
					{
						pcl::PointXYZRGB pt;
						pt.x=FPs[i][0];
						pt.y=FPs[i][1];
						pt.z=FPs[i][2];
						m_pFPs->points.push_back(pt);
					}

					boost::shared_ptr<CPose3D> final_pose(new CPose3D(pose));
					robotpath.push_back(pose);
					// Send point_cloud and pose to Mapbuilder
					//while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose))
					while(!pMapbuilder->setrawmapwithtransform(m_pFPs,final_pose)){
						boost::this_thread::yield();
					}
				}
				// noisy translation
				else{
					// delete last noisy frame
					m_graph_mgr.deleteLastFrame();
					cout<<"delete noisy plane!"<<endl;
					delete node_ptr;						
					continue;
				}
			}
			else{
				cout<<"Abandon this node!"<<endl;
				delete node_ptr;
				continue;
			}		
		}
	}
}

void CVisualSlam::run(){

	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// 1. Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
	//v.startframe();
	v.start();

	boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);

	// Get OpenNI visual image
	
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;

	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();
	
	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;

	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// record num of frames
	static int frame_num = 0;
	double start_t_slam, end_t_slam;  // record SLAM time-consuming
	char buf[100]; // for receive char* from sprintf

	while(1){
		while(v.getCloudPoint( point_cloud))
		{
			/*xn::ImageMetaData xnImd;
			xnImd.CopyFrom(openni_image->getMetaData());*/

			// record start time of SLAM
			start_t_slam = ::GetTickCount();

			// get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);
			//openni_depth_image->fillDepthImageRaw(640,480,depth_buffer) ;
			//openni_image->fillRGB(640,480,rgb_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);

			//memcpy(cvGrayImage.data, xnImd.Grayscale8Data(), 640*480*1);
			//memcpy(cvDepthImage.data, xnDmd.Data(), 640*480*2);
			// Convert RGB image to grayscale image
			cvRGBImage.convertTo(cvGrayImage, CV_8UC1);
			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			// 3. Create node to extract feature and wrap the image and feature data
			//Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);
			Node* node_ptr = createFeatureNode(cvGrayImage, point_cloud, cvDepth8UC1Image);

			// filter frame that obtains too small features
			if(node_ptr->feature_locations_2d_.size() < features_threshold){
				cout<<" this frame has too small features! drop it!"<<endl;
				continue;
			}

			// 4. Insert node into node-graph to compute the pose of the node
			//double xyz[3]={0,0,0};
			//double rpy[3]={0,0,0};
			//needs to be done here as the graph size can change inside this function
			node_ptr->id_ = m_graph_mgr.graph_.size();

			bool isGoodTra = false;

			boost::shared_ptr<CPose3D> final_pose(new CPose3D());

			if(robotpath.size() == 0) // first frame
			{
				node_ptr->buildFlannIndex();
				m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
				robotpath.push_back( CPose3D());
				isGoodTra = true;
			}
			else{
				// search robot_path to find most similar Node
				//Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-1];
				//MatchingResult mr = node_ptr->matchNodePair(pre_node);
				//boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); // relative Pose to previous frame

				// find pose relative to robot_path[index] 
				boost::shared_ptr<CPose3D> pose(new CPose3D);
				int index = SearchAllPath(node_ptr,pose); 
				if(index == -1) 
				{
					cout<<"failed to find a valid match from robot_path!"<<endl;
					delete node_ptr;
					continue;

				}
				if(IsBigTra(*pose)){
					isGoodTra = true; // This is good transformation, 
					
					//boost::shared_ptr<CPose3D> prepose = *robotpath.rbegin();
					//CPose3D prepose = *robotpath.rbegin();
					CPose3D prepose = robotpath[robotpath.size() - index];
					*pose += prepose ; // compute pose relative to global coordinates
					pose.get()->output(std::cout);
					final_pose = pose;
	
					// record this location
					node_ptr->buildFlannIndex();
					m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
					robotpath.push_back(*pose);				
				}
				else
					delete node_ptr;
			}
			if(isGoodTra) // will display this frame
			{
				end_t_slam = ::GetTickCount(); // end of SLAM for a valid frame
				sprintf(buf,"%d SLAM :%f\t",frame_num++,end_t_slam-start_t_slam);
				std::string slamstr(buf);
				mylogfile.writeintolog(slamstr);

				// save this frame into files for debug
				/*string filename;
				static int num_frame = 0;
				addFeatureToImage(node_ptr,rgb_buffer);
				saveImgeToBmp(filename,++num_frame,rgb_buffer);*/

				while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose)){
					boost::this_thread::yield();
				}
				isGoodTra = false;
			}

		}
	}
	/*		if(computeNodePose(node_ptr, xyz, rpy)){

				if(robotpath.size() == 0){
					
				}*/
			
	//		// use adapted rgbdslam 

	//		dst_ptr->buildFlannIndex();

	//		MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	//		CPose3D pose(mr.final_trafo);
	//		//cout<<pose;
	//		pose.output(std::cout);

	//			//	string filename;
	//			//	static int num_frame = 0;
	//			//	addFeatureToImage(node_ptr,rgb_buffer);
	//			//	saveImgeToBmp(filename,++num_frame,rgb_buffer);


	//			// after feature match
	//			showxyzrpy(xyz,rpy);


	//			// just for debug

	//			/*static string file_name("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\");
	//			static int num_frame = 0;
	//			char c = '0' + (++num_frame);
	//			string filename = file_name + c +".pcd"; */
	//			// here we will save the cloud_point as for debugging
	//			/*	const pcl::PointCloud<point_type>& py = *point_cloud.get();
	//			pcl::io::savePCDFileBinary<point_type>(filename,*point_cloud.get());*/

	//			

	//			// 5. compute current pose relative to global coordinates
	//			boost::shared_ptr<CPose3D> current(new CPose3D(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]));
	//		//	*current += *robotpath.rbegin(); // this is already global pose info, so, 
	//			robotpath.push_back(*current);

	//			// see what changed from CPose3D
	//			/*current->getYawPitchRoll(rpy[2],rpy[1],rpy[0]);
	//			current->getXYZ(xyz);*/

	//			//showxyzrpy(xyz,rpy);
	//
	//			// show transformation matrix
	//			//displayResult(current);

	//			// 6. set raw points and pose info to map-builder
	//			while(!pMapbuilder->setrawmapwithtransform(point_cloud,current)){
	//				boost::this_thread::yield();
	//			}
	//			//// 6. set translated points
	//			//while(!pMapbuilder->setMap(point_cloud)){
	//			//	boost::this_thread::yield();
	//			//}
	//		}
	//		else
	//			delete node_ptr;
	//	}

	//}

}

void CVisualSlam::test(){
	//string srcf("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\2.pcd");
	string srcf("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\1.pcd");
	string dstf("D:\\PCL_install_on_VS2008\\PCL-No_CMake\\PCL-1.1.1-Source\\PCL-1.1.1-Source\\bin32\\rgdbslam\\featurebmp\\1.pcd");

	boost::shared_ptr<pointcloud_type> src_cloud(new pointcloud_type);
	boost::shared_ptr<pointcloud_type> dst_cloud(new pointcloud_type);
	
	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;

	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// generate Node src
	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,src_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);
	Node* src_ptr = createFeatureNode(cvRGBImage, src_cloud, cvDepth8UC1Image);
	src_ptr->buildFlannIndex();

	// generate Node dst
	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,dst_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);
	Node* dst_ptr = createFeatureNode(cvRGBImage, dst_cloud, cvDepth8UC1Image);
	dst_ptr->buildFlannIndex();

	double xyz[3] = {0,0,0};
	double rpy[3] = {0,0,0};
	
	// using Hogman method! 
	bool bmr = computeNodePose(dst_ptr,xyz,rpy);
	bmr = computeNodePose(src_ptr,xyz,rpy);

//	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	CPose3D pose(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]);
//	CPose3D pose(2*rpy[2],2*rpy[1],2*rpy[0],xyz[0],xyz[1],xyz[2]);
//	CPose3D pose(mr.final_trafo);
	pose.output(std::cout);

	MapBuilder mapbuilder;
	mapbuilder.global_map = dst_cloud;
	mapbuilder.weighted.resize(mapbuilder.global_map->points.size(),1);
	//mapbuilder._pose  = pose;
	boost::shared_ptr<CPose3D> npose(new CPose3D(pose));
	mapbuilder.cloudRegistration(npose,src_cloud);
	mapbuilder.local_map = src_cloud;
	
	//mapbuilder.MergeLocalMap();
	//mapbuilder.viewer.showCloud(mapbuilder.global_map);
	
	mapbuilder.fromPCtoCells(mapbuilder.local_map);
	mapbuilder.fromPCtoCells(mapbuilder.global_map);
	mapbuilder.viewer.showCloud(mapbuilder.global_cell_map);

	while(!mapbuilder.viewer.wasStopped())
	{
		boost::this_thread::yield();
	}

	delete src_ptr;
	delete dst_ptr;

}



void main()
{
	
	//testPlanes("D:\\Work\\Session1");
	//testRootNode("D:\\Work\\Session5");
	//testSessionGraph("D:\\Work\\Session11");
	//testDel();
	//testicp1();
	//testDisplayFPs("D:\\Work\\Session11");
	//testDisplayPCs("D:\\Work\\Session11");
	//testCicleNodes("D:\\Work\\Session11");
	//testFeatureMatch("D:\\Work\\Session11");
	//testTriangleMesh();
	//testMy3DSWriter(string("D:\\PCL_install_on_VS2008\\3DS\\man.3ds"));
	//testLoopMatch("D:\\Work\\Session11");
	//testMy3DSReader();
	CVisualSlam slam("SURF", "SURF", "BruteForce");
	//CVisualSlam slam("FAST", "FAST", "BruteForce");
	//slam.runFeature();
	slam.runHogman();

	//testCoordinateTranslate();

	//testFPFH();
	//testNarf();
	//CaptureFrames();
	getchar();
	system("pause");
}
void testFeatureMatch(string dir)
{
	// 1, obtain sessions
	vector<boost::shared_ptr<CSession> > sessions;
	getSessions(dir,sessions,11);
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	CMyGraphManager myGraph;

	for(size_t i=4;i<9/*sessions.size()*/;i++){
		if(i==2) continue;
		CMyNode* pNode = new CMyNode(sessions[i]);
		myGraph.addNode(pNode);
	}
	CMyNode* pNode = new CMyNode(sessions[2]);
	myGraph.addNode(pNode);

	/*CMyNode* psecNode2 = new CMyNode(sessions[3]);
	CMyNode* pfstNode3 = new CMyNode(sessions[4]);
	CMyNode* psecNode4 = new CMyNode(sessions[5]);
	CMyNode* psecNode5 = new CMyNode(sessions[6]);
	
	psecNode2->buildFlannIndex();
	pfstNode3->buildFlannIndex();
	psecNode4->buildFlannIndex();
	psecNode2->m_id=3;
	pfstNode3->m_id=4;
	psecNode4->m_id=5;
	psecNode5->m_id=6;*/

	
	//myGraph.addNode(psecNode2);
	//myGraph.addNode(pfstNode3);
	//myGraph.addNode(psecNode4);
	//myGraph.addNode(psecNode5);

	//MatchingResult mr;
	//mr= psecNode4->matchNodePair(pfstNode3,false);
	//Eigen::Matrix4f	HM1;
	//Eigen::Matrix4f	HM2;
	//Eigen::Matrix4f	HM3;

	//if(mr.edge.id1>=0)
	//{
	//	cout<<"(3,4) still work!"<<endl;
	//	CPose3D trans(mr.final_trafo);
	//	trans.getHomogeneousMatrix(HM1);
	////	pfstNode3->m_rootPose=CPose3D();
	//	pfstNode3->m_rootPose.getHomogeneousMatrix(HM2);
	//	HM3=HM2*HM1;
	//	cout<<"before: "<<endl;
	//	psecNode4->m_rootPose.output(std::cout);
	//	psecNode4->m_rootPose=CPose3D(HM3);
	//	cout<<"after: "<<endl;
	//	psecNode4->m_rootPose.output(std::cout);
	//}
	//else
	//{
	//	cout<<"(3,4) die!!"<<endl;
	//}
	//pfstNode3->translatePCs(pfstNode3->m_rootPose,pfstNode3->m_pPC);
	//m_disPC->points.insert(m_disPC->points.end(),pfstNode3->m_pPC->points.begin(),pfstNode3->m_pPC->points.end());
	//psecNode4->translatePCs(psecNode4->m_rootPose,psecNode4->m_pPC);
	//m_disPC->points.insert(m_disPC->points.end(),psecNode4->m_pPC->points.begin(),psecNode4->m_pPC->points.end());

	//Eigen::Matrix4f	HM;
	//Eigen::Matrix4f	HM1;
	//Eigen::Matrix4f	HM2;
	//Eigen::Matrix4f HM3;

	std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = myGraph.optimizer_->vertices().begin();
	map<int,CMyNode*>::iterator it_graph;
	int color_index=0;
	
	AdjustPoseInGraph(&myGraph);
	
	//for( ;vertex_iter!=myGraph.optimizer_->vertices().end();vertex_iter++)
	//{
	//	AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
	//	it_graph=myGraph.graph_.find(vertex_iter->second->id());
	//
	//	CPose3D node_pose;
	//	double x1=v_to_del->transformation.translation().x();
	//	double y1=v_to_del->transformation.translation().y();
	//	double z1=v_to_del->transformation.translation().z();
	//	_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
	//	double roll = rpy.roll();
	//	double pitch = rpy.pitch();
	//	double yaw = rpy.yaw();
	//	node_pose.setrpy(roll,pitch,yaw);
	//	node_pose.setxyz(x1,y1,z1);
	///*	if(vertex_iter->second->id()==0)
	//	{
	//		node_pose.getHomogeneousMatrix(HM1);
	//		it_graph->second->m_rootPose.getHomogeneousMatrix(HM);
	//	}*/
	//	/*node_pose.getHomogeneousMatrix(HM2);
	//	HM3=HM1.inverse()*HM2;
	//	CPose3D tmpFpose(HM3);
	//	HM2=HM*HM3;*/

	//	CMyNode* pNode=it_graph->second;
	//	pNode->m_rootPose=node_pose;
	//	pNode->translatePCs(pNode->m_rootPose,pNode->m_tmpFP);
	//	addcoloroffeatures(pNode,color_index++);
	//	m_disPC->points.insert(m_disPC->points.end(),pNode->m_tmpFP->points.begin(),pNode->m_tmpFP->points.end());
	//	//pNode->translatePCs(node_pose,pNode->m_tmpFP_pre);
	//	//pNode->translatePCs(node_pose,pNode->m_tmpFP_pos);
	//	//m_disPC->points.insert(m_disPC->points.end(),pNode->m_tmpFP_pre->points.begin(),pNode->m_tmpFP_pre->points.end());
	//	//m_disPC->points.insert(m_disPC->points.end(),pNode->m_tmpFP_pos->points.begin(),pNode->m_tmpFP_pos->points.end());

	//	pNode->translatePCs(pNode->m_rootPose,pNode->m_pPC);
	//	m_disPC->points.insert(m_disPC->points.end(),pNode->m_pPC->points.begin(),pNode->m_pPC->points.end());
	//}

	for(map<int,CMyNode*>::iterator it=myGraph.graph_.begin();it!=myGraph.graph_.end();it++)
	{
		CMyNode* pthisnode=it->second;
		/*pthisnode->translateFeatures(pthisnode->m_rootPose,pthisnode->m_tmpFP);
		m_disPC->points.insert(m_disPC->points.end(),it->second->m_tmpFP->points.begin(),it->second->m_tmpFP->points.end());*/
		pthisnode->translatePCs(pthisnode->m_rootPose,pthisnode->m_pPC);
		m_disPC->points.insert(m_disPC->points.end(),pthisnode->m_pPC->points.begin(),pthisnode->m_pPC->points.end());
	}

	//pcl::PCDWriter pcdWrite;
	//pcdWrite.write("D:\\PCL_install_on_VS2008\\pcds\\workplace.pcd",*m_disPC);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	DisplayPointXYZRGB(viewer,m_disPC);
	return ;

}
void testLoopMatch(string dir)
{
	// 1, obtain sessions
	vector<boost::shared_ptr<CSession> > sessions;
	getSessions(dir,sessions,11);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	CMyNode* pfstNode0 = new CMyNode(sessions[0]);
	CMyNode* psecNode2 = new CMyNode(sessions[2]);
	CMyNode* pfstNode6 = new CMyNode(sessions[6]);
	CMyNode* psecNode9 = new CMyNode(sessions[9]);

	pfstNode0->buildFlannIndex();
	pfstNode6->buildFlannIndex();
	psecNode2->buildFlannIndex();
	psecNode9->buildFlannIndex();
	
	pfstNode0->m_id=0;
	pfstNode6->m_id=6;
	psecNode2->m_id=2;
	psecNode9->m_id=9;

	MatchingResult mr;
	mr= psecNode2->matchNodePair(pfstNode0,true);
	if(mr.edge.id1>=0)
	{
		cout<<"(0,2) still work!"<<endl;
	}
	else
	{
		cout<<"(0,2) die!!"<<endl;
	}


	mr= psecNode9->matchNodePair(pfstNode6,true);
	if(mr.edge.id1>=0)
	{
		cout<<"(6,9) still work!"<<endl;
	}
	else
	{
		cout<<"(6,9) die!!"<<endl;
	}

	
}

void addcoloroffeatures(CMyNode* pNode, int color_index)
{
	color_index%=3;
	for(size_t i=0;i<pNode->m_tmpFP->points.size();i++)
	{
		pcl::PointXYZRGB& sp=pNode->m_tmpFP->points[i];
		addcolor(sp,color_index);
	}
	for(size_t i=0;i<pNode->m_tmpFP_pre->points.size();i++)
	{
		pcl::PointXYZRGB& sp=pNode->m_tmpFP_pre->points[i];
		addcolor(sp,color_index);
	}
	for(size_t i=0;i<pNode->m_tmpFP_pos->points.size();i++)
	{
		pcl::PointXYZRGB& sp=pNode->m_tmpFP_pos->points[i];
		addcolor(sp,color_index);
	}
	return ;
}
void addcolor(pcl::PointXYZRGB& sp,int index)
{
	// 0 red,1 green,2 blue, 
	switch (index)
	{
	case 0:
		sp.r=255; sp.g=0; sp.b=0;
		break;
	case 1:
		sp.r=0; sp.g=255; sp.b=0;
		break;
	case 2:
		sp.r=0; sp.g=0; sp.b=255;
		break;
	default:
		sp.r=255;sp.g=255;sp.b=255;
		break;
	}
}
void testCicleNodes(string dir)
{
	// 1, obtain sessions
	vector< vector<boost::shared_ptr<CSession> >  > circle_sessions;
	vector<boost::shared_ptr<CSession> > sessions;
	int circles=3;
	while(circles-->0){
		getSessions(dir,sessions,11);
		circle_sessions.push_back(sessions);
		sessions.swap( vector<boost::shared_ptr<CSession> >());
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);

	CPose3D lastPose;
	CPose3D firstPose;
	vector<CPose3D> originalPose;
	vector<CPose3D> translatedPose;
	Eigen::Matrix4f	HM1;
	Eigen::Matrix4f	HM2;
	Eigen::Matrix4f HM;
	Eigen::Matrix4f tmpHM;
	for(size_t i=0;i<circle_sessions.size();i++){
		lastPose.getHomogeneousMatrix(HM2);
		for(size_t sid=0;sid<circle_sessions[i].size();sid++)
		{
			CMyNode* rootNode=new CMyNode(circle_sessions[i][sid]);			
			rootNode->m_rootPose.getHomogeneousMatrix(HM1);
			HM=HM2*HM1;
			CPose3D tmpPose(HM);
			if(sid==2)
			{
				lastPose=tmpPose;//rootNode->m_lastPose;
			}
		
			rootNode->translatePCs(tmpPose,rootNode->m_pPC);
			m_disPC->points.insert(m_disPC->points.end(),rootNode->m_pPC->points.begin(),rootNode->m_pPC->points.end());
			delete rootNode;
		}
	}
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	DisplayPointXYZRGB(viewer,m_disPC);
}

void testDisplayPCs(string dir)
{
	// 1, obtain sessions
	vector< vector<boost::shared_ptr<CSession> >  > circle_sessions;
	vector<boost::shared_ptr<CSession> > sessions;
	int circles=2;
	int tmp=circles;
	while(tmp-->0){
		getSessions(dir,sessions,11);
		circle_sessions.push_back(sessions);
		sessions.swap( vector<boost::shared_ptr<CSession> >());
	}
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	CPose3D lastPose;
	CPose3D tmpPose;
	for(size_t i=0;i<circles;i++){
		for(size_t sid=0;sid<circle_sessions[i].size();sid++)
		{
			if(sid==2) continue;
			CMyNode* rootNode=new CMyNode(circle_sessions[i][sid]);
			tmpPose.composeFrom(rootNode->m_rootPose,lastPose);
			rootNode->translatePCs(tmpPose,rootNode->m_pPC);
			m_disPC->points.insert(m_disPC->points.end(),rootNode->m_pPC->points.begin(),rootNode->m_pPC->points.end());
			delete rootNode;
		}
		CMyNode* rootNode=new CMyNode(circle_sessions[i][2]);
		tmpPose.composeFrom(rootNode->m_rootPose,lastPose);
		rootNode->translatePCs(tmpPose,rootNode->m_pPC);
		m_disPC->points.insert(m_disPC->points.end(),rootNode->m_pPC->points.begin(),rootNode->m_pPC->points.end());
		
		lastPose=rootNode->m_lastPose; 
		delete rootNode;
	}
	pcl::visualization::CloudViewer viewer("testSession");
	viewer.showCloud(m_disPC);
	while (!viewer.wasStopped ())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}
void testDisplayFPs(string dir)
{
	// 1, obtain sessions
	vector<boost::shared_ptr<CSession> > sessions;
	getSessions(dir,sessions,11);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	int nodeid=0;
	int nodeend=nodeid+4;
	for(size_t sid=nodeid;sid<sessions.size();sid++)
	{
		CMyNode* rootNode=new CMyNode(sessions[sid]);
		//rootNode->translateFeatures(rootNode->m_rootPose,rootNode->m_feature_locations_3d);
		rootNode->translateFeatures(rootNode->m_rootPose,rootNode->m_tmpFP);
		m_disPC->points.insert(m_disPC->points.end(),rootNode->m_tmpFP->points.begin(),rootNode->m_tmpFP->points.end());
	/*	rootNode->translateFeatures(rootNode->m_rootPose,rootNode->m_tmpFP_pre);
		m_disPC->points.insert(m_disPC->points.end(),rootNode->m_tmpFP_pre->points.begin(),rootNode->m_tmpFP_pre->points.end());
		rootNode->translateFeatures(rootNode->m_rootPose,rootNode->m_tmpFP_pos);
		m_disPC->points.insert(m_disPC->points.end(),rootNode->m_tmpFP_pos->points.begin(),rootNode->m_tmpFP_pos->points.end());*/
		delete rootNode;
	}
	pcl::visualization::CloudViewer viewer("testSession");
	viewer.showCloud(m_disPC);
	while (!viewer.wasStopped ())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}
void tesetq()
{
	CPose3D pose(3.01,1.2,-0.6,0,0,0);
	double q[4];
	double roll,pitch,yaw;
	for(int i=0;i<4;i++)
		q[i]=pose.q[i];
	Eigen::Matrix4f eigen_mat;
	pose.getHomogeneousMatrix(eigen_mat);
	Eigen::Affine3f eigen_transform(eigen_mat);
	Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
	Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
	Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(), eigen_quat.w());
	Transformation3 result(translation, rotation);
	_Vector<3, double> rpy = result.rotation().rotationMatrix().angles();
	roll = rpy.roll();
	pitch = rpy.pitch();
	yaw = rpy.yaw();

}

void testRootNode(string dir)
{
	// get only one session, and create its root node
	vector<boost::shared_ptr<CSession> > sessions;
	getSessions(dir,sessions,24);

	// for test transformPoint
	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_obtain=sessions[0]->m_pc;
	CPose3D dummyPose(-0.1,0.3,2,1,-0.4,2.1);
	Eigen::Matrix4f	HM;
	dummyPose.getHomogeneousMatrix(HM);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_comp1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::transformPointCloud(*sessions[0]->m_pc,*m_comp1,HM);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_comp2(new pcl::PointCloud<pcl::PointXYZRGB>);
	size_t NP = m_comp1->points.size();
	m_comp2->points.resize(NP);
	double gx,gy,gz;
	for(size_t k=0;k<NP;k++)
	{
		pcl::PointXYZRGB& sp=m_obtain->points[k];
		pcl::PointXYZRGB tmpsp;
		dummyPose.transformPoint(sp.x,sp.y,sp.z,gx,gy,gz);
		tmpsp.x=gx; tmpsp.y=gy; tmpsp.z=gz;
		tmpsp.r=sp.r; tmpsp.g=sp.g; tmpsp.b=sp.b;
		m_comp2->points.push_back(tmpsp);
	}*/
	
	//m_comp1->points.insert(m_comp1->points.end(),m_comp2->points.begin(),m_comp2->points.end());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(size_t i=18;i<sessions.size();i++){
		CMyNode* rootNode=new CMyNode(sessions[i]);
		cout<<"sessions "<<i+1<<" PC size: "<<sessions[i]->m_pc->points.size()<<endl;
		m_disPC->points.insert(m_disPC->points.end(),rootNode->m_pPC->points.begin(),rootNode->m_pPC->points.end());
		delete rootNode;
	}

	pcl::visualization::CloudViewer viewer("testSession");
	//viewer.showCloud(m_comp1);
	//viewer.showCloud(rootNode->m_pPC);
	//viewer.showCloud(rootNode->m_tmpFP);
	viewer.showCloud(m_disPC);
	/*--------------------
	-----Main loop-----
	--------------------*/
	while (!viewer.wasStopped ())
	{
		//viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}

}

void testSessionGraph(string dir)
{
	// 1, obtain sessions
	vector< vector<boost::shared_ptr<CSession> >  > circle_sessions;
	vector<boost::shared_ptr<CSession> > sessions;
	int circles=2;
	while(circles-->0){
		getSessions(dir,sessions,11);
		circle_sessions.push_back(sessions);
		sessions.swap( vector<boost::shared_ptr<CSession> >());
	}
	
	// 2, reduce session into node, and add it into graph
	CMyGraphManager myGraph;
	bool bfirst=true;
	int nodeid=0;
	for(size_t i=0;i<circle_sessions.size();i++){
		for(size_t sid=nodeid;sid<circle_sessions[i].size();sid++)
		{
			if(sid==2) continue;
			CMyNode* rootNode=new CMyNode(circle_sessions[i][sid]);
			if(bfirst){
				myGraph.initFirstNode(rootNode->m_rootPose);
				bfirst=false;
			}
			myGraph.addNode(rootNode);
			//delete rootNode;
		}
		CMyNode* rootNode=new CMyNode(circle_sessions[i][2]);
		myGraph.addNode(rootNode);
	}
	// Adjust root pose of each session
	AdjustPoseInGraph(&myGraph);

	// Translate local points of each session into global frame
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(map<int,CMyNode*>::iterator it=myGraph.graph_.begin();it!=myGraph.graph_.end();it++)
	{
		CMyNode* pthisnode=it->second;
		/*pthisnode->translateFeatures(pthisnode->m_rootPose,pthisnode->m_tmpFP);
		m_disPC->points.insert(m_disPC->points.end(),it->second->m_tmpFP->points.begin(),it->second->m_tmpFP->points.end());*/
		pthisnode->translatePCs(pthisnode->m_rootPose,pthisnode->m_pPC);
		m_disPC->points.insert(m_disPC->points.end(),pthisnode->m_pPC->points.begin(),pthisnode->m_pPC->points.end());
	}

	// 5, display all sessions
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	DisplayPointXYZRGB(viewer,m_disPC);

}
void AdjustPoseInGraph(CMyGraphManager* pMyGraph)
{
	// update all path node's position
	// transformation all point cloud from Nodes
	// Merge all pc into a global_pc
	std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = pMyGraph->optimizer_->vertices().begin();
	std::map<int, CMyNode* >::iterator vertex_iter_graph = pMyGraph->graph_.begin();

	for( ;vertex_iter!=pMyGraph->optimizer_->vertices().end();vertex_iter++)
	{
		AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
		vertex_iter_graph=pMyGraph->graph_.find(vertex_iter->second->id());
		CPose3D node_pose;
		double x1=v_to_del->transformation.translation().x();
		double y1=v_to_del->transformation.translation().y();
		double z1=v_to_del->transformation.translation().z();
		_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
		double roll = rpy.roll();
		double pitch = rpy.pitch();
		double yaw = rpy.yaw();
		node_pose.setrpy(roll,pitch,yaw);
		node_pose.setxyz(x1,y1,z1);
		vertex_iter_graph->second->m_rootPose=node_pose;
	}

}
void testicp1()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc2(new pcl::PointCloud<pcl::PointXYZRGB>);
	srand((unsigned int)time(NULL));
	pcl::PointXYZRGB sp1;
	pcl::PointXYZRGB sp2;
	CPose3D TransPose(-0.2,-3.1,2.3,1,-2,3);
	Eigen::Matrix4f	HM1;
	TransPose.getHomogeneousMatrix(HM1);

	vector<pcl::TMatchingPair> cors;
	Eigen::Vector4f src;
	Eigen::Vector4f dst;

	float pt[4];
	float tt[4];
	for(int i=0;i<200;i++)
	{
		// calculate Local coordinates
		pt[0]=(rand()%5)*0.01;
		pt[1]=(rand()%3)*0.01; 
		pt[2]=(rand()%5)*0.01;
		pt[3]=1.0;
		Eigen::Map<Eigen::Vector3f> thispt=Eigen::Vector3f::Map(pt);
		Eigen::Map<Eigen::Vector3f> thatpt=Eigen::Vector3f::Map(tt);
		thatpt=(Eigen::Affine3f)(HM1)*thispt;

		src=Eigen::Vector4f::Map(pt);
		dst=Eigen::Vector4f::Map(tt);
		cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));
	}

	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;

	//double me_start_t = ::GetTickCount();
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	cout<<"trans pose: "<<endl;
	TransPose.output(std::cout);
	TransPose.displayROT();
	cout<<"icp pose: "<<endl;
	finalPose.output(std::cout);
	finalPose.displayROT();

	MatchingResult mr;
	//mr.final_trafo = mr.ransac_trafo;
	finalPose.getHomogeneousMatrix(mr.final_trafo);
	mr.edge.id1 = 0;//and we have a valid transformation
	mr.edge.id2 = 1; //since there are enough matching features,
	mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
	mr.edge.informationMatrix =   Matrix6::eye(5*5); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

	CMyGraphManager mygraph;
	mygraph.optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0)); //fix at origin
	mygraph.addEdgeToHogman(mr.edge,true);

	std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = mygraph.optimizer_->vertices().begin();
	for( ;vertex_iter!=mygraph.optimizer_->vertices().end();vertex_iter++)
	{
		AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
		CPose3D node_pose;
		double x1=v_to_del->transformation.translation().x();
		double y1=v_to_del->transformation.translation().y();
		double z1=v_to_del->transformation.translation().z();
		_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
		double roll = rpy.roll();
		double pitch = rpy.pitch();
		double yaw = rpy.yaw();
		node_pose.setrpy(roll,pitch,yaw);
		node_pose.setxyz(x1,y1,z1);

		/*	cout<<"In optimize id: "<<v_to_del->id()<<endl;
		cout<<"(x,y,z)" <<"("<<x1<<","<<y1<<","<<z1<<")" <<endl;
		cout<<"(r,p,y)" <<"("<<R2D(roll)<<","<<R2D(pitch)<<","<<R2D(yaw)<<")"<<endl;*/
		cout<<"node pose:"<<endl;
		//node_pose.output(std::cout);
		node_pose.displayROT();
	}
}

void testDel()
{
	CPose3D RootPose;
	CPose3D TransPose(-0.2,-3.1,2.3,1,2,3);
	Eigen::Matrix4f	HM1;
	Eigen::Matrix4f HM2;
	Eigen::Matrix4f	HM3;
	RootPose.getHomogeneousMatrix(HM1);
	TransPose.getHomogeneousMatrix(HM2);
	HM3=HM1*HM2;
	
	CPose3D RobotPose(HM3);
	cout<<"TransPose: "<<endl;
	//TransPose.output(std::cout);
	TransPose.displayROT();
	cout<<"RobotPose: "<<endl;
	//RobotPose.output(std::cout);
	RobotPose.displayROT();

	MatchingResult mr;
	//mr.final_trafo = mr.ransac_trafo;
	TransPose.getHomogeneousMatrix(mr.final_trafo);
	mr.edge.id1 = 0;//and we have a valid transformation
	mr.edge.id2 = 1; //since there are enough matching features,
	mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
	mr.edge.informationMatrix =   Matrix6::eye(5*5); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

	CMyGraphManager mygraph;
	mygraph.optimizer_->addVertex(0, Transformation3(), 1e9*Matrix6::eye(1.0)); //fix at origin
	mygraph.addEdgeToHogman(mr.edge,true);
	
	std::map<int, AISNavigation::Graph::Vertex*>::iterator vertex_iter = mygraph.optimizer_->vertices().begin();
	for( ;vertex_iter!=mygraph.optimizer_->vertices().end();vertex_iter++)
	{
		AISNavigation::PoseGraph3D::Vertex* v_to_del =  reinterpret_cast<AISNavigation::PoseGraph3D::Vertex*>(vertex_iter->second);
		CPose3D node_pose;
		double x1=v_to_del->transformation.translation().x();
		double y1=v_to_del->transformation.translation().y();
		double z1=v_to_del->transformation.translation().z();
		_Vector<3, double> rpy = v_to_del->transformation.rotation().rotationMatrix().angles();
		double roll = rpy.roll();
		double pitch = rpy.pitch();
		double yaw = rpy.yaw();
		node_pose.setrpy(roll,pitch,yaw);
		node_pose.setxyz(x1,y1,z1);

	/*	cout<<"In optimize id: "<<v_to_del->id()<<endl;
		cout<<"(x,y,z)" <<"("<<x1<<","<<y1<<","<<z1<<")" <<endl;
		cout<<"(r,p,y)" <<"("<<R2D(roll)<<","<<R2D(pitch)<<","<<R2D(yaw)<<")"<<endl;*/
		cout<<"node pose:"<<endl;
		//node_pose.output(std::cout);
		node_pose.displayROT();
	}
}

void testFPFH(){
	//((CFPFHNode*)(0))->correspondences_demo("D:\\PCL_install_on_VS2008\\pcds\\robot\\robot");
	((CFPFHNode*)(0))->fpfh_demo("D:\\PCL_install_on_VS2008\\pcds\\robot\\robot");
	//CFPFHNode node1("D:\\PCL_install_on_VS2008\\pcds\\robot\\robot1.pcd");
	//CFPFHNode node2("D:\\PCL_install_on_VS2008\\pcds\\robot\\robot2.pcd");
	//
	//MatchingResult mr=node2.matchNodePair(&node1);
	//CPose3D pose(mr.final_trafo);
	//pose.output(std::cout);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//m_disPC->insert(m_disPC->points.end(),node1.m_pc->points.begin(),node1.m_pc->points.end());
	//((CMyNode*)(0))->translatePCs(pose,node2.m_pc);
	//m_disPC->insert(m_disPC->points.end(),node2.m_pc->points.begin(),node2.m_pc->points.end());

	//// 5, display all sessions
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//DisplayPointXYZRGB(viewer,m_disPC);

	return ;
}

