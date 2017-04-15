#include "Iot_Robot_rgbdslam_Interface.h"


#include "Openni.h"
#include <boost/array.hpp>
#include <boost/shared_array.hpp>


Node* Iot_Robot_rgdbslam_Interface::createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pointcloud_type>& point_cloud, 
									 const cv::Mat& depth){
										 Node* node_ptr = new Node(visual, m_detector, m_extractor, m_matcher,
											 point_cloud, depth);
										 return node_ptr;
}

bool Iot_Robot_rgdbslam_Interface::computeNodePose(Node* node_ptr, double* xyz, double* rpy){

	if(m_graph_mgr.addNode(node_ptr))
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

void Iot_Robot_rgdbslam_Interface::getImagesandDepthMetaData(boost::shared_ptr<pointcloud_type> point_cloud,
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

// whether pose is good
bool Iot_Robot_rgdbslam_Interface::IsBigTra(CPose3D & pose){
	if(fabs(pose.m_coords[0]) > 2 || \
		fabs(pose.m_coords[1]) > 2 || \
		fabs(pose.m_coords[2]) > 2 || \
		fabs(R2D(pose.yaw)) > 50 || \
		fabs(R2D(pose.pitch)) > 50 || \
		fabs(R2D(pose.roll)) > 50 )
		return false;
	if(fabs(pose.m_coords[0]) > 0.1 || \
		fabs(pose.m_coords[1]) > 0.1 || \
		fabs(pose.m_coords[2]) > 0.1 || \
		fabs(R2D(pose.yaw)) > 5 || \
		fabs(R2D(pose.pitch)) > 5 || \
		fabs(R2D(pose.roll)) > 5 )
		return true;
	return false;
}


void Iot_Robot_rgdbslam_Interface::runoneframe(pcl::PointCloud<pcl::PointXYZRGB>::Ptr frame){
	
	boost::shared_ptr<openni_wrapper::Image> openni_image;
	boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
	boost::shared_ptr<pointcloud_type> point_cloud;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Initialize OpenNI Graber and Mapbuilder
	SimpleOpenNIViewer v;
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

	while(1){
		while(v.getCloudPoint( point_cloud))
		{
			// 1. get depth_image and image_matadata
			getImagesandDepthMetaData(point_cloud,rgb_buffer, depth_buffer);

			// Copy RGB data from OpenNI image to cv mat image
			memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
			memcpy(cvDepthImage.data, depth_buffer, 640*480*2);

			cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

			// 2. Create node to extract feature and wrap the image and feature data
			Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);
			node_ptr->id_ = m_graph_mgr.graph_.size();

			// indicate whether this frame will be added
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
				// 3. Match with last pose to compute relative pose
				Node* pre_node = m_graph_mgr.graph_[m_graph_mgr.graph_.size()-1];
				MatchingResult mr = node_ptr->matchNodePair(pre_node);
				// relative Pose to previous frame
				boost::shared_ptr<CPose3D> pose(new CPose3D(mr.final_trafo)); 

				// If this new pose is good: not noise nor too small
				if(IsBigTra(*pose)){
					isGoodTra = true; // This is good transformation, 

					// last pose
					CPose3D prepose = *robotpath.rbegin();
					// compute pose relative to global coordinates
					*pose += prepose ; 
					// for debug
					//pose.get()->output(std::cout);
					final_pose = pose;

					// 4. Insert node into node-graph and Insert pose into robot-path
					node_ptr->buildFlannIndex();
					m_graph_mgr.graph_[node_ptr->id_] = node_ptr;
					robotpath.push_back(*pose);				
				}
				else
					delete node_ptr;
			}
			if(isGoodTra) // will display this frame
			{
				while(!pMapbuilder->setrawmapwithtransform(point_cloud,final_pose)){
					boost::this_thread::yield();
				}
				isGoodTra = false;
			}

		}
	}
	
}

//boost::shared_ptr<openni_wrapper::Image> openni_image;
//boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;
//boost::shared_ptr<pointcloud_type> point_cloud;
//
//// Create cv mat image with the right size
//cv::Mat cvRGBImage(480, 640, CV_8UC3);
//cv::Mat cvGrayImage(480, 640,CV_8UC1);
//cv::Mat cvDepthImage(480, 640, CV_16UC1);
//cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);
//
//// 1. Initialize OpenNI Graber and Mapbuilder
//SimpleOpenNIViewer v;
//v.startframe();
//
//boost::shared_ptr<MapBuilder> pMapbuilder(new MapBuilder);
//
//// Get OpenNI visual image
//// Get OpenNI depth image
//
//static unsigned rgb_array_size = 0;
//static boost::shared_array<unsigned char> rgb_array(0);
//static unsigned char* rgb_buffer = 0;
//
//rgb_array_size = 480*640*3; // size of each frame
//rgb_array.reset(new unsigned char [rgb_array_size]);
//rgb_buffer = rgb_array.get();
//
//while(1){
//	while(v.getframe(openni_image, openni_depth_image, point_cloud))
//	{
//		/*xn::ImageMetaData xnImd;
//		xnImd.CopyFrom(openni_image->getMetaData());*/
//
//		// obtain depth metadata
//		xn::DepthMetaData xnDmd;
//		xnDmd.CopyFrom(openni_depth_image->getDepthMetaData());
//
//		// obtain image metadata
//		openni_image->fillRGB(640, 480, rgb_buffer, 640*3);
//
//		// Copy RGB data from OpenNI image to cv mat image
//		memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
//		//memcpy(cvGrayImage.data, xnImd.Grayscale8Data(), 640*480*1);
//		memcpy(cvDepthImage.data, xnDmd.Data(), 640*480*2);
//		// Convert RGB image to grayscale image
//		//cvRGBImage.convertTo(cvGrayImage, CV_8UC1);
//		cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);
//
//		// 3. Create node to extract feature and wrap the image and feature data
//		Node* node_ptr = createFeatureNode(cvRGBImage, point_cloud, cvDepth8UC1Image);
//
//		// 4. Insert node into node-graph to compute the pose of the node
//		double xyz[3]={0,0,0};
//		double rpy[3]={0,0,0};
//		if(computeNodePose(node_ptr, xyz, rpy)){
//
//			if(robotpath.size() == 0){
//				robotpath.push_back( CPose3D());
//			}
//
//
//
//			// 5. compute current pose relative to global coordinates
//			boost::shared_ptr<CPose3D> current(new CPose3D(rpy[2],rpy[1],rpy[0],xyz[0],xyz[1],xyz[2]));
//			//	*current += *robotpath.rbegin(); // this is already global pose info, so, 
//			robotpath.push_back(*current);
//
//			// see what changed from CPose3D
//			current->getYawPitchRoll(rpy[2],rpy[1],rpy[0]);
//			current->getXYZ(xyz);
//
//
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
//
//}
