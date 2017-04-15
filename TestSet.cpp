#include "TestSet.h"
#include <opencv2/highgui/highgui_c.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/range_image/range_image.h>
//#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>

#include "Openni.h"
#include "CPose3D.h"
#include "AreaStore.h"
#include "CMy3DSWriter.h"
#include "CMy3DSReader.h"
#include "TriangleMesh.h"



//#include "CMyNode.h"
//#include "graph_manager.h"

void testCoordinateTranslate()
{
	CPose3D local(1,2,3,-0.5,1.2,3.6);
	pcl::PointXYZ lpt(1,1,1);
	pcl::PointXYZ gpt;
	Eigen::Matrix4f	HM;
	local.getHomogeneousMatrix(HM);
	float pt[4];
	float tt[4];
	pt[0]=lpt.x; pt[1]=lpt.y; pt[2]=lpt.z; pt[3]=1.0;
	Eigen::Map<Eigen::Vector3f> thispt=Eigen::Vector3f::Map(pt);
	Eigen::Map<Eigen::Vector3f> thatpt=Eigen::Vector3f::Map(tt);
	thatpt=(Eigen::Affine3f)(HM)*thispt;
	gpt.x=thatpt(0);
	gpt.y=thatpt(1);
	gpt.z=thatpt(2);
	

	cout<<"Local Point:("<<lpt.x<<","<<lpt.y<<","<<lpt.z<<")"<<endl;
	cout<<"Global Poitn: ("<<gpt.x<<","<<gpt.y<<","<<gpt.z<<")"<<endl;
	cout<<"After Inverse"<<endl;

	//local.output(std::cout);
	Eigen::Matrix4f tmpHM=HM.inverse();
	//HM = tmpHM.inverse();
	//CPose3D tmpPose(HM);

	thispt=(Eigen::Affine3f)(tmpHM)*thatpt;
	lpt.x=thispt(0); lpt.y=thispt(1); lpt.z=thispt(2);
	cout<<"Local Point:("<<lpt.x<<","<<lpt.y<<","<<lpt.z<<")"<<endl;
	cout<<"Global Poitn: ("<<gpt.x<<","<<gpt.y<<","<<gpt.z<<")"<<endl;
}

void testPose(){

	CPose3D p1;
	CPose3D p2(1,0.3,0.2,0.4,0.2,0.1)/*(1,2,0.4,0.4,0.2,0.1)*/;
	CPose3D p3(-0.2,-0.1,0.3,-0.4,-0.2,-0.2);
	CPose3D p12(p2);

	cout<<"p2"<<endl;
	p2.output(std::cout);
	cout<<"p3"<<endl;
	p3.output(std::cout);

	CPose3D p13 = p2+p3;
	cout<<"p13"<<endl;
	p13.output(std::cout);
	CPose3D p23 = p13 - p12;
	cout<<"p23"<<endl;
	p23.output(std::cout);
}

void testSession(string dir)
{
	CAreaStore m_LoadSession;
	m_LoadSession.LoadConfig(dir);

	// get store header list
	AreaStoreHdrList hdrList;
	m_LoadSession.GetStoreHdrList(hdrList);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl :: PointXYZRGB>());
	// file2area
	for(int i=0; i<2/*hdrList.size()*/; i++)
	{
		m_LoadSession.File2AreaMap(m_pc, hdrList[i].id);
	}

	// Create a shared plane model pointer directly
	pcl::SampleConsensusModelPlane<pcl::PointXYZRGB>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> (m_pc));
	// Create the RANSAC object
	pcl::RandomSampleConsensus<pcl::PointXYZRGB> sac (model, 0.03);
	// perform the segmenation step
	bool result = sac.computeModel ();

	// get inlier indices
	boost::shared_ptr<vector<int> > inliers (new vector<int>);
	sac.getInliers (*inliers);
	cout << "Found model with " << inliers->size () << " inliers"<<endl;

	// Create a visualizer
	pcl::visualization::PCLVisualizer vis ("RSS 2011 PCL Tutorial - 04 Segmentation");
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
	// Extract the inliers
	extract.setInputCloud (m_pc);
	extract.setIndices (inliers);
	extract.setNegative (false);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr subcloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	extract.filter (*subcloud);
	// finally, add both clouds to screen

	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> handler (m_pc);
	vis.addPointCloud<pcl::PointXYZRGB>(m_pc, handler, "cloud");
	vis.addPointCloud<pcl::PointXYZRGB>(subcloud, handler, "inliers");

	while (!vis.wasStopped ())
	{
		vis.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

	//pcl::visualization::CloudViewer viewer("testSession");
	//viewer.showCloud(m_pc);
	//--------------------
	// -----Main loop-----
	//--------------------
	//while (!viewer..wasStopped ())
	//{
	//	//viewer.spinOnce (100);
	//	boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	//}
}

void testresampling(string dir)
{
	CAreaStore m_LoadSession;
	m_LoadSession.LoadConfig(dir);

	// get store header list
	AreaStoreHdrList hdrList;
	m_LoadSession.GetStoreHdrList(hdrList);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl :: PointXYZRGB>());
	// file2area
	for(int i=0; i<1/*hdrList.size()*/; i++)
	{
		m_LoadSession.File2AreaMap(m_pc, hdrList[i].id);
	}
	//// Create a KD-Tree
	//pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);

	////pcl::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTree<pcl::PointXYZRGB>);

	//// Output has the same type as the input one, it will be only smoothed
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZRGB>);

	//// Init object (second point type is for the normals, even if unused)
	//pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal> mls;

	//// Optionally, a pointer to a cloud can be provided, to be set by MLS
	//pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
	//mls.setOutputNormals (mls_normals);

	//// Set parameters
	//mls.setInputCloud (m_pc);
	//mls.setPolynomialFit (true);
	//mls.setSearchMethod (tree);
	//mls.setSearchRadius (0.1);

	//// Reconstruct
	//mls.reconstruct (*mls_points);
	pcl::visualization::CloudViewer viewer("testSession");
	viewer.showCloud(m_pc);
	while (!viewer.wasStopped ())
	{
		//viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}

}

void testMap(int W, int H, vector<vector<unsigned char> >& m_Map)
{
	int total=W*H;
	int n_blocks=total>>2;
	int w=W;
	int h=H;

	int original_x=(W>>1)-1;
	int original_y=(H>>1)-1;
	//	vector<vector<unsigned char> > m_Map;
	m_Map.swap(vector<vector<unsigned char> >());
	m_Map.resize(H);
	for(int i=0;i<H;i++)
		m_Map[i].resize(W,0);

	set<int> used_location;
	used_location.insert(original_y*W+original_x);
	srand(unsigned long(time(NULL)));
	for(int k=0;k<n_blocks;k++)
	{
		int l_x=rand()%W;
		int l_y=rand()%H;
		int used_l=l_y*W+l_x;
		while(used_location.count(used_l)) {
			l_x=((l_x)+rand()%W)%W;
			l_y=((l_y)+rand()%H)%H;
			used_l=l_y*W+l_x;
		}
		used_location.insert(used_l);
		m_Map[l_y][l_x]=255;
	}
	for(int i=0;i<H;i++)
	{
		for(int j=0;j<W;j++)
		{
			if(i==original_y && j==original_x)
				cout<<"S";
			else if(m_Map[i][j]==255)
				cout<<"B";
			else cout<<"0";
		}
		cout<<endl;
	}
}
void testMem()
{
	boost::dynamic_bitset<> tmpbits;
	try{
		tmpbits.resize(1024*1024*1024*80,false);
		if(tmpbits.size()==0)
			cout<<"failed!"<<endl;
		else tmpbits[100]=1;
	}
	catch(...)
	{
		cout<<"cacaca!!"<<endl;
	}
	cout<<"successful!"<<endl;
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
	// --------------------------------------------------------
	// -----Open 3D viewer and add point cloud and normals-----
	// --------------------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();
	return (viewer);
}
void getSessions(string dir,vector<boost::shared_ptr<CSession> >& sessions,unsigned int n_S)
{
	CAreaStore m_LoadSession;
	m_LoadSession.LoadConfig(dir);

	// get store header list
	AreaStoreHdrList hdrList;
	m_LoadSession.GetStoreHdrList(hdrList);

	int U_limit=min(n_S,hdrList.size());
	//vector<boost::shared_ptr<CSession> > sessions;
	// file2area
	for(int i=0; i<U_limit/*hdrList.size()*/; i++)
	{
		boost::shared_ptr<CSession> tmp_session(new CSession);
		tmp_session->m_fhdr=hdrList[i];
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl :: PointXYZRGB>());
		m_LoadSession.File2AreaMap(tmp_session->m_pc, hdrList[i].id);
		//Area3DDescMap descMap;
		m_LoadSession.File2Area3DDesc(tmp_session->m_descMap, hdrList[i].id);
		//AreaPathList pathList;
		m_LoadSession.File2AreaPath(tmp_session->m_pathList, hdrList[i].id);
		sessions.push_back(tmp_session);
	}
}
void testPlanes(string dir)
{
	vector<boost::shared_ptr<CSession> > sessions;
	getSessions(dir,sessions,1);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB> );
	pcl::PointCloud<pcl::Normal>::Ptr m_normal(new pcl::PointCloud<pcl::Normal> );
	for(size_t i=0;i<sessions.size();i++)
	{
		sessions[i]->GeneratePlanes5();
		//sessions[i]->GeneratePlanes6();
	}
	//sessions[0]->GetNormalsAndPoints(m_pc,m_normal);
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	//viewer=normalsVis(m_pc,m_normal);
	////--------------------
	//// -----Main loop-----
	////--------------------
	//while (!viewer->wasStopped ())
	//{
	//	viewer->spinOnce (100);
	//	boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	//}

	sessions[0]->FetchPlanePoints(m_pc);
	for(size_t i=0;i<sessions[0]->m_Planes5.size();i++)
	{
		sessions[0]->m_Planes5[i]->CalculateVar();
	}
	pcl::visualization::CloudViewer viewer("testSession");
	viewer.showCloud(m_pc);
	/*--------------------
	-----Main loop-----
	--------------------*/
	while (!viewer.wasStopped ())
	{
		//viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (10000));
	}
}



void testMy3DSReader()
{
	//string file("C:\\Users\\Iot\\Desktop\\3ds\\man.3ds");
	string file("C:\\Users\\Iot\\Desktop\\3ds\\3ds\\teapot.3ds");
	CMy3DSReader myReader;
	if(!myReader.Load(file.c_str()))
	{
		cout<<"failed to parse file: "<<file<<endl;
	}
	return ;
}

void testMy3DSWriter(string file)
{
	CMy3DSReader myReader;
	if(!myReader.Load(file.c_str()))
	{
		cout<<"failed to parse file: "<<file<<endl;
		return ;
	}
	CMy3DSWriter myWriter;
	myWriter.UserPrepareContentCMyReader((void*)&myReader);
	myWriter.Create("D:\\PCL_install_on_VS2008\\3DS\\man2.3ds");
}

void DisplayPointXYZRGB(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_disPC,int _pointProperty)
{
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(m_disPC);
	viewer->addPointCloud<pcl::PointXYZRGB> (m_disPC, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _pointProperty, "sample cloud");
	viewer->addCoordinateSystem (2.0);
	viewer->initCameraParameters ();

	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void DisplayTriangleMesh(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_disPC,std::vector<pcl::Vertices>& m_indices)
{
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addPolygonMesh<pcl::PointXYZRGB>(m_disPC, m_indices, "sample triangleMesh");
	viewer->addCoordinateSystem (2.0);
	viewer->initCameraParameters ();
	while (!viewer->wasStopped ())
	{
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
}

void testTriangleMesh()
{
	pcl::PCDReader pcdRead;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcdRead.read("D:\\PCL_install_on_VS2008\\pcds\\1.pcd",*pPC);
	// generate cross-links cell from PC
	CTriangleMesh triMesh(pPC);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pDis(new pcl::PointCloud<pcl::PointXYZRGB>);
	triMesh.FromPt2Cell();

	// generate triangle-mesh
	triMesh.FromCell2TriMesh();

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	
	// display triangle-mesh
	//std::vector<pcl::Vertices> m_indices;
	//triMesh.GetTriIndices(m_indices);
	//triMesh.m_pVertex->is_dense=true;
	//DisplayTriangleMesh(viewer,triMesh.m_pVertex,m_indices);
	
	// display Outlier PC
	/*triMesh.GetOutlierVertex(pDis);*/


	triMesh.FilterSmallPolygon();

	//// display PC
	//triMesh.GetVertex(pDis);
	//DisplayPointXYZRGB(viewer,pDis);
	
	// dump to 3DS.file
	CMy3DSWriter myWriter;
	myWriter.UserPrepareContentTriangleMesh((void*)&triMesh);
	myWriter.Create("D:\\PCL_install_on_VS2008\\3DS\\mesh3.3ds");

	return;
}

void 
setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose)
{
	Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f (0, 0, 0);
	Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f (0, 0, 1) + pos_vector;
	Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f (0, -1, 0);
	viewer.camera_.pos[0] = pos_vector[0];
	viewer.camera_.pos[1] = pos_vector[1];
	viewer.camera_.pos[2] = pos_vector[2];
	viewer.camera_.focal[0] = look_at_vector[0];
	viewer.camera_.focal[1] = look_at_vector[1];
	viewer.camera_.focal[2] = look_at_vector[2];
	viewer.camera_.view[0] = up_vector[0];
	viewer.camera_.view[1] = up_vector[1];
	viewer.camera_.view[2] = up_vector[2];
	viewer.updateCamera ();
}

void testNarf()
{
	// --------------------
	// -----Parameters-----
	// --------------------
	float angular_resolution = 0.5f;
	float support_size = 1.0f;
	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
	bool setUnseenToMaxRange = false;

	// ------------------------------------------------------------------
	// -----Read pcd file or create example point cloud if not given-----
	// ------------------------------------------------------------------
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	string ss1("D:\\PCL_install_on_VS2008\\pcds\\robot\\robot1.pcd");
	pcl::io::loadPCDFile (ss1.c_str(), *point_cloud_ptr);
	pcl::PointCloud<pcl::PointXYZRGB>& point_cloud = *point_cloud_ptr;
	pcl::PointCloud<pcl::PointWithViewpoint> far_ranges;
	Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());

	// -----------------------------------------------
	// -----Create RangeImage from the PointCloud-----
	// -----------------------------------------------
	float noise_level = 0.0;
	float min_range = 0.0f;
	int border_size = 1;
	boost::shared_ptr<pcl::RangeImage> range_image_ptr (new pcl::RangeImage);
	pcl::RangeImage& range_image = *range_image_ptr;   
	range_image.createFromPointCloud (point_cloud, angular_resolution, pcl::deg2rad (360.0f), pcl::deg2rad (180.0f),
		scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
	range_image.integrateFarRanges (far_ranges);
	if (setUnseenToMaxRange)
		range_image.setUnseenToMaxRange ();

	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	pcl::visualization::PCLVisualizer viewer ("3D Viewer");
	viewer.setBackgroundColor (1, 1, 1);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler (range_image_ptr, 0, 0, 0);
	viewer.addPointCloud (range_image_ptr, range_image_color_handler, "range image");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
	viewer.addCoordinateSystem (1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
	//viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
	viewer.initCameraParameters ();
	setViewerPose (viewer, range_image.getTransformationToWorldSystem ());

	// --------------------------
	// -----Show range image-----
	// --------------------------
	//pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
	//range_image_widget.setRangeImage(range_image);

	// --------------------------------
	// -----Extract NARF keypoints-----
	// --------------------------------
	//pcl::RangeImageBorderExtractor range_image_border_extractor;
	//pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
	//narf_keypoint_detector.setRangeImage (&range_image);
	//narf_keypoint_detector.getParameters ().support_size = support_size;
	////narf_keypoint_detector.getParameters ().add_points_on_straight_edges = true;
	////narf_keypoint_detector.getParameters ().distance_for_additional_points = 0.5;

	//pcl::PointCloud<int> keypoint_indices;
	//narf_keypoint_detector.compute (keypoint_indices);
	//std::cout << "Found "<<keypoint_indices.points.size ()<<" key points.\n";

	// ----------------------------------------------
	// -----Show keypoints in range image widget-----
	// ----------------------------------------------
	//for (size_t i=0; i<keypoint_indices.points.size (); ++i)
	//range_image_widget.markPoint (keypoint_indices.points[i]%range_image.width,
	//keypoint_indices.points[i]/range_image.width);

	// -------------------------------------
	// -----Show keypoints in 3D viewer-----
	// -------------------------------------
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>& keypoints = *keypoints_ptr;
	//keypoints.points.resize (keypoint_indices.points.size ());
	//for (size_t i=0; i<keypoint_indices.points.size (); ++i)
	//	keypoints.points[i].getVector3fMap () = range_image.points[keypoint_indices.points[i]].getVector3fMap ();

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> keypoints_color_handler (keypoints_ptr, 0, 255, 0);
	//viewer.addPointCloud<pcl::PointXYZRGB> (keypoints_ptr, keypoints_color_handler, "keypoints");
	//viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	//--------------------
	// -----Main loop-----
	//--------------------
	while (!viewer.wasStopped ())
	{
	//	range_image_widget.spinOnce ();  // process GUI events
		viewer.spinOnce ();
		pcl_sleep(0.01);
	}
}

void CaptureFrames()
{

	SimpleOpenNIViewer v;
	v.start();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPt(new pcl::PointCloud<pcl::PointXYZRGB>);

	string file_name_dir("D:\\MyProjects\\pcl\\darkworkroom\\");
	int i=0;
	string file_name;
	int c_input;
	bool exit1=false;
	while(1){
		while(v.getCloudPoint(m_pPt))
		{
			stringstream file_name_stream;
			file_name_stream<<file_name_dir<<++i<<".pcd";
			file_name_stream>>file_name;
			pcl::io::savePCDFile(file_name,*m_pPt);
			cout<<i<<" frame has been captured! Continue: 'c'; Exit:'e'.";
			cout.flush();
			c_input=getchar();
			if(c_input=='c'|| c_input=='C'){
				continue;
			}
			else if(c_input=='e'|| c_input=='E')
			{
				exit1=true;
				break;
			}
		}
		if(exit1)
			break;
	}
}