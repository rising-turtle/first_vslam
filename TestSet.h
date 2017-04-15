#ifndef TESTSET_H
#define TESTSET_H

#include "preheader.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include "CSession.h"

class CMyNode;
class CMyGraphManager;

using namespace std;
extern void testPose();
extern void testSession(string dir);
extern void testresampling(string dir);
extern void testMap(int W, int H, vector<vector<unsigned char> >& m_Map);
extern void testMem();
extern void tesetq();
extern boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals);
extern void getSessions(string dir,vector<boost::shared_ptr<CSession> >& sessions,unsigned int n_S);
extern void testPlanes(string dir);
extern void testRootNode(string dir);
extern void testMy3DSReader();
extern void testMy3DSWriter(string file);
extern void testSessionGraph(string dir);
extern void testDisplayFPs(string dir);
extern void testDisplayPCs(string dir);
extern void testLoopMatch(string dir);
extern void testCoordinateTranslate();
extern void testCicleNodes(string dir);
extern void testFeatureMatch(string dir);
extern void testDel();
extern void testicp1();
extern void addcolor(pcl::PointXYZRGB& sp,int index);
extern void addcoloroffeatures(CMyNode*,int color_index);
extern void DisplayPointXYZRGB(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ,int _pointProperty=2);
extern void DisplayTriangleMesh(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer,\
						 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_disPC,std::vector<pcl::Vertices>& m_indices);
extern void AdjustPoseInGraph(CMyGraphManager* );
extern void testTriangleMesh();


extern void testFPFH();
extern void testNarf();

extern void CaptureFrames();
#endif