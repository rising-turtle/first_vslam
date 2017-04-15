#ifndef CMYGRAPHMANAGER
#define CMYGRAPHMANAGER

//#include "node.h"
#include "CMyNode.h"
#include <aislib/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <aislib/graph/loadEdges3d.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <map>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <memory> //for auto_ptr
#include "globaldefinitions.h"
#include "CPose3D.h"

namespace AIS = AISNavigation;

extern Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat);

class CMyGraphManager{
public:
	/// Start over with new graph
	void reset();

	// set the first node pose
	void initFirstNode(CPose3D initPose);

	///Throw the last node out, reoptimize
	void eraseNode(CMyNode* node);
	bool deleteOutRangeFrames(double x, double y, double z, double radius);

	void deleteLastFrame(); 
	void setMaxDepth(float max_depth);

public:
	CMyGraphManager();
	~CMyGraphManager();

	/// Add new node to the graph.
	/// Node will be included, if a valid transformation to one of the former nodes
	/// can be found. If appropriate, the graph is optimized
	/// graphmanager owns newNode after this call. Do no delete the object
	bool addNode(CMyNode* newNode); 
	
	///Flag to indicate that the graph is globally corrected after the addNode call.
	///However, currently optimization is done in every call anyhow
	bool freshlyOptimized_;
	//GLViewer glviewer_;

	std::map<int, CMyNode* > graph_;
	double latest_pose[6];

	void flannNeighbours();

	float Max_Depth;
	//void setMaxDepth(float max_depth);

	bool matched;
	// for saving matched images
	MatchingResult lastmr;

	bool usingIMU;
	CPose3D last_pose;
	CPose3D curr_pose;

	//protected:
public:
	std::vector < cv::DMatch > last_inlier_matches_;
	std::vector < cv::DMatch > last_matches_;
	/// The parameter max_targets determines how many potential edges are wanted
	/// max_targets < 0: No limit
	/// max_targets = 0: Compare to first frame only
	/// max_targets = 1: Compare to previous frame only
	/// max_targets > 1: Select intelligently
	std::vector<int> getPotentialEdgeTargets(const CMyNode* new_node, int max_targets);

	std::vector<int> getPotentialEdgeTargetsFeatures(const CMyNode* new_node, int max_targets);

	void optimizeGraph(bool online=true);
	void initializeHogman();
	bool addEdgeToHogman(AIS::LoadedEdge3D edge, bool good_edge);

	void resetGraph();

	void mergeAllClouds(pointcloud_type & merge);

	AIS::GraphOptimizer3D* optimizer_;

	// true if translation > 10cm or largest euler-angle>5 deg
	// used to decide if the camera has moved far enough to generate a new nodes
	bool isBigTrafo(const Eigen::Matrix4f& t);
	bool isBigTrafo(const Transformation3& t);

	bool isNoiseTrafo(const Eigen::Matrix4f& t);
	bool isNoiseTrafo(const Transformation3& t);

	/// get euler angles from 4x4 homogenous
	void static mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
	/// get translation-distance from 4x4 homogenous
	void static mat2dist(const Eigen::Matrix4f& t, double &dist);
	void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
	void testOptimizer();
	bool reset_request_;
	std::clock_t last_batch_update_;
	unsigned int marker_id;
	int last_matching_node_;
	bool batch_processing_runs_;

};


#endif