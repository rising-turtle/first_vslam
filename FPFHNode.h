#ifndef FPFHNODE_H
#define FPFHNODE_H

#include "preheader.h"
#include <Eigen/Core>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/pfh.h"
#include "pcl/keypoints/sift_keypoint.h"
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "matching_result.h" 

extern Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat);

class CFPFHNode
{
public:
	CFPFHNode();
	CFPFHNode(string file);
	~CFPFHNode();
public:
	int m_id; // id of node in the graph
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_downpc;
	pcl::PointCloud<pcl::Normal>::Ptr m_normal;
	pcl::PointCloud<pcl::PointWithScale>::Ptr m_keypt; 
	pcl::PointCloud<pcl::PFHSignature125>::Ptr m_des;
	pcl::KdTreeFLANN<pcl::PFHSignature125>::Ptr m_deskd;
	/////Compare the features of two nodes and compute the transformation
	MatchingResult matchNodePair(const CFPFHNode* older_node);
	void findPairsFlann(const CFPFHNode* older_node,vector<cv::DMatch>* out_matches);

	/////Compute the relative transformation between the nodes
	/////Do either max_ransac_iterations or half of it, 
	/////Iterations with more than half of the initial_matches 
	/////inlying, count twice. Iterations with more than 80% of 
	/////the initial_matches inlying, count threefold
	//bool getRelativeTransformationTo(const Node* target_node, 
	//	std::vector<cv::DMatch>* initial_matches,
	//	Eigen::Matrix4f& resulting_transformation, 
	//	float& rmse,
	//	std::vector<cv::DMatch>& matches,//for visualization?
	//	unsigned int max_ransac_iterations = 1000) const;

	//void buildFlannIndex();

	//// helper for ransac
	//// check for distances only if max_dist_cm > 0
	//template<class InputIterator>
	//Eigen::Matrix4f getTransformFromMatches(const Node* other_node, 
	//	InputIterator iter_begin,
	//	InputIterator iter_end,
	//	bool* valid = NULL, 
	//	float max_dist_m = -1
	//	) const;

	//// helper for ransac
	//void computeInliersAndError(const std::vector<cv::DMatch>& initial_matches,
	//	const Eigen::Matrix4f& transformation,
	//	const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
	//	const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
	//	std::vector<cv::DMatch>& new_inliers, //output var
	//	double& mean_error, vector<double>& errors,
	//	double squaredMaxInlierDistInM = 0.0009) const; //output var;

public:
	void downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out);
	void compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
		pcl::PointCloud<pcl::Normal>::Ptr &normals_out);
	void compute_PFH_features (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, 
		pcl::PointCloud<pcl::Normal>::Ptr &normals, 
		float feature_radius,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);
	void detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
		float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
		pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out);
	void compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, 
		pcl::PointCloud<pcl::Normal>::Ptr &normals, 
		pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out);
	void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
		pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
		std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);
	void find_fpfh_correspondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
		pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
		std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out);
	void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
		const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints);
	void visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
		const pcl::PointCloud<pcl::Normal>::Ptr normals);  
	template<typename PointT>
	void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
		const boost::shared_ptr<pcl::PointCloud<PointT> > keypoints1,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
		const boost::shared_ptr<pcl::PointCloud<PointT> > keypoints2,
		const std::vector<int> &correspondences,
		const std::vector<float> &correspondence_scores);
	void normals_demo (const char * filename);
	void keypoints_demo (const char * filename);
	void correspondences_demo (const char * filename_base);
	void fpfh_demo (const char * filename_base);
private:
};



#endif