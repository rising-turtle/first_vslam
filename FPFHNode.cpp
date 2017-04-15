#include "FPFHNode.h"
#include "MyICP.h"
#include <pcl/features/fpfh.h>

CFPFHNode::CFPFHNode():
m_pc(new pcl::PointCloud<pcl::PointXYZRGB>),
m_downpc(new pcl::PointCloud<pcl::PointXYZRGB>),
m_normal(new pcl::PointCloud<pcl::Normal>),
m_keypt(new pcl::PointCloud<pcl::PointWithScale>),
m_des(new pcl::PointCloud<pcl::PFHSignature125>),
m_deskd(new pcl::KdTreeFLANN<pcl::PFHSignature125>){}
CFPFHNode::~CFPFHNode(){}

CFPFHNode::CFPFHNode(string file):
m_pc(new pcl::PointCloud<pcl::PointXYZRGB>),
m_downpc(new pcl::PointCloud<pcl::PointXYZRGB>),
m_normal(new pcl::PointCloud<pcl::Normal>),
m_keypt(new pcl::PointCloud<pcl::PointWithScale>),
m_des(new pcl::PointCloud<pcl::PFHSignature125>),
m_deskd(new pcl::KdTreeFLANN<pcl::PFHSignature125>)
{
	// Load the pair of point clouds
	pcl::io::loadPCDFile (file.c_str(), *m_pc);
	if(m_pc->points.size()<=0)
	{
		cout<<"file: "<<file<<" is not valid!"<<endl;
		return ;
	}
	// Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;
	downsample (m_pc, voxel_grid_leaf_size, m_downpc);

	// Compute surface normals
	const float normal_radius = 0.03;
	compute_surface_normals (m_downpc, normal_radius, m_normal);

	// Compute keypoints
	const float min_scale = 0.01;
	const int nr_octaves = 3;
	const int nr_octaves_per_scale = 3;
	const float min_contrast = 10.0;
	detect_keypoints (m_pc, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, m_keypt);

	// Compute PFH features
	const float feature_radius = 0.08;
	compute_PFH_features_at_keypoints (m_downpc, m_normal, m_keypt, feature_radius, m_des);
	
	// Build PFH Kd_Tree
	m_deskd->setInputCloud(m_des);
}


MatchingResult CFPFHNode::matchNodePair(const CFPFHNode* older_node)
{
	MatchingResult mr;
	const unsigned int min_matches = 10;//global_min_inliers; // minimal number of feature correspondences to be a valid candidate for a link

	// get matched pairs
	this->findPairsFlann(older_node, &mr.all_matches); 
	




	vector<pcl::TMatchingPair> cors;
	for(std::vector<cv::DMatch>::iterator it = mr.all_matches.begin();
		it!= mr.all_matches.end(); it++)
	{
		pcl::PointWithScale& sp1=m_keypt->points[(*it).queryIdx];
		pcl::PointWithScale& sp2=older_node->m_keypt->points[(*it).trainIdx];
		/*Eigen::Vector4f src = m_keypt->points[(*it).queryIdx];
		Eigen::Vector4f dst = older_node->feature_locations_3d_[(*it).trainIdx];*/
		cors.push_back(pcl::TMatchingPair(0,0,sp1.x,sp1.y,sp1.z,sp2.x,sp2.y,sp2.z));

		//f<<src[0]<<" "<<src[1]<<" "<<src[2]<<""<<dst[0]<<" "<<dst[1]<<" "<<dst[2]<<endl;
		/*cors.push_back(pcl::TMatchingPair(0,0,older_node->feature_locations_3d_[(*it).trainIdx].x,older_node->feature_locations_3d_[(*it).trainIdx].y,\
		older_node->feature_locations_3d_[(*it).trainIdx].z,feature_locations_3d_[(*it).queryIdx].x,feature_locations_3d_[(*it).queryIdx].y,\
		feature_locations_3d_[(*it).queryIdx].z));*/
		//cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));

	}
	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	
	//mr.final_trafo = mr.ransac_trafo;
	finalPose.getHomogeneousMatrix(mr.final_trafo);
	mr.edge.id1 = older_node->m_id;//and we have a valid transformation
	mr.edge.id2 = this->m_id; //since there are enough matching features,
	mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
	mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)

	return mr;

}
void CFPFHNode::findPairsFlann(const CFPFHNode* older_node,vector<cv::DMatch>* out_matches)
{
	assert(out_matches->size()==0);
	if(older_node->m_deskd==NULL)
	{
		cout<<"older_node has no PFH kd_tree!"<<endl;
		return;
	}
	
	// Find feature correspondences
	//std::vector<int> correspondences;
	//std::vector<float> correspondence_scores;
	//find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);
	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 2;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	cv::DMatch match;
	for (size_t i = 0; i < m_des->size(); ++i)
	{
		older_node->m_deskd->nearestKSearch(*(this->m_des),i,k,k_indices,k_squared_distances);
		//correspondences[i] = k_indices[0];
		//correspondence_scores[i] = k_squared_distances[0];
		match.queryIdx=i;
		match.trainIdx=k_indices[0];

		assert(match.queryIdx<this->m_des->size());
		assert(match.trainIdx<older_node->m_des->size());

		if(k_squared_distances[0]<0.6*k_squared_distances[1])
		{
			match.distance=k_squared_distances[0];
			out_matches->push_back(match);
		}
	}
}

void CFPFHNode::downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> vox_grid;
	vox_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	vox_grid.setInputCloud (points);
	vox_grid.filter (*downsampled_out);
}

void CFPFHNode::compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
						 pcl::PointCloud<pcl::Normal>::Ptr &normals_out)
{
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;

	// Use a FLANN-based KdTree to perform neighborhood searches
	norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));

	// Specify the size of the local neighborhood to use when computing the surface normals
	norm_est.setRadiusSearch (normal_radius);

	// Set the input points
	norm_est.setInputCloud (points);

	// Estimate the surface normals and store the result in "normals_out"
	norm_est.compute (*normals_out);
}

void CFPFHNode::compute_PFH_features (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, 
					  pcl::PointCloud<pcl::Normal>::Ptr &normals, 
					  float feature_radius,
					  pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
	// Create a PFHEstimation object
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

	// Set it to use a FLANN-based KdTree to perform its neighborhood searches
	pfh_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));

	// Specify the radius of the PFH feature
	pfh_est.setRadiusSearch (feature_radius);

	// Set the input points and surface normals
	pfh_est.setInputCloud (points);  
	pfh_est.setInputNormals (normals);  

	// Compute the features
	pfh_est.compute (*descriptors_out);
}

void CFPFHNode::detect_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
				  float min_scale, int nr_octaves, int nr_scales_per_octave, float min_contrast,
				  pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints_out)
{
	pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift_detect;

	// Use a FLANN-based KdTree to perform neighborhood searches
	sift_detect.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));

	// Set the detection parameters
	sift_detect.setScales (min_scale, nr_octaves, nr_scales_per_octave);
	sift_detect.setMinimumContrast (min_contrast);

	// Set the input
	sift_detect.setInputCloud (points);

	// Detect the keypoints and store them in "keypoints_out"
	sift_detect.compute (*keypoints_out);
}

void CFPFHNode::compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, 
								   pcl::PointCloud<pcl::Normal>::Ptr &normals, 
								   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
								   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out)
{
	// Create a PFHEstimation object
	pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;

	// Set it to use a FLANN-based KdTree to perform its neighborhood searches
	pfh_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));

	// Specify the radius of the PFH feature
	pfh_est.setRadiusSearch (feature_radius);

	/* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
	* use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
	* we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to 
	* "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB 
	* values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
	*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);

	// Use all of the points for analyzing the local structure of the cloud
	pfh_est.setSearchSurface (points);  
	pfh_est.setInputNormals (normals);  

	// But only compute features at the keypoints
	pfh_est.setInputCloud (keypoints_xyzrgb);

	// Compute the features
	pfh_est.compute (*descriptors_out);
}

void CFPFHNode::find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
							  pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
							  std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
	// Resize the output vector
	correspondences_out.resize (source_descriptors->size ());
	correspondence_scores_out.resize (source_descriptors->size ());

	// Use a KdTree to search for the nearest matches in feature space
	pcl::KdTreeFLANN<pcl::PFHSignature125> descriptor_kdtree;
	descriptor_kdtree.setInputCloud (target_descriptors);

	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 1;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	for (size_t i = 0; i < source_descriptors->size (); ++i)
	{
		descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
		correspondences_out[i] = k_indices[0];
		correspondence_scores_out[i] = k_squared_distances[0];
	}
}
void CFPFHNode::find_fpfh_correspondences(pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
							   pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
							   std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out)
{
	// Resize the output vector
	//correspondences_out.resize (source_descriptors->size ());
	//correspondence_scores_out.resize (source_descriptors->size ());
	
	// Use a KdTree to search for the nearest matches in feature space
	pcl::KdTreeFLANN<pcl::FPFHSignature33> descriptor_kdtree;
	descriptor_kdtree.setInputCloud (target_descriptors);
	
	// Find the index of the best match for each keypoint, and store it in "correspondences_out"
	const int k = 2;
	std::vector<int> k_indices (k);
	std::vector<float> k_squared_distances (k);
	for (size_t i = 0; i < source_descriptors->size (); ++i)
	{
		descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
		if(k_squared_distances[0]<0.6*k_squared_distances[1])
		{
			correspondences_out.push_back(k_indices[0]);
			correspondence_scores_out.push_back(k_squared_distances[0]);
		}
	}
}
void CFPFHNode::visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
						  const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
	// Add the points to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud (points, "points");

	// Draw each keypoint as a sphere
	for (size_t i = 0; i < keypoints->size (); ++i)
	{
		// Get the point data
		const pcl::PointWithScale & p = keypoints->points[i];

		// Pick the radius of the sphere *
		float r = 2 * p.scale;
		// * Note: the scale is given as the standard deviation of a Gaussian blur, so a
		//   radius of 2*p.scale is a good illustration of the extent of the keypoint

		// Generate a unique string for each sphere
		std::stringstream ss ("keypoint");
		ss << i;

		// Add a sphere at the keypoint
		viz.addSphere (p, 2*p.scale, 1.0, 0.0, 0.0, ss.str ());
	}

	// Give control over to the visualizer
	viz.spin ();
}
void CFPFHNode::visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
						const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
						const pcl::PointCloud<pcl::Normal>::Ptr normals)                      
{
	// Add the points and normals to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud (points, "points");
	viz.addPointCloud (normal_points, "normal_points");

	viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");

	// Give control over to the visualizer
	viz.spin ();
}

template<typename PointT>
void CFPFHNode::visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
								const boost::shared_ptr<pcl::PointCloud<PointT> > keypoints1,
								const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
								const boost::shared_ptr<pcl::PointCloud<PointT> > keypoints2,
								const std::vector<int> &correspondences,
								const std::vector<float> &correspondence_scores)
{
	// We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
	// by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points

	// Create some new point clouds to hold our transformed data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointT>::Ptr keypoints_left (new pcl::PointCloud<PointT>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PointT>::Ptr keypoints_right (new pcl::PointCloud<PointT>);

	// Shift the first clouds' points to the left
	//const Eigen::Vector3f translate (0.0, 0.0, 0.3);
	const Eigen::Vector3f translate (0.4, 0.0, 0.0);
	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
	pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);

	// Shift the second clouds' points to the right
	pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
	pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);

	// Add the clouds to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.addPointCloud (points_left, "points_left");
	viz.addPointCloud (points_right, "points_right");

	// Compute the median correspondence score
	std::vector<float> temp (correspondence_scores);
	std::sort (temp.begin (), temp.end ());
	float median_score = temp[temp.size ()/2];

	// Draw lines between the best corresponding points
	for (size_t i = 0; i < correspondences.size()/*keypoints_left->points.size()*/; ++i)
	{
		if (correspondence_scores[i] > median_score)
		{
			continue; // Don't draw weak correspondences
		}

		// Get the pair of points
		const PointT & p_left = keypoints_left->points[i];
		const PointT & p_right = keypoints_right->points[correspondences[i]];

		// Generate a random (bright) color
		double r = (rand() % 100);
		double g = (rand() % 100);
		double b = (rand() % 100);
		double max_channel = std::max (r, std::max (g, b));
		r /= max_channel;
		g /= max_channel;
		b /= max_channel;

		// Generate a unique string for each line
		std::stringstream ss ("line");
		ss << i;

		// Draw the line
		viz.addLine (p_left, p_right, r, g, b, ss.str ());
	}

	// Give control over to the visualizer
	viz.spin ();
}

void CFPFHNode::normals_demo (const char * filename)
{
	// Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

	// Load a point cloud
	pcl::io::loadPCDFile (filename, *points);

	// Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;
	downsample (points, voxel_grid_leaf_size, downsampled);

	// Compute surface normals
	const float normal_radius = 0.03;
	compute_surface_normals (downsampled, normal_radius, normals);

	// Visualize the points and normals
	visualize_normals (points, downsampled, normals);

	return ;
}
void CFPFHNode::keypoints_demo (const char * filename)
{

	// Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints (new pcl::PointCloud<pcl::PointWithScale>);

	// Load a point cloud
	pcl::io::loadPCDFile (filename, *points);

	// Compute keypoints
	const float min_scale = 0.01;
	const int nr_octaves = 3;
	const int nr_octaves_per_scale = 3;
	const float min_contrast = 10.0;
	detect_keypoints (points, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints);

	// Visualize the point cloud and its keypoints
	visualize_keypoints (points, keypoints);

	return ;
}

void CFPFHNode::correspondences_demo (const char * filename_base)
{
	// Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors1 (new pcl::PointCloud<pcl::PFHSignature125>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2 (new pcl::PointCloud<pcl::PointWithScale>);
	pcl::PointCloud<pcl::PFHSignature125>::Ptr descriptors2 (new pcl::PointCloud<pcl::PFHSignature125>);

	// Load the pair of point clouds
	std::stringstream ss1, ss2;
	ss1 << filename_base << "1.pcd";
	pcl::io::loadPCDFile (ss1.str (), *points1);
	ss2 << filename_base << "2.pcd";
	pcl::io::loadPCDFile (ss2.str (), *points2);

	// Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;
	downsample (points1, voxel_grid_leaf_size, downsampled1);
	downsample (points2, voxel_grid_leaf_size, downsampled2);

	// Compute surface normals
	const float normal_radius = 0.03;
	compute_surface_normals (downsampled1, normal_radius, normals1);
	compute_surface_normals (downsampled2, normal_radius, normals2);

	// Compute keypoints
	const float min_scale = 0.01;
	const int nr_octaves = 3;
	const int nr_octaves_per_scale = 3;
	const float min_contrast = 10.0;
	detect_keypoints (points1, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints1);
	detect_keypoints (points2, min_scale, nr_octaves, nr_octaves_per_scale, min_contrast, keypoints2);

	// Compute PFH features
	const float feature_radius = 0.08;
	compute_PFH_features_at_keypoints (downsampled1, normals1, keypoints1, feature_radius, descriptors1);
	compute_PFH_features_at_keypoints (downsampled2, normals2, keypoints2, feature_radius, descriptors2);

	// Find feature correspondences
	std::vector<int> correspondences;
	std::vector<float> correspondence_scores;
	find_feature_correspondences (descriptors1, descriptors2, correspondences, correspondence_scores);

	// Print out ( number of keypoints / number of points )
	std::cout << "First cloud: Found " << keypoints1->size () << " keypoints "
		<< "out of " << downsampled1->size () << " total points." << std::endl;
	std::cout << "Second cloud: Found " << keypoints2->size () << " keypoints "
		<< "out of " << downsampled2->size () << " total points." << std::endl;

	// Visualize the two point clouds and their feature correspondences
	visualize_correspondences (points1, keypoints1, points2, keypoints2, correspondences, correspondence_scores);

	return ;
}

void CFPFHNode::fpfh_demo (const char * filename_base)
{
	// Create some new point clouds to hold our data
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled1 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals1 (new pcl::PointCloud<pcl::Normal>);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled2 (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr normals2 (new pcl::PointCloud<pcl::Normal>);
	
	// Create the FPFH estimation class, and pass the input dataset+normals to it
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh1;
	pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh2;

	// Load the pair of point clouds
	std::stringstream ss1, ss2;
	ss1 << filename_base << "1.pcd";
	pcl::io::loadPCDFile (ss1.str (), *points1);
	ss2 << filename_base << "2.pcd";
	pcl::io::loadPCDFile (ss2.str (), *points2);

	// Downsample the cloud
	const float voxel_grid_leaf_size = 0.01;
	downsample (points1, voxel_grid_leaf_size, downsampled1);
	downsample (points2, voxel_grid_leaf_size, downsampled2);

	// Compute surface normals
	const float normal_radius = 0.03;
	compute_surface_normals (downsampled1, normal_radius, normals1);
	compute_surface_normals (downsampled2, normal_radius, normals2);

	fpfh1.setInputCloud(downsampled1);
	fpfh1.setInputNormals(normals1);
	fpfh2.setInputCloud(downsampled2);
	fpfh2.setInputNormals(normals2);

	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr  tree1(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree2 (new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	//pcl::search::KdTree<PointXYZ>::Ptr tree1 (new pcl::search::KdTree<PointXYZ>);
	//pcl::search::KdTree<PointXYZ>::Ptr tree2 (new pcl::search::KdTree<PointXYZ>);

	fpfh1.setSearchMethod (tree1);
	fpfh2.setSearchMethod (tree2);


	// Output datasets
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1 (new pcl::PointCloud<pcl::FPFHSignature33> ());
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2 (new pcl::PointCloud<pcl::FPFHSignature33> ());

	// Use all neighbors in a sphere of radius 5cm
	// IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
	fpfh1.setRadiusSearch (0.05);
	fpfh2.setRadiusSearch (0.05);

	// Compute the features
	fpfh1.compute (*fpfhs1);
	fpfh2.compute (*fpfhs2);

	// 
	std::vector<int> correspondences;
	std::vector<float> correspondence_scores;
	find_fpfh_correspondences(fpfhs1,fpfhs2,correspondences,correspondence_scores);

	// Visualize the two point clouds and their feature correspondences
	visualize_correspondences (points2, downsampled2, points1, downsampled1, correspondences, correspondence_scores);

}