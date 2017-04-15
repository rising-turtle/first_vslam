#include "CMyNode.h"
#include "MyICP.h"
//#include "node.h"
//#include <cmath>
//#include <ctime>
//#include <Eigen/Geometry>
//#include "pcl/ros/conversions.h"
//#include "pcl/point_types.h"
//#include <pcl/common/transformation_from_correspondences.h>
//#include <opencv2/highgui/highgui.hpp>

#include <pcl/filters/passthrough.h>

#include "CMyTransformation.h"
#include "QuickSort.h"
#include "globaldefinitions.h"

//#include <math.h>
//#include <float.h>
typedef unsigned int uint;
extern Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat);

CMyNode::CMyNode():m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP_pre(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP_pos(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	m_bIsKDok=false;
	m_flannIndex=NULL;
	m_preflannIndex=NULL;
	m_posflannIndex=NULL;
}
CMyNode::~CMyNode(){
	if(m_flannIndex!=NULL)
		delete m_flannIndex;
	if(m_preflannIndex!=NULL)
		delete m_preflannIndex;
	if(m_posflannIndex!=NULL)
		delete m_posflannIndex;
}
CMyNode::CMyNode(boost::shared_ptr<CSession>& pSession):m_pPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP_pre(new pcl::PointCloud<pcl::PointXYZRGB>),
m_tmpFP_pos(new pcl::PointCloud<pcl::PointXYZRGB>),
m_bIsKDok(false)
{
	m_flannIndex=NULL;
	m_preflannIndex=NULL;
	m_posflannIndex=NULL;

	// the first pose of session act as root node
	NodePose pPose=pSession->m_pathList[0];
	m_rootPose.setrpy(pPose.pose.roll,pPose.pose.pitch,pPose.pose.yaw);
	m_rootPose.setxyz(pPose.pose.x,pPose.pose.y,pPose.pose.z);
	//m_rootPose=CPose3D(pPose.pose.yaw,pPose.pose.pitch,pPose.pose.roll,pPose.pose.x,pPose.pose.y,pPose.pose.z);
	pPose=pSession->m_pathList[pSession->m_pathList.size()-1];
	m_lastPose=CPose3D(pPose.pose.yaw,pPose.pose.pitch,pPose.pose.roll,pPose.pose.x,pPose.pose.y,pPose.pose.z);

	// Filter the input CloudPoint 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_axis (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_voxel (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Create the filtering object
	pcl::PassThrough<pcl::PointXYZRGB> pass;
	pass.setInputCloud (pSession->m_pc);
	pass.setFilterFieldName ("y");
	pass.setFilterLimits (-3, 3);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered_axis);

	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (cloud_filtered_axis);
	sor.setLeafSize (0.1f, 0.1f, 0.1f);
	sor.filter (*cloud_filtered_voxel);

	// copy PC into RootNode
	//m_pPC->points.insert(m_pPC->points.end(),pSession->m_pc->points.begin(),pSession->m_pc->points.end());
	translatePCs(m_rootPose,cloud_filtered_voxel,true);
	m_pPC->points.insert(m_pPC->points.end(),cloud_filtered_voxel->points.begin(),cloud_filtered_voxel->points.end());

	// add all those descriptors to root node
	AreaPathList& pPathList=pSession->m_pathList;
	Area3DDescMap& pDesc=pSession->m_descMap;
	vector<PT> m_FeaturePts;
	vector<PT> m_GlobalFPs;
	vector<vector<float> >m_Features;

	if(pPathList.size()!=pDesc.size())
	{
		cout<<"error occurs at CMyNode.cpp!"<<endl;
		return;
	}
	// calculate location of each FP according to RootNode
	CPose3D RootPose;
	CPose3D RobotPose;
	CPose3D TransPose;
	PT tmpPT;

	// here we create preFeaturs and posFeature without Filter 
	int n_of_overlapped_nodes=global_bg_graph_threshold;
	int n_of_total_nodes=pPathList.size();
	int first_pos=n_of_total_nodes-n_of_overlapped_nodes;
	// [0,n_of_overlapped_nodes) is pre_features
	// [first_pos,n_of_total_nodes) is pos_features
	int n_of_pre_features=0;
	int n_of_pos_features=0;

	Eigen::Matrix4f	HM1;
	Eigen::Matrix4f	HM2;
	Eigen::Matrix4f HM3;

	for(size_t i=0;i<n_of_total_nodes;i++)
	{
		NodePose pNode=pPathList[i];
		Area3DDescMap::iterator it=pDesc.find(pNode.id);
		if(it==pDesc.end())
		{
			cout<<"error in Matched ID between FP and PL!"<<endl;
			return ;
		}
		if(i==0){
			RootPose.setrpy(pNode.pose.roll,pNode.pose.pitch,pNode.pose.yaw);
			RootPose.setxyz(pNode.pose.x,pNode.pose.y,pNode.pose.z);
		}
		RobotPose.setrpy(pNode.pose.roll,pNode.pose.pitch,pNode.pose.yaw);
		RobotPose.setxyz(pNode.pose.x,pNode.pose.y,pNode.pose.z);
		
		RobotPose.getHomogeneousMatrix(HM2);
		RootPose.getHomogeneousMatrix(HM1);
		HM3 = HM1.inverse()*HM2;
		TransPose=CPose3D(HM3);

		//TransPose=RobotPose-RootPose;
		//RobotPose.getHomogeneousMatrix(HM);
		
		TransPose.getHomogeneousMatrix(HM1);
		RobotPose.getHomogeneousMatrix(HM2);

		// Sort according to Global coordinates, but record Local coordinates
		vector<Feature3DDesc>& pFDesc=it->second;
		
		if(i<n_of_overlapped_nodes)
			n_of_pre_features+=pFDesc.size();
		if(i>=first_pos)
			n_of_pos_features+=pFDesc.size();

		float pt[4];
		float tt[4];
		vector<float> tmpFeatures;
		tmpFeatures.resize(64);
		for(int j=0;j<pFDesc.size();j++){
			// calculate Local coordinates
			pt[0]=pFDesc[j].xyz.x; pt[1]=pFDesc[j].xyz.y; pt[2]=pFDesc[j].xyz.z; pt[3]=1.0;
			Eigen::Map<Eigen::Vector4f> thispt=Eigen::Vector4f::Map(pt);
			Eigen::Map<Eigen::Vector4f> thatpt=Eigen::Vector4f::Map(tt);
			thatpt=HM1*thispt;
			tmpPT.x=thatpt(0);
			tmpPT.y=thatpt(1);
			tmpPT.z=thatpt(2);
			m_FeaturePts.push_back(tmpPT);

			// calculate Global coordinates
			thatpt=HM2*thispt;
			tmpPT.x=thatpt(0);
			tmpPT.y=thatpt(1);
			tmpPT.z=thatpt(2);
			m_GlobalFPs.push_back(tmpPT);

			// record content of each FeaturePTs
			for(int k=0;k<64;k++)
			{
				tmpFeatures[k]=pFDesc[j].desc[k];
			}
			m_Features.push_back(tmpFeatures);
		}
	}
	// copy the pre and pos overlapped features
	m_feature_descriptors_pre.create(n_of_pre_features,64,CV_32FC1);
	m_feature_descriptors_pos.create(n_of_pos_features,64,CV_32FC1);
	int n_of_all_features=m_Features.size();

	/*cout<<"n_of_pre_FPs: "<<n_of_pre_features<<endl;
	cout<<"n_of_pos_FPs: "<<n_of_pos_features<<endl;
	cout<<"n_of_all_FPs: "<<n_of_all_features<<endl;*/

	Eigen::Vector4f tmpFP1;
	for(int i=0;i<n_of_pre_features;i++)
	{
		pcl::PointXYZRGB tmpP;
		tmpP.x=tmpFP1[0]=m_FeaturePts[i].x;
		tmpP.y=tmpFP1[1]=m_FeaturePts[i].y;
		tmpP.z=tmpFP1[2]=m_FeaturePts[i].z;
		tmpFP1[3]=1.0;
		m_feature_locations_3d_pre.push_back(tmpFP1);
		m_tmpFP_pre->points.push_back(tmpP);
		for(int j=0;j<64;j++)
			m_feature_descriptors_pre.at<float>(i,j)=m_Features[i][j];
	}
	for(int i=n_of_pos_features;i>0;i--)
	{
		pcl::PointXYZRGB tmpP;
		tmpP.x=tmpFP1[0]=m_FeaturePts[n_of_all_features-i].x;
		tmpP.y=tmpFP1[1]=m_FeaturePts[n_of_all_features-i].y;
		tmpP.z=tmpFP1[2]=m_FeaturePts[n_of_all_features-i].z;
		tmpFP1[3]=1.0;
		m_feature_locations_3d_pos.push_back(tmpFP1);
		m_tmpFP_pos->points.push_back(tmpP);
		for(int j=0;j<64;j++)
			m_feature_descriptors_pos.at<float>(n_of_pos_features-i,j)=m_Features[n_of_all_features-i][j];
	}

	// filter similar feature points according to Global coordinates
	vector<int> FP_index;
	QuickSort<PT>(m_GlobalFPs,FP_index);
	//QuickSort<PT>(m_FeaturePts,FP_index);
	//displayVec<PT>(m_FeaturePts,FP_index);

	boost::dynamic_bitset<> bValidFP;
	bValidFP.resize(FP_index.size());
	bValidFP.set();
	FilterSimilarFeaturs(m_GlobalFPs,FP_index,bValidFP);
	//FilterSimilarFeaturs(m_FeaturePts,FP_index,bValidFP);

	Eigen::Vector4f tmpFP;
	float tmpfeature[64];

	// num of left features after filtering
	int filtered_features=bValidFP.count();// 
	m_feature_descriptors.create(filtered_features,64,CV_32FC1/*cv::DataType<double>::type*/);
	int index_of_features=0;

	for(int i=0;i<FP_index.size();i++)
	{
		if(bValidFP[i])
		{
			// record location of FP
			pcl::PointXYZRGB tmpP;
			int location_=FP_index[i];
			tmpFP[0]=tmpP.x=m_FeaturePts[location_].x;
			tmpFP[1]=tmpP.y=m_FeaturePts[location_].y;
			tmpFP[2]=tmpP.z=m_FeaturePts[location_].z;
			tmpFP[3]=1.0;

			// record Local coordinates of selected FPs
			m_tmpFP->points.push_back(tmpP);

			/*tmpFP[0]=m_GlobalFPs[location_].x;
			tmpFP[1]=m_GlobalFPs[location_].y;
			tmpFP[2]=m_GlobalFPs[location_].z;
			tmpFP[3]=1.0;*/
			m_feature_locations_3d.push_back(tmpFP);
	
			// record content of FP
			for(int j=0;j<64;j++)
			{
				m_feature_descriptors.at<float>(index_of_features,j)=m_Features[location_][j];
			}
			index_of_features++;
		}
	}
	/*cout<<"m_FeaturePts size: "<<m_FeaturePts.size()<<endl;
	cout<<"After Feature filter: "<<m_tmpFP->points.size()<<endl;*/
	//m_feature_descriptors.create(total_featurs,64,cv::DataType<double>::type);
	//int cv_row=0;
	//double tmpfeature[64];
	//for(it=pDesc.begin();it!=pDesc.end();it++)
	//{
	//	vector<Feature3DDesc>& pFDesc=it->second;
	//	for(int i=0;i<pFDesc.size();i++,cv_row++)
	//	{
	//		for(int j=0;j<64;j++)
	//		{
	//			tmpfeature[j]=(double)pFDesc[i].desc[j];
	//			//m_feature_descriptors.at<double>(i,j)=(double)(pFDesc[i].desc[j]);
	//		}
	//		unsigned char * pMi=m_feature_descriptors.ptr<unsigned char>(i);
	//		memcpy(pMi,tmpfeature,n_step);
	//		//memcpy(m_feature_descriptors.ptr<double>(n_step*i),tmpfeature,n_step/*64*sizeof(double)*/);
	//		//memcpy(m_feature_descriptors.data + n_step*i,tmpfeature,n_step/*64*sizeof(double)*/);
	//		//memcpy((void*)(&m_feature_descriptors[cv_row],pFDesc[i].desc,sizeof()))
	//	}
	//}
	/*if(cv_row!=total_featurs)
	{
		cout<<"There must be error!"<<endl;
	}*/
}
void CMyNode::translateFeatures(CPose3D& rootnode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_tmpFP)
{
	float pt[4];
	float tt[4];
	Eigen::Matrix4f	HM;
	rootnode.getHomogeneousMatrix(HM);
	for(size_t i=0;i<m_tmpFP->points.size();i++)
	{	
		pcl::PointXYZRGB& sp=m_tmpFP->points[i];
		pt[0]=sp.x; pt[1]=sp.y; pt[2]=sp.z; pt[3]=1.0;
		Eigen::Map<Eigen::Vector3f> thispt=Eigen::Vector3f::Map(pt);
		Eigen::Map<Eigen::Vector3f> thatpt=Eigen::Vector3f::Map(tt);
		thatpt=(Eigen::Affine3f)(HM)*thispt;
		sp.x=thatpt(0);
		sp.y=thatpt(1);
		sp.z=thatpt(2);
	}	
}
void CMyNode::translatePCs(CPose3D& rootnode,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_Ppc, bool inv)
{
	float pt[4];
	float tt[4];
	Eigen::Matrix4f	HM;
	Eigen::Matrix4f	HM1;
	rootnode.getHomogeneousMatrix(HM1);
	if(inv)
	{
		HM=HM1.inverse();
	}
	else
	{
		HM=HM1;
	}
	for(size_t i=0;i<m_Ppc->points.size();i++)
	{	
		pcl::PointXYZRGB& sp=m_Ppc->points[i];
		pt[0]=sp.x; pt[1]=sp.y; pt[2]=sp.z; pt[3]=1.0;
		Eigen::Map<Eigen::Vector3f> thispt=Eigen::Vector3f::Map(pt);
		Eigen::Map<Eigen::Vector3f> thatpt=Eigen::Vector3f::Map(tt);
		thatpt=(Eigen::Affine3f)(HM)*thispt;
		sp.x=thatpt(0);
		sp.y=thatpt(1);
		sp.z=thatpt(2);
	}	
}
void CMyNode::translateFeatures(CPose3D& rootnode, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >&m_features)
{
	float pt[4];
	float tt[4];
	Eigen::Matrix4f	HM;
	rootnode.getHomogeneousMatrix(HM);

	for(size_t i=0;i<m_features.size();i++)
	{	
		pt[0]=m_features[i][0]; pt[1]=m_features[i][1]; pt[2]=m_features[i][2]; pt[3]=1.0;
		Eigen::Map<Eigen::Vector3f> thispt=Eigen::Vector3f::Map(pt);
		Eigen::Map<Eigen::Vector3f> thatpt=Eigen::Vector3f::Map(tt);
		thatpt=(Eigen::Affine3f)(HM)*thispt;
		m_features[i][0]=thatpt(0);
		m_features[i][1]=thatpt(1);
		m_features[i][2]=thatpt(2);
	}
}

// Filter similar features in the Session
void CMyNode::FilterSimilarFeaturs(vector<PT>& m_FP,vector<int>& m_index, boost::dynamic_bitset<>& m_Valid)
{
	size_t N_=m_index.size();
	if(N_!=m_FP.size())
	{
		cout<<"Error occurs in FilterSimilarFeatures()!"<<endl;
		return; 
	}
	if(m_Valid.size()!=N_)
	{
		m_Valid.resize(N_);
	}
	m_Valid.set();
	int cout=m_Valid.size();
	//  
	int curr_index=0;
	int next_index=1;
	while(curr_index<N_){
		while(next_index<N_)
		{
			if(m_FP[m_index[curr_index]]._dis(m_FP[m_index[next_index]]) < gl_feature_filter_threshold) // close enough
			{
				m_Valid[next_index]=false; // discard this FP
				next_index++;
			}
			else break;
		}
		curr_index=next_index;
		next_index++;
	}
}
// build search structure for descriptor matching
void CMyNode::buildFlannIndex() {
	if(m_feature_descriptors.elemSize()<=0)
	{
		cout<<"failed to build KD tree!"<<endl;
		return;
	}
	//std::clock_t starttime=std::clock();
	// use same type as in http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
	m_flannIndex = new cv_flannIndex(m_feature_descriptors, cv::flann::KDTreeIndexParams(4));
	m_preflannIndex = new cv_flannIndex(m_feature_descriptors_pre, cv::flann::KDTreeIndexParams(4));
	m_posflannIndex = new cv_flannIndex(m_feature_descriptors_pos, cv::flann::KDTreeIndexParams(4));
	m_bIsKDok=true;
	//  ROS_DEBUG("Built flannIndex (address %p) for Node %i", flannIndex, this->id_);
	// ROS_INFO_STREAM_COND_NAMED(( (std::clock()-starttime) / (double)CLOCKS_PER_SEC) > 0.001, "timings", "buildFlannIndex runtime: "<< ( std::clock() - starttime ) / (double)CLOCKS_PER_SEC  <<"sec");
}

//TODO: This function seems to be resistant to paralellization probably due to knnSearch
int CMyNode::findPairsFlann(const CMyNode* other, vector<cv::DMatch>* matches,bool normal) {

	assert(matches->size()==0);
	if (other->m_bIsKDok == false) 
	{
		return -1;
	}

	// number of neighbours found (has to be two, see l. 57)
	const int k = 2;

	// compare
	// http://opencv-cocoa.googlecode.com/svn/trunk/samples/c/find_obj.cpp
	/*cv::Mat indices(m_feature_descriptors.rows, k, CV_32S);
	cv::Mat dists(m_feature_descriptors.rows, k, CV_32F);*/

	cv::Mat indices;
	cv::Mat dists;

	if(normal)
	{
		indices.create(m_feature_descriptors.rows,k,CV_32S);
		dists.create(m_feature_descriptors.rows,k,CV_32F);
	}
	else
	{
		indices.create(m_feature_descriptors_pre.rows,k,CV_32S);
		dists.create(m_feature_descriptors_pre.rows,k,CV_32F);
	}

	// normal match use all features
	if(normal){
		// get the best two neighbours
		other->m_flannIndex->knnSearch(m_feature_descriptors, indices, dists, k,
			cv::flann::SearchParams(64));
	}
	else
	{
		other->m_posflannIndex->knnSearch(m_feature_descriptors_pre,indices,dists,k,
			cv::flann::SearchParams(64));
	}

	int* indices_ptr = indices.ptr<int> (0);
	float* dists_ptr = dists.ptr<float> (0);

	cv::DMatch match;
	for (int i = 0; i < indices.rows; ++i) {
		if (dists_ptr[2 * i] < 0.6 * dists_ptr[2 * i + 1]) {
			match.queryIdx = i;
			match.trainIdx = indices_ptr[2 * i];
			match.distance = dists_ptr[2 * i];

			if(normal){
				assert(match.trainIdx < other->m_feature_descriptors.rows);
				assert(match.queryIdx < m_feature_descriptors.rows);
			}
			else
			{
				assert(match.trainIdx < other->m_feature_descriptors_pos.rows);
				assert(match.queryIdx < m_feature_descriptors_pre.rows);
			}

			matches->push_back(match);
		}
	}
	return matches->size();
}

///Find transformation with largest support, RANSAC style.
///Return false if no transformation can be found
bool CMyNode::getRelativeTransformationTo(const CMyNode* earlier_node,
									   std::vector<cv::DMatch>* initial_matches,
									   Eigen::Matrix4f& resulting_transformation,
									   float& rmse, 
									   std::vector<cv::DMatch>& matches,
									   //float min_inlier_ratio,
									   bool normal,
									   unsigned int ransac_iterations) {
   //ROS_INFO("unsigned int min_inlier_threshold %i", min_inlier_threshold);
   // std::clock_t starttime=std::clock();

   assert(initial_matches != NULL);

   // ROS_INFO("inlier_threshold: %d", min_inlier_threshold);


   matches.clear();

   // get 30% of initial matches as inliers (but cut off at 30)
   float pct=0; //= 0.7;
   uint min_fix=0;// = 90;

   
   if(normal){ // global match
	/*   pct = 0.25;
	   min_fix = 20;*/
	    pct = 0.85;
	    min_fix = 110;
   }
   else{ // adj match
	   pct = 0.5;
	   min_fix = 60;
   }

   uint min_inlier_threshold = int(initial_matches->size()*pct);
   min_inlier_threshold = min(min_inlier_threshold,min_fix);
   min_inlier_threshold = max(min_inlier_threshold,global_min_inliers);


   uint min_feature_cnt = global_min_inliers;

   if(initial_matches->size() <= min_feature_cnt){ 
	   return false;
   }
   std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
   double inlier_error; //all squared errors
   srand((long)std::clock());

   // a point is an inlier if it's no more than max_dist_m m from its partner apart
   //const float max_dist_m = 0.03;
   // 2012/3/5_ZH for session we set 0.2
   const float max_dist_m = 0.5;

   const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
   vector<double> errors;
   vector<double> temp_errorsA;

   double best_error = 1e6;
   uint best_inlier_cnt = 0;

   int valid_iterations = 0;
   Eigen::Matrix4f transformation;

   // best values of all iterations (including invalids)
   double best_error_invalid = 1e6;
   uint best_inlier_invalid = 0;

   for (uint n_iter = 0; n_iter < ransac_iterations; n_iter++) {
	   //generate a map of samples. Using a map solves the problem of drawing a sample more than once

	   std::set<cv::DMatch> sample_matches;
	   std::vector<cv::DMatch> sample_matches_vector;
	   double little_dis=0.00001;
	   while(sample_matches.size() < sample_size){
		   int id = rand() % initial_matches->size();
		   if(initial_matches->at(id).distance < 1e-6)
		   {
			   initial_matches->at(id).distance += little_dis; 
			   little_dis+=little_dis;
		   }
		   sample_matches.insert(initial_matches->at(id));
		   sample_matches_vector.push_back(initial_matches->at(id));
	   }

	   bool valid;

	   transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),normal,&valid,max_dist_m);

	   // valid is false iff the sampled points aren't inliers themself 
	   if (!valid)
		   continue;
	   if(normal){
		   computeInliersAndError(*initial_matches, transformation, this->m_feature_locations_3d, 
			   earlier_node->m_feature_locations_3d, inlier, inlier_error,  /*output*/
			   temp_errorsA, max_dist_m*max_dist_m); /*output*/
	   }
	   else
	   {
		   computeInliersAndError(*initial_matches, transformation, this->m_feature_locations_3d_pre, 
			   earlier_node->m_feature_locations_3d_pos, inlier, inlier_error,  /*output*/
			   temp_errorsA, max_dist_m*max_dist_m); /*output*/
	   }

	   // check also invalid iterations
	   if (inlier.size() > best_inlier_invalid)
	   {
		   best_inlier_invalid = inlier.size();
		   best_error_invalid = inlier_error;
	   }

	   // ROS_INFO("iteration %d  cnt: %d, best: %d,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);

	   if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
			   continue;
	   }
	   valid_iterations++;
	   assert(inlier_error>0);

	   //Performance hacks:
	   ///Iterations with more than half of the initial_matches inlying, count twice
	   if (inlier.size() > initial_matches->size()*0.5) n_iter++;
	   ///Iterations with more than 80% of the initial_matches inlying, count threefold
	   if (inlier.size() > initial_matches->size()*0.8) n_iter++;

	   if (inlier_error < best_error) { //copy this to the result
		   resulting_transformation = transformation;
		   matches = inlier;
		   assert(matches.size()>= min_inlier_threshold);
		   best_inlier_cnt = inlier.size();
		   //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
		   rmse = inlier_error;
		   errors = temp_errorsA;
		   best_error = inlier_error;
		   // ROS_INFO("  new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);

	   }else
	   {
		   // ROS_INFO("NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
	   }

	   //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
	   double new_inlier_error;

	   transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end(),normal); // compute new trafo from all inliers:
	   if(normal){
		   computeInliersAndError(*initial_matches, transformation,
			   this->m_feature_locations_3d, earlier_node->m_feature_locations_3d,
			   inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);
	   }
	   else
	   {
		   computeInliersAndError(*initial_matches, transformation,
			   this->m_feature_locations_3d_pre, earlier_node->m_feature_locations_3d_pos,
			   inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);
	   }
	   // check also invalid iterations
	   if (inlier.size() > best_inlier_invalid)
	   {
		   best_inlier_invalid = inlier.size();
		   best_error_invalid = inlier_error;
	   }

	   if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
			   continue;
	   }

	   assert(new_inlier_error>0);

	   if (new_inlier_error < best_error) 
	   {
		   resulting_transformation = transformation;
		   matches = inlier;
		   assert(matches.size()>= min_inlier_threshold);
		   //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
		   rmse = new_inlier_error;
		   errors = temp_errorsA;
		   best_error = new_inlier_error;
	   }else
	   {
	   }
   } 

   return matches.size() >= min_inlier_threshold;
}
/////Find transformation with largest support, RANSAC style.
/////Return false if no transformation can be found
//bool CMyNode::getRelativeTransformationTo(const CMyNode* earlier_node,
//    std::vector<cv::DMatch>* initial_matches,
//    Eigen::Matrix4f& resulting_transformation,
//    float& rmse, 
//    std::vector<cv::DMatch>& matches,
//    //float min_inlier_ratio,
//    unsigned int ransac_iterations){
//  //ROS_INFO("unsigned int min_inlier_threshold %i", min_inlier_threshold);
//  // std::clock_t starttime=std::clock();
//
//  assert(initial_matches != NULL);
//
//  // ROS_INFO("inlier_threshold: %d", min_inlier_threshold);
//
//
//  matches.clear();
//  
//  // get 30% of initial matches as inliers (but cut off at 30)
//  float pct = 0.2;
//  uint min_fix = 30;
//  
//  uint min_inlier_threshold = int(initial_matches->size()*pct);
//  min_inlier_threshold = min(min_inlier_threshold,min_fix);
//  min_inlier_threshold = max(min_inlier_threshold,global_min_inliers);
//
//    
//  uint min_feature_cnt = global_min_inliers;
//
//  if(initial_matches->size() <= min_feature_cnt){ 
////    ROS_INFO("Only %d feature matches between %d and %d (minimal: %i)",(int)initial_matches->size() , this->id_, earlier_node->id_, min_feature_cnt);
//    return false;
//  }
//  std::vector<cv::DMatch> inlier; //holds those feature correspondences that support the transformation
//  double inlier_error; //all squared errors
//  srand((long)std::clock());
//  
//  // a point is an inlier if it's no more than max_dist_m m from its partner apart
//  const float max_dist_m = 0.03;
//  const unsigned int sample_size = 3;// chose this many randomly from the correspondences:
//  vector<double> errors;
//  vector<double> temp_errorsA;
//
//  double best_error = 1e6;
//  uint best_inlier_cnt = 0;
//
//  int valid_iterations = 0;
//  Eigen::Matrix4f transformation;
//
//  // best values of all iterations (including invalids)
//  double best_error_invalid = 1e6;
//  uint best_inlier_invalid = 0;
//
//  
//  
//  //ROS_INFO("running %i iterations with %i initial matches, min_match: %i, max_error: %.2f", (int) ransac_iterations, (int) initial_matches->size(), (int) min_inlier_threshold, max_dist_m*100 );
//
//  for (uint n_iter = 0; n_iter < ransac_iterations; n_iter++) {
//    //generate a map of samples. Using a map solves the problem of drawing a sample more than once
//
//    // ROS_INFO("iteration %d of %d", n_iter,ransac_iterations);
//
//    std::set<cv::DMatch> sample_matches;
//    std::vector<cv::DMatch> sample_matches_vector;
//    while(sample_matches.size() < sample_size){
//      int id = rand() % initial_matches->size();
//	  if(initial_matches->at(id).distance < 1e-6)
//		  initial_matches->at(id).distance += 0.001; 
//      sample_matches.insert(initial_matches->at(id));
//      sample_matches_vector.push_back(initial_matches->at(id));
//    }
//
//    bool valid;
//
//    transformation = getTransformFromMatches(earlier_node, sample_matches.begin(), sample_matches.end(),&valid,max_dist_m);
//
//    // valid is false iff the sampled points aren't inliers themself 
//    if (!valid)
//      continue;
//
//    computeInliersAndError(*initial_matches, transformation, this->m_feature_locations_3d, 
//        earlier_node->m_feature_locations_3d, inlier, inlier_error,  /*output*/
//        temp_errorsA, max_dist_m*max_dist_m); /*output*/
//
//    // check also invalid iterations
//    if (inlier.size() > best_inlier_invalid)
//    {
//      best_inlier_invalid = inlier.size();
//      best_error_invalid = inlier_error;
//    }
//
//    // ROS_INFO("iteration %d  cnt: %d, best: %d,  error: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100);
//
//    if(inlier.size() < min_inlier_threshold || inlier_error > max_dist_m){
//      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
//      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
//      continue;
//    }
//    // ROS_INFO("Refining iteration from %i samples: all matches: %i, inliers: %i, inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), inlier_error);
//    valid_iterations++;
//    //if (inlier_error > 0) ROS_ERROR("size: %i", (int)temp_errorsA.size());
//    assert(inlier_error>0);
//
//    //Performance hacks:
//    ///Iterations with more than half of the initial_matches inlying, count twice
//    if (inlier.size() > initial_matches->size()*0.5) n_iter++;
//    ///Iterations with more than 80% of the initial_matches inlying, count threefold
//    if (inlier.size() > initial_matches->size()*0.8) n_iter++;
//
//
//
//    if (inlier_error < best_error) { //copy this to the result
//      resulting_transformation = transformation;
//      matches = inlier;
//      assert(matches.size()>= min_inlier_threshold);
//      best_inlier_cnt = inlier.size();
//      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
//      rmse = inlier_error;
//      errors = temp_errorsA;
//      best_error = inlier_error;
//      // ROS_INFO("  new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
//
//    }else
//    {
//      // ROS_INFO("NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.4f, bestError: %.4f",n_iter, inlier.size(), best_inlier_cnt, inlier_error, best_error);
//    }
//
//    //int max_ndx = min((int) min_inlier_threshold,30); //? What is this 30?
//    double new_inlier_error;
//
//    transformation = getTransformFromMatches(earlier_node, matches.begin(), matches.end()); // compute new trafo from all inliers:
//    computeInliersAndError(*initial_matches, transformation,
//        this->m_feature_locations_3d, earlier_node->feature_locations_3d_,
//        inlier, new_inlier_error, temp_errorsA, max_dist_m*max_dist_m);
//
//    // ROS_INFO("asd recomputed: inliersize: %i, inlier error: %f", (int) inlier.size(),100*new_inlier_error);
//
//
//    // check also invalid iterations
//    if (inlier.size() > best_inlier_invalid)
//    {
//      best_inlier_invalid = inlier.size();
//      best_error_invalid = inlier_error;
//    }
//
//    if(inlier.size() < min_inlier_threshold || new_inlier_error > max_dist_m){
//      //inlier.size() < ((float)initial_matches->size())*min_inlier_ratio || 
//      // ROS_INFO("Skipped iteration: inliers: %i (min %i), inlier_error: %.2f (max %.2f)", (int)inlier.size(), (int) min_inlier_threshold,  inlier_error*100, max_dist_m*100);
//      continue;
//    }
//    // ROS_INFO("Refined iteration from %i samples: all matches %i, inliers: %i, new_inlier_error: %f", (int)sample_size, (int)initial_matches->size(), (int)inlier.size(), new_inlier_error);
//
//    assert(new_inlier_error>0);
//
//    if (new_inlier_error < best_error) 
//    {
//      resulting_transformation = transformation;
//      matches = inlier;
//      assert(matches.size()>= min_inlier_threshold);
//      //assert(matches.size()>= ((float)initial_matches->size())*min_inlier_ratio);
//      rmse = new_inlier_error;
//      errors = temp_errorsA;
//      best_error = new_inlier_error;
//      // ROS_INFO("  improved: new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
//    }else
//    {
//      // ROS_INFO("improved: NO new best iteration %d  cnt: %d, best_inlier: %d,  error: %.2f, bestError: %.2f",n_iter, inlier.size(), best_inlier_cnt, inlier_error*100, best_error*100);
//    }
//  } //iterations
//return matches.size() >= min_inlier_threshold;
//}

void CMyNode::computeInliersAndError(const std::vector<cv::DMatch>& matches,
								  const Eigen::Matrix4f& transformation,
								  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
								  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
								  std::vector<cv::DMatch>& inliers, //output var
								  double& mean_error,
								  vector<double>& errors,
								  double squaredMaxInlierDistInM) { //output var

	  std::clock_t starttime=std::clock();

	  inliers.clear();
	  errors.clear();

	  vector<pair<float,int> > dists;
	  std::vector<cv::DMatch> inliers_temp;

	  assert(matches.size() > 0);
	  mean_error = 0.0;
	  for (unsigned int j = 0; j < matches.size(); j++){ //compute new error and inliers

		  unsigned int this_id = matches[j].queryIdx;
		  unsigned int earlier_id = matches[j].trainIdx;

		  Eigen::Vector4f vec = (transformation * origins[this_id]) - earlier[earlier_id];

		  double error = vec.dot(vec);

		  if(error > squaredMaxInlierDistInM)
			  continue; //ignore outliers


		  error = sqrt(error);
		  dists.push_back(pair<float,int>(error,j));
		  inliers_temp.push_back(matches[j]); //include inlier

		  mean_error += error;
		  errors.push_back(error);
	  }

	  if (inliers_temp.size()==0){
		  mean_error = -1;
		  inliers = inliers_temp;
	  }
	  else
	  {
		  mean_error /= inliers_temp.size();

		  // sort inlier ascending according to their error
		  sort(dists.begin(),dists.end());

		  inliers.resize(inliers_temp.size());
		  for (unsigned int i=0; i<inliers_temp.size(); i++){
			  inliers[i] = matches[dists[i].second];
		  }
	  }
}
/*
template<class InputIterator, class T>
InputIterator find ( InputIterator first, InputIterator last, const T& value )
{
for ( ;first!=last; first++) if ( *first==value ) break;
return first;
}
*/
template<class InputIterator>
Eigen::Matrix4f CMyNode::getTransformFromMatches(const CMyNode* earlier_node,
											  InputIterator iter_begin,
											  InputIterator iter_end,
											  bool normal,
											  bool* valid, 
											  float max_dist_m) {

	//  pcl::TransformationFromCorrespondences tfc;
	  CMyTransformationFromCorrespondences tfc;

	  vector<Eigen::Vector3f> f;
	  vector<Eigen::Vector3f> t;
		
	  Eigen::Vector3f from;
	  Eigen::Vector3f to;

	  for ( ;iter_begin!=iter_end; iter_begin++) {
		  int this_id    = iter_begin->queryIdx;
		  int earlier_id = iter_begin->trainIdx;

		/*  Eigen::Vector3f from(this->m_feature_locations_3d[this_id][0],
			  this->m_feature_locations_3d[this_id][1],
			  this->m_feature_locations_3d[this_id][2]);
		  Eigen::Vector3f  to (earlier_node->m_feature_locations_3d[earlier_id][0],
			  earlier_node->m_feature_locations_3d[earlier_id][1],
			  earlier_node->m_feature_locations_3d[earlier_id][2]);*/
		  if(normal)
		  {
			from(0)=this->m_feature_locations_3d[this_id][0];
			from(1)=this->m_feature_locations_3d[this_id][1];
			from(2)=this->m_feature_locations_3d[this_id][2];
			to(0)=earlier_node->m_feature_locations_3d[earlier_id][0];
			to(1)=earlier_node->m_feature_locations_3d[earlier_id][1];
			to(2)=earlier_node->m_feature_locations_3d[earlier_id][2];
		  }
		  else
		  {
			  from(0)=this->m_feature_locations_3d_pre[this_id][0];
			  from(1)=this->m_feature_locations_3d_pre[this_id][1];
			  from(2)=this->m_feature_locations_3d_pre[this_id][2];
			  to(0)=earlier_node->m_feature_locations_3d_pos[earlier_id][0];
			  to(1)=earlier_node->m_feature_locations_3d_pos[earlier_id][1];
			  to(2)=earlier_node->m_feature_locations_3d_pos[earlier_id][2];
		  }

		  if (max_dist_m > 0)
		  {
			  f.push_back(from);
			  t.push_back(to);    
		  }

		  tfc.add(from, to);
	  }


	  // find smalles distance between a point and its neighbour in the same cloud
	  // je groesser das dreieck aufgespannt ist, desto weniger fallen kleine positionsfehler der einzelnen
	  // Punkte ist Gewicht!

	  if (max_dist_m > 0)
	  {  
		  float min_neighbour_dist = 1e6;
		  Eigen::Matrix4f foo;

		  *valid = true;
		  for (uint i=0; i<f.size(); i++)
		  {
			  float d_f = (f.at((i+1)%f.size())-f.at(i)).norm();

			  min_neighbour_dist = min(d_f,min_neighbour_dist);

			  float d_t = (t.at((i+1)%t.size())-t.at(i)).norm();

			  // distances should be equal since moving and rotating preserves distances
			  // 5 cm as threshold
			  if ( abs(d_f-d_t) > max_dist_m )
			  {
				  *valid = false;
				  return foo;
			  }
		  }

		  // check minimal size
		  if (min_neighbour_dist < 0.5)
		  {
		  }
	  }
	  // get relative movement from samples
	  return tfc.getTransformation().matrix();
}


//TODO: Merge this with processNodePair
MatchingResult CMyNode::matchNodePair(const CMyNode* older_node,bool normal){
	MatchingResult mr;
	const unsigned int min_matches = global_min_inliers; // minimal number of feature correspondences to be a valid candidate for a link
	// std::clock_t starttime=std::clock();


	//double  fm_start_t = ::GetTickCount();
	this->findPairsFlann(older_node, &mr.all_matches,normal); 
	//gl_pf_fm += ::GetTickCount() - fm_start_t; 

	//ROS_DEBUG("found %i inital matches",(int) mr.all_matches.size());
	if (mr.all_matches.size() < min_matches){
		//ROS_INFO("Too few inliers: Adding no Edge between %i and %i. Only %i correspondences to begin with.",
		//older_node->id_,this->id_,(int)mr.all_matches.size());
	} 
	else {

		//double id_start_t = ::GetTickCount();
		bool out = getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches,normal);
		//gl_pf_id += ::GetTickCount() - id_start_t; 
		if(!out){
			// if (!getRelativeTransformationTo(older_node,&mr.all_matches, mr.ransac_trafo, mr.rmse, mr.inlier_matches) ){ // mr.all_matches.size()/3
			//ROS_INFO("Found no valid trafo, but had initially %d feature matches",(int) mr.all_matches.size());


			// for debug
			cerr<<"bad Alignment form last frame with matched-inliers " << mr.inlier_matches.size()<<endl;

		} else  {
			cerr<<"matched!"<<endl;
			// + here we want to use correspond points
			vector<pcl::TMatchingPair> cors;

			// for debug
			/*ofstream f("D:\\PCL_install_on_VS2008\\match_points.txt");
			f<<"that " << "this"<<endl;*/

			for(std::vector<cv::DMatch>::iterator it = mr.inlier_matches.begin();
				it!= mr.inlier_matches.end(); it++)
			{
				Eigen::Vector4f src;
				Eigen::Vector4f dst;
				if(normal){
					 src = m_feature_locations_3d[(*it).queryIdx];
					 dst = older_node->m_feature_locations_3d[(*it).trainIdx];
				}
				else
				{
					src = m_feature_locations_3d_pre[(*it).queryIdx];
					dst = older_node->m_feature_locations_3d_pos[(*it).trainIdx];
				}
			/*	PT src = (m_FeaturePts[(*it).queryIdx]);
				PT dst = (older_node->m_FeaturePts[(*it).trainIdx]);*/

				//f<<src[0]<<" "<<src[1]<<" "<<src[2]<<""<<dst[0]<<" "<<dst[1]<<" "<<dst[2]<<endl;
				/*cors.push_back(pcl::TMatchingPair(0,0,older_node->feature_locations_3d_[(*it).trainIdx].x,older_node->feature_locations_3d_[(*it).trainIdx].y,\
				older_node->feature_locations_3d_[(*it).trainIdx].z,feature_locations_3d_[(*it).queryIdx].x,feature_locations_3d_[(*it).queryIdx].y,\
				feature_locations_3d_[(*it).queryIdx].z));*/
				cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));
			}
			pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
			CPose3D finalPose;


			//double me_start_t = ::GetTickCount();
			icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
			//gl_pf_me += ::GetTickCount() - me_start_t;

			/*cout<<"after leastSquareErrorRigidTransformation6D"<<endl;
			finalPose.output(std::cout);
			finalPose.displayROT();*/

			// debug information
			//cout<<"All matched pairs of points :"<<cors.size()<<endl;
			//cout<<"compute PoseInfo: "<<endl;
			//finalPose.output(std::cout);

			//mr.final_trafo = mr.ransac_trafo;
			finalPose.getHomogeneousMatrix(mr.final_trafo);
			mr.edge.id1 = older_node->m_id;//and we have a valid transformation
			mr.edge.id2 = this->m_id; //since there are enough matching features,
			mr.edge.mean = eigen2Hogman(mr.final_trafo);//we insert an edge between the frames
			mr.edge.informationMatrix =   Matrix6::eye(mr.inlier_matches.size()*mr.inlier_matches.size()); //TODO: What do we do about the information matrix? Scale with inlier_count. Should rmse be integrated?)
		}
	}
	return mr;
}

ostream& operator<<(ostream& out,PT& pt){
	return pt.output(out);
}
