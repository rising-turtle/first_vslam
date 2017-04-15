#ifndef CMYNODE_H
#define CMYNODE_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
//#include <image_geometry/pinhole_camera_model.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include "globaldefinitions.h"

#include "matching_result.h" 
#include <Eigen/StdVector>
#include "CPose3D.h"
#include "CSession.h"
#include <opencv2/core/core.hpp>
#include "AreaStore.h"
#include <boost/dynamic_bitset.hpp>

// Search structure for descriptormatching
typedef cv::flann::Index cv_flannIndex;

typedef	union _PT{
	struct{
		float x;
		float y;
		float z;
	};
	float a[3];	
	ostream& output(ostream& out){
		out<<"(x,y,z):"<<"("<<x<<","<<y<<","<<z<<")"<<endl;
		return out;
	}
	inline double _dis(const _PT& other){
		return (squres(x-other.x)+squres(y-other.y)+squres(z-other.z));
	}
	_PT(){}
	_PT(const _PT& other){
		for(int i=0;i<3;i++)
			a[i]=other.a[i];
	}
	const _PT& operator=(const _PT& other){
		for(int i=0;i<3;i++)
			a[i]=other.a[i];
		return other;
	}
	bool operator<(_PT& other){
		if(fabs(z-other.z)>=gl_feature_dis) //according to Z-axis 
		{
			return (z<other.z);
		}
		else if(fabs(sqrt(squresxoy())-sqrt(other.squresxoy()))>=gl_feature_dis) // according to dis between 
		{
			return (squresxoy()<other.squresxoy());
		}
		else
		{
			return ((x<other.x)|| ((x==other.x)&&y<other.y));
		}
	}
	inline double squres(){return (x*x+y*y+z*z);}
	inline double squresxoy(){return (x*x+y*y);}
	inline double squres(double a){return (a*a);}
}PT;		

// Act as pose-node, 
class CMyNode
{
public:
	CMyNode();
	CMyNode(boost::shared_ptr<CSession>& pSession);
	~CMyNode();

	
	int m_id; // id of this node
	//vector<PT> m_FeaturePts;
	CPose3D m_rootPose;
	CPose3D m_lastPose;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPC;
	
	void buildFlannIndex();
	int  findPairsFlann(const CMyNode* other, vector<cv::DMatch>* matches,bool normal=true);
	MatchingResult matchNodePair(const CMyNode* older_node,bool normal=true);
	
	cv::Mat m_feature_descriptors;
	cv::Mat m_feature_descriptors_pre;
	cv::Mat m_feature_descriptors_pos;
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > m_feature_locations_3d;
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > m_feature_locations_3d_pre; // for comparing with last Session
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > m_feature_locations_3d_pos; // for comparing with next Session
	
	cv_flannIndex* m_flannIndex;
	cv_flannIndex* m_preflannIndex;
	cv_flannIndex* m_posflannIndex;
	bool m_bIsKDok;

	void translateFeatures(CPose3D& rootnode, std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >&);
	void translateFeatures(CPose3D& rootnode, pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	void translatePCs(CPose3D& rootnode,pcl::PointCloud<pcl::PointXYZRGB>::Ptr&, bool inv=false);

	// Filter similar features in the Session
	void FilterSimilarFeaturs(vector<PT>& ,vector<int>& m_index, boost::dynamic_bitset<>& m_Valid);

	///Find transformation with largest support, RANSAC style.
	///Return false if no transformation can be found
	bool getRelativeTransformationTo(const CMyNode* earlier_node,
		std::vector<cv::DMatch>* initial_matches,
		Eigen::Matrix4f& resulting_transformation,
		float& rmse, 
		std::vector<cv::DMatch>& matches,
		//float min_inlier_ratio,
		bool normal,
		unsigned int ransac_iterations=1000);

	template<class InputIterator>
	Eigen::Matrix4f getTransformFromMatches(const CMyNode* earlier_node,
		InputIterator iter_begin,
		InputIterator iter_end,
		bool normal,
		bool* valid=NULL, 
		float max_dist_m=-1);
	void computeInliersAndError(const std::vector<cv::DMatch>& matches,
		const Eigen::Matrix4f& transformation,
		const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
		const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& earlier,
		std::vector<cv::DMatch>& inliers, //output var
		double& mean_error,
		vector<double>& errors,
		double squaredMaxInlierDistInM);


	// tmp for experiment
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpFP;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpFP_pre;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpFP_pos;
protected:
private:
};

// overlap operator <<
extern ostream& operator<<(ostream& out,PT& pt);

#endif