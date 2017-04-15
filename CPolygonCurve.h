#ifndef CPOLYGONCURVE_H
#define CPOLYGONCURVE_H

#include "AreaStore.h"
#include <vector>
#include <cmath>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#pragma once
typedef struct _Parameters{
public:
	struct _Parameters();

	/*These are for fusing triangles*/
	/************************************************************************/
	/*  For 10 degree displacement max cos(i) - cos(i+10)   < 0.175
	For 7 degree displacement max cos(i) - cos(i+7)   < 0.13  
	For 5 degree displacement max cos(i) - cos(i+5)   < 0.09
	For 3 degree displacement max cos(i) - cos(i+3)   < 0.053
	For 2 degree displacement max cos(i) - cos(i+2)   < 0.035
	For 1 degree displacement max cos(i) - cos(i+1)   < 0.018
	then if these Curves are also close to each other then Merge them*/
	/************************************************************************/
	double thresh_vector_angle; // 1 degree
	double thresh_dis_btw_planes ; 
	double thresh_dis_btw_vertex; 
	size_t    thresh_num_of_triangles; // 

	/*These are for fusing planes*/
	double min_dis_bt_planes; // min_dis_bt_planes
	double min_ang_bt_planes; // min_ang_bt_planes
	size_t N_of_new_plane; // N of new plane that can be added into pEigenVs

	/*These are for controlling N of points*/
	unsigned int N_of_points; // total number of points in a frame				
	size_t sample_step; // sampling step for each frame
	unsigned int N_of_first_frame; // number of points in first frame
	size_t sample_step_first_frame; // sampling step for first frame

	/*These are for merging points between different plane*/
	double ToleratedDistSqr; // ToleratedDistance to determine whether fuse or just insert
	int NTobeFused; // N to determine whether fuse or just insert

	/*For obtaining NVs through K nearest neighbours*/
	double search_radius; // search -radius for K nearest neighbours
	size_t K_N; // obtain NVs through K neighbours 
	size_t P_N; // Least number of points to form a local plane
}EEParameters;


#define LOWER_ASS(Low,V){if(V<Low) Low=V;}
#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}

using std::vector;
class CPolygonCurve{
public:
	typedef struct Vector_N{float nx;float ny;float nz;Vector_N():nz(0.9949874371),ny(0.1),nx(0.1){}}NV,HD; // Normal and Heading Vector struct
	typedef struct _Point3D{float x;float y;float z; float R; float G; float B;
	public:
		_Point3D(){}
		_Point3D(float& _x,float& _y,float& _z, float& _R, float& _G, float _B):x(_x),y(_y),z(_z),R(_R),G(_G),B(_B){}
		inline double _dis(float& fx, float & fy, float &fz) const{
			return (x-fx)*(x-fx) + (y-fy)*(y-fy) + (z-fz)*(z-fz);
		}}P3D; 
		//CPoint3D
		typedef struct _Triangle{P3D a; P3D b; P3D c;}Triangle;//Triangle 
public:
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pVertex;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pColorVertex;
	pcl::PointCloud<pcl::Normal>::Ptr m_pNormals;
public:
	CPolygonCurve();
	CPolygonCurve(pcl::PointXYZRGBNormal& pt);
	CPolygonCurve(pcl::PointXYZRGB& );
	void AddNewPTtoPlane(pcl::PointXYZRGBNormal& pt);
	void AddNewPTtoPlane(pcl::PointXYZRGB& pt);
	
	double m_var; // variance of this plane
	void CalculateVar();

	// boundary of this plane
	double m_upper_x,m_lower_x; // Bounder along x-axis
	double m_upper_y,m_lower_y;	// Bounder along y-axis
	double m_upper_z,m_lower_z;	// Bounder along z-axis

	inline bool UpdateBoundary(double x,double y, double z)
	{
		if( x>=m_lower_x && x<=m_upper_x && y>=m_lower_y && y<=m_upper_y && z>=m_lower_z && z<=m_upper_z)
			return false; // failed to update Boundary

		LOWER_ASS(m_lower_x,x);
		LOWER_ASS(m_lower_y,y);
		LOWER_ASS(m_lower_z,z);

		UPPER_ASS(m_upper_x,x);
		UPPER_ASS(m_upper_y,y);
		UPPER_ASS(m_upper_z,z);
		return true;	 // succeed to update Boundary
	}
	// only save Boundary points
	void ObtainBoundaryPoints();
	void AdjustBoundaryPoints();

	void NormalizeALL()
	{
		float factor=1./(float)m_num;
		GravityP.x*=factor; GravityP.y*=factor; GravityP.z*=factor;
		GravityP.R*=factor; GravityP.G*=factor; GravityP.B*=factor;
		NormalV.nx*=factor; NormalV.ny*=factor; NormalV.nz*=factor;
	}
	
	bool IsOverLapped( CPolygonCurve* );
	bool IsSimilarPlane( CPolygonCurve* );
	void MergeWithPlane(CPolygonCurve*);

	//		CPolygonCurve(CPoint3D&a, CPoint3D&b, CPoint3D&c);
	/*CPolygonCurve(std::vector<float>&x, std::vector<float>&y, std::vector<float>&z,CPoint3D& key, float R, float G, float B);
	CPolygonCurve(CPoint3D& a,CPoint3D& b, CPoint3D&c, vector<float>& R, vector<float>& G, vector<float>&B);
	CPolygonCurve(vector<CPoint3D>& points, vector<float>& R,vector<float>& G, vector<float>& B);*/
	~CPolygonCurve();
public: /*members*/ // Normally we use NV() and GP   
	NV NormalV; // Normal vector 
	NV TotalNormalV;
	P3D GravityP; // Gravity point in this Curve
	P3D TotalGP;
	double angle;// angle = cos(angle) = HD.* NV
	double dis; // Distance to (0,0,0)
	size_t m_num; // number of inserted points
	
	std::vector<P3D> pVertex; //Pointer to all points, This may be useless
	std::vector<P3D> pKeyVertex; // This is used to identify the key Vertex

	bool operator<(const CPolygonCurve& o){
		if(angle < o.angle) return true;
		else return false;
	}

	
	/*Calculate cross product */
	inline double CalculateCrossProduct(CPolygonCurve&o){
		double fx = NormalV.ny * o.NormalV.nz - NormalV.nz * o.NormalV.ny;
		double fy = -(NormalV.nx * o.NormalV.nz - NormalV.nz* o.NormalV.nx);
		double fz = NormalV.nx * o.NormalV.ny - NormalV.ny *o.NormalV.nx;
		return sqrt(fx*fx + fy*fy + fz*fz);
	}
	// Calculate Cross Product Vector of two vectors
	inline void CalculateCrossProduct(CPolygonCurve&o, NV& out_v){
		out_v.nx = NormalV.ny * o.NormalV.nz - NormalV.nz * o.NormalV.ny;
		out_v.ny = -(NormalV.nx * o.NormalV.nz - NormalV.nz* o.NormalV.nx);
		out_v.nz = NormalV.nx * o.NormalV.ny - NormalV.ny *o.NormalV.nx;
		NormalizeVector(out_v.nx,out_v.ny,out_v.nz);
	}

	inline double getD4(){
		return (GravityP.x*NormalV.nx + GravityP.y*NormalV.ny + GravityP.z*NormalV.nz);
	}

	/*Determine whether these two Curves are in the same Curve*/
	bool InTheSameSurface(CPolygonCurve& o);

	/*Merge two PolygonCurves */
	void MergeWith(CPolygonCurve& o);

	/*Fuse with New Points*/
	inline void FuseWithPoint(P3D& o){FuseWithPoint(o.x,o.y,o.z);}			
	void FuseWithPoint(float& fx,float& fy,float& fz);

	/*Insert with New Points*/
	inline void InsertWithPoint(P3D& o){InsertWithPoint(o.x,o.y,o.z,o.R,o.G,o.B);}
	void InsertWithPoint(float& fx,float& fy,float& fz,float& R, float& G, float& B);

	/*Determine whether to fuse with or just add*/
	inline bool NeedTofuse(P3D& o,int max_num,double err){return NeedTofuse(o.x,o.y,o.z,max_num,err);}
	bool NeedTofuse(float& fx,float& fy, float& fz, int max_num, double err);

	/*Adjust point in the same surface*/
	void AdjustPoint(float& fx, float& fy, float& fz);
	void AdjustAllPoints();

	/*Calculate Distance to (0,0,0)*/
	inline void CalculateDisToOrigin(){
		dis = fabs(NormalV.nx*GravityP.x + NormalV.ny*GravityP.y + NormalV.nz*GravityP.z);
	}
	inline float PointMultiply(NV& nv1, NV nv2)
	{
		return (nv1.nx*nv2.nx+nv1.ny*nv2.ny+nv1.nz*nv2.nz);
	}
	inline float PointMultiply(NV& nv1, pcl::PointXYZRGBNormal& nv2)
	{
		return (nv1.nx*nv2.normal_x+nv1.ny*nv2.normal_y+nv1.nz*nv2.normal_z);
	}
	inline float PointMultiply(NV& nv1, pcl::Normal& nv2)
	{
		return (nv1.nx*nv2.normal_x+nv1.ny*nv2.normal_y+nv1.nz*nv2.normal_z);
	}
	/*Using covariance matrix to calculate surface parameters Nv(Normal Vector) + Gp(Gravity Point)*/
	void CalcultePlaneParameters(vector<float>& x, vector<float>&y, vector<float>&z, NV& normalV, P3D& gp);
	void EstimateNormals(vector<float>& x, vector<float>&y, vector<float>&z,P3D& centroid,vector<double>& out_N );
	void CalculateCentroid(vector<float>& x, vector<float>&y, vector<float>&z, P3D & centroid);
	void CalculateCovarianceMatrix(vector<float>& x, vector<float>&y, vector<float>&z, P3D & centroid,  Eigen::Matrix3f& Cov);

	//inline void getKeyVertexIndex(size_t index, float& fx, float& fy, float&fz, float& R, float& G, float& B){
	//	if(index <0 || index > pKeyVertex.size()) {
	//		std::cout <<"false index to get KeyVertex"<<std::endl;
	//		return ;
	//	}
	//	fx = pKeyVertex[index].x; fy = pKeyVertex[index].y; fz = pKeyVertex[index].z;
	//	R = pKeyVertex[index].R; G = pKeyVertex[index].G; B = pKeyVertex[index].B;
	//}	
	/*Obtain all pKeyVertexes in <vector>out*/
	inline void getKeyVertex(std::vector<P3D>& out){
		for(size_t i=0;i<pKeyVertex.size();i++)
		{
			out.push_back(pKeyVertex[i]);
		}
	}
	/*Obtain NVs of this Curve*/
	inline void getNormalV(float& nx,float &ny, float&nz )
	{
		nx =NormalV.nx; ny = NormalV.ny; nz = NormalV.nz;
	}

public:
	static const int thresh_dis = 5; //Max distance of Point to surface 
	static HD _heading_vector;
	inline void NormalizeVector(NV& o){
		NormalizeVector(o.nx,o.ny,o.nz);
	}
	inline void NormalizeVector(float& x,float& y,float& z){
		//if(x>0){x*=-1.f; y*=-1.f; z*=-1.f;}
		float d = x*x + y*y + z*z;
		d = sqrt((double)d);
		x /=d; y/=d; z/=d; 
	}
	inline void CalculateAngle(const NV &nv)  {
		angle = nv.nx*_heading_vector.nx +nv.ny*_heading_vector.ny+nv.nz*_heading_vector.nz;
		return;
	}
	inline void CalculateAngle(const NV& nv,double& angle) const{
		angle = nv.nx*NormalV.nx + nv.ny*NormalV.ny + nv.nz*NormalV.nz;
		return ;
	}
public:
	void CalculateDistanceBetweenPlane(P3D& o, double& dis) const; // Calculate Distance of Point to surface
	void CalculateDistanceBetweenPlane(float& x,float& y, float& z, double& dis) const;

	inline void CalculateDistanceBetweenPoint(float &x,float &y, float& z, double& dis) const
	{ dis = GravityP._dis(x,y,z);}
	inline void CalculateDistanceBetweenPoint(P3D& o,double &dis) const{
		CalculateDistanceBetweenPoint(o.x,o.y,o.z,dis);
	}

	/*Update Vector with another plane*/
	void UpdateVectorWith(CPolygonCurve*);
	/*Update Gravity Point with another plane*/
	void UpdateGravityPointWith(CPolygonCurve*);

	void CalculateVector(P3D &a, P3D &b, P3D &c);
	void CalculateGravity();
private:

	CPolygonCurve(const CPolygonCurve&o);
	CPolygonCurve& operator = (const CPolygonCurve& o);
};
#endif