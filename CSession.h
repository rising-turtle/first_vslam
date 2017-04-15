#ifndef CSESSION_H
#define CSESSION_H
#pragma once
//#include <pcl/features/normal_3d.h>
//#include "pcl/kdtree/kdtree_flann.h"
//#include <pcl/features/integral_image_normal.h>
#include <boost/dynamic_bitset.hpp>
#include <set>
#include <vector>
#include "AreaStore.h"
#include "CPolygonCurve.h"
#include "CMy3DSWriter.h"
#include "CNewDynamicArea.h"

class CPolygonCurve;
template <typename PointT>
class CDynamicArea;
struct CPolygonCurve::Vector_N;


class CSession
{
public:
	//typedef pcl::KdTree<pcl::PointXYZRGB>::Ptr KdTreePtr;
	typedef pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr NVPtr;
	typedef boost::shared_ptr<CPolygonCurve> PPtr;

	CSession();
	~CSession();
	AreaStoreHdr m_fhdr;	// file info
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc; // point cloud of this session
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_render; // just for debug
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_rendercolor; // just for debug
	Area3DDescMap m_descMap;	// descriptor of all nodes in this session
	AreaPathList m_pathList;	// path list of all nodes in this session
	std::set<PPtr> m_plans;		// local planes 
	
	// bounding value along each axis
	float m_lower_x,m_upper_x;
	float m_lower_y,m_upper_y;
	float m_lower_z,m_upper_z;

	void SetBoundary(float l_x,float l_y,float l_z,float u_x,float u_y,float u_z);
	void SetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	void TransmitPC(unsigned char *pucImg, float *pfVertex);
	void TransmitPC2(unsigned char *pucImg, float *pfVertex);
	void TransmitPC3(unsigned char *pucImg, float *pfVertex);

	bool m_bRenderPlaneReady;

	void GeneratePlanes();
	//  1 Calculate Normal Vectors of each point
	void CalculateNV();
	//	2 Flood-Flow expand according to KD-tree
	void CalculatePlanes();
	bool IsSamePlane(pcl::PointXYZRGBNormal&,pcl::PointXYZRGBNormal&);
	//	3 Merge Planes which is overlapped along boundary
	void AdjustAllPlanes();
	void ObtainBoundaryPts();
	void AdjustBoundaryPts();
	void MergeOverlappedPlanes();
	//	4 Offer Render method(Boundary, NV, GP, Color)
	void RenderPlanes();

	// KDT search
	//KdTreePtr m_KDT;
	float m_fSearchRadiusKD;
	
	// NVs and PC
	NVPtr m_NVP;

	// Dynamic Area to Discretize PC
	boost::shared_ptr<CDynamicArea<pcl::PointXYZRGBNormal> > m_pDyArea;

	void GeneratePlanes2();
	//  1 Discretize pc into Cells
	void GenerateDynamicCellMap();
	//	2 Calculate Normal Vectors of each point
	//void CalculateNV2();
	//	3 Find planes along XYZ-axis on CellMap
	void FindPlanesAlongAxis();
	void FloodFlowOnCellMap();
	void DeleteInvalidPlanes();
	//	4 Merge Similar Planes 
	void MergeSimilarPlanes();
	void MergeSimilarLocalPlanes();
	//	5 Offer Render method(Boundary, NV, GP, Color)
	//void RenderPlanes()
	boost::dynamic_bitset<> m_valid_planes;

	std::vector<boost::shared_ptr<CPolygonCurve> > m_local_planes;

	void GeneratePlanes4();
	void GenerateDynamicCellMap3();
	void FloodFlowAlongAxis();
	void DeletePlanesAlongAxis();
	void MergeSimilarPlaneAlongAxis();
	void RenderColorAlongAxis();

	std::vector<boost::shared_ptr<CPolygonCurve> > m_xoy_planes2;
	std::vector<boost::shared_ptr<CPolygonCurve> > m_xoz_planes2;
	std::vector<boost::shared_ptr<CPolygonCurve> > m_yoz_planes2;


	std::vector <CPolygonCurve*> m_xoy_planes;
	std::vector <CPolygonCurve*> m_xoz_planes;
	std::vector <CPolygonCurve*> m_yoz_planes;

	boost::dynamic_bitset<> m_xoy_plane_index;
	boost::dynamic_bitset<> m_xoz_plane_index;
	boost::dynamic_bitset<> m_yoz_plane_index;

	boost::shared_ptr<CDynamicArea<pcl::PointXYZRGB> > m_pDyArea2;
	//boost::shared_ptr<CDynamicArea<pcl::PointXYZRGBNormal> > m_pDyArea2;
	boost::shared_ptr<CDynamicArea<pcl::PointXYZRGB> > m_pDyArea3;
	void GeneratePlanes3();
	// 1 generate Dynamic CellMap
	void GenerateDynamicCellMap2();
	
	// for output normals with its pointcloud
	pcl::PointCloud<pcl::Normal>::Ptr m_normals;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPcNormals;

	std::vector<boost::shared_ptr<CPolygonCurve> > m_Planes5;
	// Output Normals and Pc from m_NVP
	void GetNormalsAndPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pPC, pcl::PointCloud<pcl::Normal>::Ptr& pNormal);
	void GeneratePlanes5();
	//* Only PC has value
	void TranslatePC2NVP();
	//	1 VoxelGrid by DynamicCellMap
	//GenerateDynamicCellMap2();
	//	2 Calculate NVs according to neighbor cell points
	void CaculateNVOnCellMap();
	//	3 Flood Flow on this CellMap to find planes 
	void FindPlanesFloodFlowCellMap();
	//  4 Merge Similar planes on m_Plane5
	void MergeSimilarPlanes5();
	//	4 Render (Boundary, NV, GP, Color)
	void FetchPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc);

	// Dynamic Area to Discretize PC
	boost::shared_ptr<CNewDynamicArea<pcl::PointXYZRGBNormal> > m_pNewDyArea;
	void GeneratePlanes6();
	//// 1 VoxelGrid by NewDynamicCellMap
	void GenerateNewDynamicCellMap();
	//2 
	void CaculateNVOnCellMap2();
	

	// threshold for COS(NV1,NV2) COS(5)=0.99619469809174553229501040247389
	float m_dthreshCOS;
	float PointMultiply(pcl::PointXYZRGBNormal&,pcl::PointXYZRGBNormal&);
	float PointMultiply(pcl::PointXYZRGBNormal&,struct CPolygonCurve::Vector_N&);
	float PointMultiply(struct CPolygonCurve::Vector_N& nv1, pcl::Normal& nv_p);
	inline void NormalizeVector(float& x,float& y,float& z){
		if(x>0){x*=-1.f; y*=-1.f; z*=-1.f;}
		float d = x*x + y*y + z*z;
		d = sqrt((double)d);
		x /=d; y/=d; z/=d; 
	}
	// Send plane info to openGL
	void SendPlaneInfo(unsigned char* m_pucImg, float* m_pVertex, int& nCountP);
	void SendPlaneInfoParsePts(unsigned char* m_pucImg, float* m_pVertex, int& nCountP);

	// Dump Session To 3DS
	void DumpTo3DS(string dir);
	CMy3DSWriter m_My3DSWriter;

protected:
private:
};

#endif