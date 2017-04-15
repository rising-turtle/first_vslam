#include <pcl/io/io.h>
#include <set>
#include <queue>
#include <cmath>
#include "CSession.h"
#include <pcl/features/integral_image_normal.h>
#include <boost/dynamic_bitset.hpp>
#include <pcl/features/normal_3d.h>
#include "pcl/kdtree/kdtree_flann.h"
#include <pcl/filters/voxel_grid.h>
#include "CDynamicArea.h"
//#include "CPolygonCurve.h"
//
#define COS5 0.9961946980917455322950104024738
#define COS3 0.99862
#define COS10 0.98480775301220805936674302458951
#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}
#define LOWER_ASS(Low,V){if(V<Low) Low=V;}

CSession::CSession():m_pc(new pcl::PointCloud<pcl :: PointXYZRGB>()),
m_render(new pcl::PointCloud<pcl::PointXYZ>),
m_rendercolor(new pcl::PointCloud<pcl::PointXYZRGB>),
m_NVP(new pcl::PointCloud<pcl::PointXYZRGBNormal>),
m_pDyArea(new CDynamicArea<pcl::PointXYZRGBNormal>),
m_pDyArea2(new CDynamicArea<pcl::PointXYZRGB>),
m_pDyArea3(new CDynamicArea<pcl::PointXYZRGB>),
//m_pNewDyArea(new CNewDynamicArea<pcl::PointXYZRGBNormal>),
m_normals(new pcl::PointCloud<pcl::Normal>),
m_pPcNormals(new pcl::PointCloud<pcl::PointXYZRGB>),
m_fSearchRadiusKD(0.1), // 10cm search radius on KDT
m_dthreshCOS(COS5) // COS(5)=0.99619469809174553229501040247389
{
#define TMP_MAX 10000
	m_lower_x=TMP_MAX;m_upper_x=-TMP_MAX;
	m_lower_y=TMP_MAX;m_upper_y=-TMP_MAX;
	m_lower_z=TMP_MAX;m_upper_z=-TMP_MAX;
	m_bRenderPlaneReady=false; // not ready to reander planes
}
CSession::~CSession()
{
	for(size_t i=0;i<m_xoy_plane_index.size();i++)
	{
		if(m_xoy_plane_index[i])
		{
			delete m_xoy_planes[i];
			m_xoy_planes[i]=NULL;
			m_xoy_plane_index[i]=0;
		}
	}
	for(size_t i=0;i<m_xoz_plane_index.size();i++)
	{
		if(m_xoz_plane_index[i])
		{
			delete m_xoz_planes[i];
			m_xoz_planes[i]=NULL;
			m_xoz_plane_index[i]=0;
		}
	}
	for(size_t i=0;i<m_yoz_plane_index.size();i++)
	{
		if(m_yoz_plane_index[i])
		{
			delete m_yoz_planes[i];
			m_yoz_planes[i]=NULL;
			m_yoz_plane_index[i]=0;
		}
	}
}

void CSession::TransmitPC(unsigned char *pucImg, float *pfVertex)
{
	for(size_t i=0;i<this->m_pDyArea2->m_dynamic_cell_map->points.size();i++)
	{
		//pcl::PointXYZRGB& pt = this->m_pc->points[i];
		pcl::PointXYZRGB& pt = this->m_pDyArea2->m_dynamic_cell_map->points[i];
		*(pucImg++)=pt.r;
		*(pucImg++)=pt.g;
		*(pucImg++)=pt.b;

		*(pfVertex++)=pt.x;
		*(pfVertex++)=pt.y;
		*(pfVertex++)=pt.z;
	}
}
void CSession::TransmitPC2(unsigned char *pucImg, float *pfVertex)
{
	for(size_t i=0;i<this->m_render->points.size();i++)
	{
		pcl::PointXYZ& pt = this->m_render->points[i];
		*(pucImg++)=127;//pt.r;
		*(pucImg++)=127;//pt.g;
		*(pucImg++)=127;//pt.b;

		*(pfVertex++)=pt.x;
		*(pfVertex++)=pt.y;
		*(pfVertex++)=pt.z;
	}
}
void CSession::TransmitPC3(unsigned char *pucImg, float *pfVertex)
{
	for(size_t i=0;i<this->m_rendercolor->points.size();i++)
	{
		pcl::PointXYZRGB& pt = this->m_rendercolor->points[i];
		*(pucImg++)=pt.r;//pt.r;
		*(pucImg++)=pt.g;//pt.g;
		*(pucImg++)=pt.b;//pt.b;

		*(pfVertex++)=pt.x;
		*(pfVertex++)=pt.y;
		*(pfVertex++)=pt.z;
	}
}
void CSession::SetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pcSession)
{
	m_pc.swap(pcSession);
}

void CSession::SetBoundary(float l_x,float l_y,float l_z,float u_x,float u_y,float u_z)
{
	LOWER_ASS(m_lower_x,l_x);
	LOWER_ASS(m_lower_y,l_y);
	LOWER_ASS(m_lower_z,l_z);

	UPPER_ASS(m_upper_x,u_x);
	UPPER_ASS(m_upper_y,u_y);
	UPPER_ASS(m_upper_z,u_z);
}
/************************************************************************/
/*                      GeneratePlanes4                                               */
/************************************************************************/
void CSession::GeneratePlanes4()
{
	// 1 GenerateDynamicCellMap
	GenerateDynamicCellMap3();
	// 2 FloodFlowAlongAxis
	FloodFlowAlongAxis();
	// 3 DeletePlanesAlongAxis
	// 4 MergeSimilarPlaneAlongAxis
	// 5 Render
	RenderColorAlongAxis();
}
void CSession::FloodFlowAlongAxis()
{
	static double color_similar = 3;
	static int n_of_points = 300;
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGB> > pCellMapPc=m_pDyArea3->m_dynamic_cell_map;
	std::vector<boost::shared_ptr<pcl::PointXYZRGB> >& pCellMap=m_pDyArea3->m_dynamic_cells;

	// whether this cell is valid
	boost::dynamic_bitset<> flag_of_valid;
	flag_of_valid.resize(m_pDyArea3->m_valid_flag.size());
	flag_of_valid=m_pDyArea3->m_valid_flag;

	// whether this point has been into planes
	boost::dynamic_bitset<> flag_of_checked;
	flag_of_checked.resize(m_pDyArea3->m_valid_flag.size(),false);

	int x_step=m_pDyArea3->m_x_step;
	int y_step=m_pDyArea3->m_y_step;

	int x_range=m_pDyArea3->m_x_cell;
	int y_range=m_pDyArea3->m_y_cell;
	int z_range=m_pDyArea3->m_z_cell;

	boost::shared_ptr<pcl::PointXYZRGB> pself;
	// find xoy_planes
	for(int lz=0;lz<z_range;lz++)
		for(int ly=0;ly<y_range;ly++)
			for(int lx=0;lx<x_range;lx++)
			{
				// find point (lx,ly,lz) 
				int index_self = lx*x_step+ly*y_step + lz; 
				if(index_self<0 || index_self>m_pDyArea3->m_all_cells)
					continue;
				if(!flag_of_valid[index_self] || flag_of_checked[index_self])
					continue;
				pself = pCellMap[index_self];

				//std::queue<boost::shared_ptr<pcl::PointXYZRGB> > m_qpts;
				std::queue<int> m_qindex;
				boost::shared_ptr<CPolygonCurve> pCurve(new CPolygonCurve(*pself));
				m_qindex.push(index_self);
				
				while(!m_qindex.empty())
				{
					int cur_index=m_qindex.front();
					if(cur_index<0 || cur_index>m_pDyArea3->m_all_cells)
						continue;
					m_qindex.pop();
					if(flag_of_checked[cur_index])
						continue;
					boost::shared_ptr<pcl::PointXYZRGB> p_cur=pCellMap[cur_index];
					flag_of_checked[cur_index]=1;
					
					// search along plane xoy 9 elements
					for(int nx=-1;nx<=1;nx++)
						for(int ny=-1;ny<=1;ny++)
						{
							int search_index=cur_index+nx*x_step+ny*y_step;
							if(search_index<0 || search_index>m_pDyArea3->m_all_cells)
								continue;
							if(flag_of_valid[search_index] && !flag_of_checked[search_index]) // valid grid and not checked
							{
								boost::shared_ptr<pcl::PointXYZRGB> psearch=pCellMap[search_index];
								if(fabs(p_cur->rgb-psearch->rgb) < color_similar) // similar point
								{
									pCurve->AddNewPTtoPlane(*psearch);
									m_qindex.push(search_index);
								}
							}
						}
				}
				if(pCurve->m_num > n_of_points)
					m_xoy_planes2.push_back(pCurve);
			}
			cout<<"xoy_planes: "<<m_xoy_planes2.size()<<endl;
}
void CSession::RenderColorAlongAxis()
{
	for(size_t i=0;i<m_xoy_planes2.size();i++)
	{
		m_rendercolor->points.insert(m_rendercolor->points.end(),m_xoy_planes2[i]->m_pColorVertex->points.begin(),m_xoy_planes2[i]->m_pColorVertex->points.end());
	}
}

/************************************************************************/
/*						GeneratePlanes6							*/
/************************************************************************/

void CSession::GeneratePlanes6()
{
	//* Only PC has value
	TranslatePC2NVP();
	//	1 VoxelGrid by DynamicCellMap
	GenerateNewDynamicCellMap();
	//  2 
	CaculateNVOnCellMap2();
}
void CSession::CaculateNVOnCellMap2()
{
	int lx,ux,ly,uy,lz,uz;
	int xrge,yrge,zrge;
	xrge=m_pNewDyArea->m_x_cell;
	yrge=m_pNewDyArea->m_y_cell;
	zrge=m_pNewDyArea->m_z_cell;

	int x_step=m_pNewDyArea->m_x_step;
	int y_step=m_pNewDyArea->m_y_step;

	vector<float> nx,ny,nz;
	vector<double> nv;
	nv.resize(3,0);
	CPolygonCurve tmpCurve;
	CPolygonCurve::P3D tmpPoint;
	#define LeastNum 5

	float tx=0;
	float ty=0;
	float tz=0;

	// traverse all cells on CellMap
	for(int px=0;px<xrge;px++)
	for(int py=0;py<yrge;py++)
	for(int pz=0;pz<zrge;pz++)
	{
		int sindex=px*x_step+py*y_step+pz;
		if(!m_pNewDyArea->m_valid_cell[sindex] /*|| m_pNewDyArea->m_pCell[sindex]==NULL*/)
			continue;
		// generate neighboring points along cell map
		// 6*6*6 = 216 neighbor points
		nx.clear();ny.clear();nz.clear();nv.clear();
		vector<int>& pIndex=m_pNewDyArea->m_pCell[sindex];
		if(pIndex.size()<LeastNum) continue;
		
		tx=0; ty=0;tz=0;
		size_t TN=pIndex.size();
		for(size_t i=0;i<TN;i++)
		{
			pcl::PointXYZRGBNormal& sp=m_pNewDyArea->m_pPC->points[pIndex[i]];
			nx.push_back(sp.x);
			ny.push_back(sp.y);
			nz.push_back(sp.z);
			tx+=sp.x; ty+=sp.y; tz+=sp.z;
		}	
			tmpCurve.EstimateNormals(nx,ny,nz,tmpPoint,nv);
		/*	m_pDyArea->m_dynamic_cells[sindex]->normal_x=nv[0];
			m_pDyArea->m_dynamic_cells[sindex]->normal_y=nv[1];
			m_pDyArea->m_dynamic_cells[sindex]->normal_z=nv[2];*/
			pcl::Normal TmpNormal;
			TmpNormal.normal_x=nv[0];
			TmpNormal.normal_y=nv[1];
			TmpNormal.normal_z=nv[2];
			m_normals->points.push_back(TmpNormal);
			pcl::PointXYZRGB TmpPoint;
			TmpPoint.x=tx/(float)(TN);//m_pDyArea->m_dynamic_cells[sindex]->x;
			TmpPoint.y=ty/(float)(TN);//m_pDyArea->m_dynamic_cells[sindex]->y;
			TmpPoint.z=tz/(float)(TN);//m_pDyArea->m_dynamic_cells[sindex]->z;
			m_pPcNormals->points.push_back(TmpPoint);
	}
}
void CSession::GenerateNewDynamicCellMap()
{
	// tmp for debug
	SetBoundary(-5.0,-3.0,-5.0,5.0,3.0,5.0);
	m_pNewDyArea->m_CellSize=2;
	m_pNewDyArea->reset(m_lower_x,m_upper_x,m_lower_y,m_upper_y,m_lower_z,m_upper_z);
	m_pNewDyArea->FromPC2Cell(m_NVP);
}
/************************************************************************/
/*                       GeneratePlanes5                                          */
/************************************************************************/
void CSession::GeneratePlanes5()
{
	//* Only PC has value
	TranslatePC2NVP();
	//	1 VoxelGrid by DynamicCellMap
	GenerateDynamicCellMap();
	//	2 Calculate NVs according to neighbor cell points
	CaculateNVOnCellMap();
	//	3 Flood Flow on this CellMap to find planes 
	FindPlanesFloodFlowCellMap();
	//  4 Merge Similar Planes
	MergeSimilarPlanes5();
	//	4 Render (Boundary, NV, GP, Color)
}
void CSession::TranslatePC2NVP()
{
	m_NVP->points.resize(m_pc->points.size());
	for(size_t i=0;i<m_pc->points.size();i++)
	{
		m_NVP->points[i].x=m_pc->points[i].x;
		m_NVP->points[i].y=m_pc->points[i].y;
		m_NVP->points[i].z=m_pc->points[i].z;
		m_NVP->points[i].r=m_pc->points[i].r;
		m_NVP->points[i].g=m_pc->points[i].g;
		m_NVP->points[i].b=m_pc->points[i].b;
	}
}
// Output Normals and Pc from m_NVP
void CSession::GetNormalsAndPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pPC, pcl::PointCloud<pcl::Normal>::Ptr& pNormal)
{
	size_t N=m_normals->points.size();
	pPC->points.resize(N);
	pNormal->points.resize(N);
	for(size_t i=0;i<N;i++)
	{
		pcl::PointXYZRGB& sp=m_pPcNormals->points[i];
		pPC->points[i].x=sp.x;
		pPC->points[i].y=sp.y;
		pPC->points[i].z=sp.z;
		pPC->points[i].r=sp.r;
		pPC->points[i].g=sp.g;
		pPC->points[i].b=sp.b;
		pcl::Normal& spn=m_normals->points[i];
		pNormal->points[i].normal_x=spn.normal_x;
		pNormal->points[i].normal_y=spn.normal_y;
		pNormal->points[i].normal_z=spn.normal_z;
	}
}
//	2 Calculate NVs according to neighbor cell points
void CSession::CaculateNVOnCellMap()
{
	int lx,ux,ly,uy,lz,uz;
	int xrge,yrge,zrge;
	xrge=m_pDyArea->m_x_cell;
	yrge=m_pDyArea->m_y_cell;
	zrge=m_pDyArea->m_z_cell;

	int x_step=m_pDyArea->m_x_step;
	int y_step=m_pDyArea->m_y_step;

	vector<float> nx,ny,nz;
	vector<double> nv;
	nv.resize(3,0);
	CPolygonCurve tmpCurve;
	CPolygonCurve::P3D tmpPoint;
#define LeastNum 5
	// traverse all cells on CellMap
	for(int px=0;px<xrge;px++)
		for(int py=0;py<yrge;py++)
			for(int pz=0;pz<zrge;pz++)
		{
			int sindex=px*x_step+py*y_step+pz;
			if(!m_pDyArea->m_valid_flag[sindex] || m_pDyArea->m_dynamic_cells[sindex]==NULL)
				continue;
			// generate neighboring points along cell map
			// 6*6*6 = 216 neighbor points
			nx.clear();ny.clear();nz.clear();nv.clear();
			for(lx=-3;lx<=3;lx++)
				for(ly=-3;ly<=3;ly++)
					for(lz=-3;lz<=3;lz++)
					{
						int tmpindex=(px+lx)*x_step+(py+ly)*y_step+pz+lz;
						if(tmpindex<0 || tmpindex>= m_pDyArea->m_all_cells)
							continue;
						if(!m_pDyArea->m_valid_flag[tmpindex] || m_pDyArea->m_dynamic_cells[tmpindex]==NULL)
							continue;
						nx.push_back(m_pDyArea->m_dynamic_cells[tmpindex]->x);
						ny.push_back(m_pDyArea->m_dynamic_cells[tmpindex]->y);
						nz.push_back(m_pDyArea->m_dynamic_cells[tmpindex]->z);
					}
			if(nx.size()>LeastNum)
			{
				tmpCurve.EstimateNormals(nx,ny,nz,tmpPoint,nv);
			}
			else
				nv.assign(3,0);
			m_pDyArea->m_dynamic_cells[sindex]->normal_x=nv[0];
			m_pDyArea->m_dynamic_cells[sindex]->normal_y=nv[1];
			m_pDyArea->m_dynamic_cells[sindex]->normal_z=nv[2];
			pcl::Normal TmpNormal;
			TmpNormal.normal_x=nv[0];
			TmpNormal.normal_y=nv[1];
			TmpNormal.normal_z=nv[2];
			m_normals->points.push_back(TmpNormal);
			pcl::PointXYZRGB TmpPoint;
			TmpPoint.x=m_pDyArea->m_dynamic_cells[sindex]->x;
			TmpPoint.y=m_pDyArea->m_dynamic_cells[sindex]->y;
			TmpPoint.z=m_pDyArea->m_dynamic_cells[sindex]->z;
			m_pPcNormals->points.push_back(TmpPoint);
		}
}
//	3 Flood Flow on this CellMap to find planes 
void CSession::FindPlanesFloodFlowCellMap()
{
#define THRESH_NP_ON_PLANE 20
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> > pCellMapPc=m_pDyArea->m_dynamic_cell_map;
	std::vector<boost::shared_ptr<pcl::PointXYZRGBNormal> >& pCellMap=m_pDyArea->m_dynamic_cells;

	// whether this point has been into planes
	boost::dynamic_bitset<> flag_of_checked;
	flag_of_checked.resize(m_pDyArea->m_valid_flag.size(),false);

	int x_step=m_pDyArea->m_x_step;
	int y_step=m_pDyArea->m_y_step;

	int x_range=m_pDyArea->m_x_cell;
	int y_range=m_pDyArea->m_y_cell;
	int z_range=m_pDyArea->m_z_cell;

	boost::shared_ptr<pcl::PointXYZRGBNormal> pself;
	for(int lx=0;lx<x_range;lx++)
		for(int ly=0;ly<y_range;ly++)
			for(int lz=0;lz<z_range;lz++)
			{
				// find point (lx,ly,lz) 
				int index_self = lx*x_step+ly*y_step + lz; 
				if(index_self<0 || index_self>m_pDyArea->m_all_cells)
					continue;
				if(!m_pDyArea->m_valid_flag[index_self] || flag_of_checked[index_self])
					continue;
				pself = pCellMap[index_self];

				//std::queue<boost::shared_ptr<pcl::PointXYZRGB> > m_qpts;
				std::queue<int> m_qindex;
				boost::shared_ptr<CPolygonCurve> pCurve(new CPolygonCurve(*pself));
				m_qindex.push(index_self);

				while(!m_qindex.empty())
				{
					int cur_index=m_qindex.front();
					if(cur_index<0 || cur_index>m_pDyArea->m_all_cells)
						continue;
					m_qindex.pop();
					if(!m_pDyArea->m_valid_flag[cur_index])
						continue;
					//if(flag_of_checked[cur_index])
						//continue;
					boost::shared_ptr<pcl::PointXYZRGBNormal> p_cur=pCellMap[cur_index];
					flag_of_checked[cur_index]=true;

					// search along plane xoy 9 elements
					for(int nx=-4;nx<=4;nx++)
						for(int ny=-4;ny<=4;ny++)
							for(int nz=-4;nz<=4;nz++)
							{
								int search_index=cur_index+nx*x_step+ny*y_step+nz;
								if(search_index<0 || search_index>m_pDyArea->m_all_cells)
									continue;
								if(m_pDyArea->m_valid_flag[search_index] && !flag_of_checked[search_index]) // valid grid and not checked
								{
									boost::shared_ptr<pcl::PointXYZRGBNormal> psearch=pCellMap[search_index];
									if(fabs(PointMultiply((*psearch),pCurve->NormalV)) >= COS5/*COS5*/) // similar point
									{
										pCurve->AddNewPTtoPlane(*psearch);
										m_qindex.push(search_index);
										flag_of_checked[search_index]=true;
									}
								}
							}
					}
				if(pCurve->m_num >= THRESH_NP_ON_PLANE)
					m_Planes5.push_back(pCurve);
			}
			cout<<"m_Planes5: "<<m_Planes5.size()<<endl;
}
// 4 MergeSimilarPlanes on m_Plane5
void CSession::MergeSimilarPlanes5()
{
	size_t N_Planes=m_Planes5.size();
	boost::dynamic_bitset<> m_VPlanes;
	m_VPlanes.resize(N_Planes,true);
	for(size_t i=0;i<N_Planes;i++)
	{
		if(!m_VPlanes[i]) continue;
		boost::shared_ptr<CPolygonCurve> pFirst=m_Planes5[i];
		for(size_t j=0;j<N_Planes;j++)
		{
			if(j==i) continue;
			if(!m_VPlanes[j]) continue;
			boost::shared_ptr<CPolygonCurve> pSecond=m_Planes5[j];
			if(pFirst->IsSimilarPlane(pSecond.get()))
			{
				pFirst->MergeWithPlane(pSecond.get());
				m_VPlanes[j]=false;
			}
		}
	}
	// Erase those invalid planes
	std::vector<boost::shared_ptr<CPolygonCurve> >::iterator it_plane=m_Planes5.begin();
	for(size_t i=0;i<N_Planes;i++)
	{
		if(!m_VPlanes[i]) it_plane=m_Planes5.erase(it_plane);
		else it_plane++;
	}
	cout<<"After Merge: "<<m_Planes5.size()<<endl;
	return ;
}

// Output points of each plane into PointCloud
void CSession::FetchPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc)
{
	for(size_t i=0;i<m_Planes5.size();i++)
	{
		boost::shared_ptr<CPolygonCurve> pCurPlane=m_Planes5[i];
		m_pc->points.insert(m_pc->points.end(),pCurPlane->m_pColorVertex->points.begin(),pCurPlane->m_pColorVertex->points.end());
	}
}

// Dump to 3ds
void CSession::DumpTo3DS(string dir)
{
	FILE* p3ds=fopen(dir.c_str(),"wb");
	if(p3ds==NULL)
	{
		cout<<"invalid dir: "<<dir<<endl;
		return;
	}
	
	m_My3DSWriter.Reset();
	WORD N_Of_Facets=m_Planes5.size();
	WORD NPoints_Of_Facet;
	UINT TotalLen;
	UINT N_Of_All_Points=0;
	for(WORD i=0;i<N_Of_Facets;i++)
	{
		N_Of_All_Points+=m_Planes5[i]->m_pColorVertex->points.size();
	}
	TotalLen=N_Of_All_Points*3*sizeof(float)+(N_Of_Facets+1)*sizeof(WORD);
	BYTE* pBuf= (BYTE*)new BYTE[TotalLen+1];
	BYTE* pTmpBuf=pBuf;
	memcpy(pTmpBuf,&N_Of_Facets,sizeof(WORD));
	pTmpBuf+=2;
	for(WORD i=0;i<N_Of_Facets;i++)
	{
		NPoints_Of_Facet=m_Planes5[i]->m_pColorVertex->points.size();
		memcpy(pTmpBuf,&NPoints_Of_Facet,sizeof(WORD));
		pTmpBuf+=2;
		for(WORD j=0;j<NPoints_Of_Facet;j++)
		{
			pcl::PointXYZRGB& sp=m_Planes5[i]->m_pColorVertex->points[j];
			memcpy(pTmpBuf,&sp.x,sizeof(float));
			memcpy(pTmpBuf+4,&sp.y,sizeof(float));
			memcpy(pTmpBuf+8,&sp.z,sizeof(float));
			pTmpBuf+=3*sizeof(float);
		}
	}
	m_My3DSWriter.UserPrepareContent((void*)pTmpBuf);
	m_My3DSWriter.Create(dir.c_str());
	delete[] pTmpBuf;

	fclose(p3ds);
}
/************************************************************************/
/*                      GeneratePlanes3                                                */
/************************************************************************/
void CSession::GeneratePlanes3()
{
	// 1 GenerateDynamicCellMap
	//CalculateNV();
	GenerateDynamicCellMap2();
	// 2 
}
void CSession::GenerateDynamicCellMap2()
{
	// BUF for display sessions only be 3MB so, max points should not be 3*1024*1024/15 = 209715
	static int max_points=100000; // just for speed up openGL
	// tmp for debug
	SetBoundary(-10.0,-10.0,-10.0,10.0,10.0,10.0);
	m_pDyArea2->m_s_CellSize=2;
	do 
	{
		m_pDyArea2->m_s_CellSize++;
		m_pDyArea2->reset(m_lower_x,m_upper_x,m_lower_y,m_upper_y,m_lower_z,m_upper_z);
		m_pDyArea2->FromPC2Cell(m_pc);
	} while (m_pDyArea2->m_dynamic_cell_map->points.size()>=max_points);
	
}
void CSession::GenerateDynamicCellMap3()
{
	// tmp for debug
	SetBoundary(-5.0,-3.0,-3.0,5.0,5.0,7.0);
	m_pDyArea3->m_s_CellSize=3;
	m_pDyArea3->reset(m_lower_x,m_upper_x,m_lower_y,m_upper_y,m_lower_z,m_upper_z);
	m_pDyArea3->FromPC2Cell(m_pc);
}
/************************************************************************/
/*                      GeneratePlanes2                                                */
/************************************************************************/
void CSession::GeneratePlanes2()
{
	// 1 Calculate NVs
	CalculateNV();
	// 2 GenerateDynamicCellMap
	GenerateDynamicCellMap();
	// 3 Flood Flow planes on DynamicCellMap
	//FindPlanesAlongAxis();
	FloodFlowOnCellMap();
	// 4 Merge Similar Planes along axis
	//AdjustAllPlanes();
	DeleteInvalidPlanes();
	MergeSimilarLocalPlanes();
	//ObtainBoundaryPts();
	//AdjustBoundaryPts();
	// 
	m_bRenderPlaneReady=true;
	// 6 Render Planes
	RenderPlanes();
	
}
//  1 Discretize pc into Cells
void CSession::GenerateDynamicCellMap()
{
	// tmp for debug
	SetBoundary(-10.0,-10.0,-10.0,10.0,10.0,10.0);
	m_pDyArea->m_s_CellSize=2;
	m_pDyArea->reset(m_lower_x,m_upper_x,m_lower_y,m_upper_y,m_lower_z,m_upper_z);
	m_pDyArea->FromPC2Cell(m_NVP);
}
//	2 Calculate Normal Vectors of each point
//void CSession::CalculateNV2()
//{
//	
//}
//	3 Find planes along XYZ-axis on CellMap
void CSession::FindPlanesAlongAxis()
{
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> > pCellMapPc=m_pDyArea->m_dynamic_cell_map;
	std::vector<boost::shared_ptr<pcl::PointXYZRGBNormal> >& pCellMap=m_pDyArea->m_dynamic_cells;
	boost::dynamic_bitset<> cur_flag;
	cur_flag.resize(m_pDyArea->m_valid_flag.size());
	cur_flag=m_pDyArea->m_valid_flag;

	int x_range=m_pDyArea->m_x_cell;
	int y_range=m_pDyArea->m_y_cell;
	int z_range=m_pDyArea->m_z_cell;
	

	// Initialize NV of xoy plane
	struct CPolygonCurve::Vector_N n_xoy,n_yoz,n_xoz;
	n_xoy.nx=0.1; n_xoy.ny=0.1; n_xoy.nz=0.98994949366116653416118210694679;
	n_xoz.nx=0.1; n_xoz.nz=0.1; n_xoz.ny=0.98994949366116653416118210694679;
	n_yoz.ny=0.1; n_yoz.nz=0.1; n_yoz.nx=0.98994949366116653416118210694679;

	// 1 along xoy-plane
	// Initialize planes along  xoy
	//vector<CPolygonCurve*> xoy_planes;
	m_xoy_planes.resize(z_range,NULL);
	//boost::dynamic_bitset<> plane_index;
	m_xoy_plane_index.reset();
	m_xoy_plane_index.resize(z_range,false);
	
	for(size_t i=0;i<pCellMapPc->points.size();i++)
	{
		pcl::PointXYZRGBNormal& sp= pCellMapPc->points[i];
		float xoy_coincide=fabs(PointMultiply(sp,n_xoy));
		float xoz_coincide=fabs(PointMultiply(sp,n_xoz));
		float yoz_coincide=fabs(PointMultiply(sp,n_yoz));
		if(xoy_coincide>xoz_coincide && xoy_coincide>yoz_coincide && xoy_coincide>=COS5)//COS5)
		{
			int xoy_index=m_pDyArea->getIndexOfAxis(sp,'z');
			if(!m_xoy_plane_index[xoy_index])
			{
				m_xoy_planes[xoy_index]=new CPolygonCurve(sp);
				m_xoy_plane_index[xoy_index]=1;
			}
			else
			{
				m_xoy_planes[xoy_index]->AddNewPTtoPlane(sp);
			}
		}
	}
	
	// 2 along xoz plane
	// 
}
#define COLOR_SIMILAR_THRESHOLD 0.5
void CSession::FloodFlowOnCellMap()
{
	boost::shared_ptr< pcl::PointCloud<pcl::PointXYZRGBNormal> > pCellMapPc=m_pDyArea->m_dynamic_cell_map;
	std::vector<boost::shared_ptr<pcl::PointXYZRGBNormal> >& pCellMap=m_pDyArea->m_dynamic_cells;

	// whether this cell is valid
	boost::dynamic_bitset<> flag_of_valid;
	flag_of_valid.resize(m_pDyArea->m_valid_flag.size());
	flag_of_valid=m_pDyArea->m_valid_flag;

	// whether this point has been into planes
	boost::dynamic_bitset<> flag_of_checked;
	flag_of_checked.resize(m_pDyArea->m_valid_flag.size(),false);
	
	int x_step=m_pDyArea->m_x_step;
	int y_step=m_pDyArea->m_y_step;

	std::queue<boost::shared_ptr<pcl::PointXYZRGBNormal> > m_qpts;
	for(size_t i=0;i<pCellMapPc->points.size();i++)
	{
		pcl::PointXYZRGBNormal& sp=pCellMapPc->points[i];
		int self_index=m_pDyArea->getIndexCell(sp.x,sp.y,sp.z);
		
		if(!flag_of_valid[self_index] || flag_of_checked[self_index])
			continue;
		boost::shared_ptr<CPolygonCurve> pCurve(new CPolygonCurve(sp));
		boost::shared_ptr<pcl::PointXYZRGBNormal> p_pt(new pcl::PointXYZRGBNormal(sp));
		m_qpts.push(p_pt);
		m_local_planes.push_back(pCurve);

		while(!m_qpts.empty())
		{
			boost::shared_ptr<pcl::PointXYZRGBNormal> p_sp=m_qpts.front();
			m_qpts.pop();
			int self_index=m_pDyArea->getIndexCell(p_sp->x,p_sp->y,p_sp->z);
			if(flag_of_checked[self_index])
				continue;
			flag_of_checked[self_index]=1;
			// traverse all neighbor points 
			for(int lx=-2;lx<=2;lx++)
				for(int ly=-2;ly<=2;ly++)
					for(int lz=-2;lz<=2;lz++)
					{
						int cur_index=self_index+lx*x_step+ly*y_step+lz;
						if(cur_index>0 && cur_index<m_pDyArea->m_all_cells) // valid range [0,m_all_cells]
						{
							if(flag_of_valid[cur_index] && !flag_of_checked[cur_index]) // this is new point should be added into plane
							{
								boost::shared_ptr<pcl::PointXYZRGBNormal> p_sp1=pCellMap[cur_index];
								if(fabs(p_sp->rgb-p_sp1->rgb)<COLOR_SIMILAR_THRESHOLD && fabs(PointMultiply(*p_sp1,pCurve->NormalV))>= COS10) // NV similar
								{
									pCurve->AddNewPTtoPlane(*p_sp1);
									m_qpts.push(p_sp1);
								}
								else if(fabs(PointMultiply(*p_sp1,pCurve->NormalV))>= COS5)
								{
									pCurve->AddNewPTtoPlane(*p_sp1);
									m_qpts.push(p_sp1);
								}
							}
						}
					}
			
		}
	}
	std::cout<<"N of local_planes: "<<m_local_planes.size()<<std::endl;
}
//	4 Merge Similar Planes 
void CSession::MergeSimilarLocalPlanes()
{
	boost::dynamic_bitset<> flag_of_checked;
	flag_of_checked.resize(m_local_planes.size(),false);
	for(size_t i=0;i<m_local_planes.size();i++)
	{
		if(flag_of_checked[i] || !m_valid_planes[i])
			continue;
		for(size_t j=i+1;j<m_local_planes.size();j++)
		{
			if(flag_of_checked[j] || !m_valid_planes[j])
				continue;
			if(m_local_planes[i]->IsSimilarPlane(m_local_planes[j].get()))
			{
				m_local_planes[i]->MergeWithPlane(m_local_planes[j].get());
				cout<<"Plane j:"<<j<<" merged with plane i:" <<i<<endl;
				flag_of_checked[j]=1;
				m_valid_planes[j]=0;
			}
		}
	}
}
void CSession::MergeSimilarPlanes()
{
	// 1 merge xoy-planes
	for(size_t i=0;i<m_xoy_plane_index.size();i++)
	{
		if(m_xoy_plane_index[i])
		{
			int j=i+1;
			while(j<i+5) // at most find 5 cells
			{
				if(m_xoy_plane_index[j] && m_xoy_planes[i]->IsSimilarPlane(m_xoy_planes[j]))
				{
					m_xoz_planes[i]->MergeWithPlane(m_xoy_planes[j]);
					delete m_xoy_planes[j];
					m_xoy_planes[j]=NULL;
					m_xoy_plane_index[j]=0;
				}
				else if(m_xoy_plane_index[j])
					break;
				j++;
			}
		}
	}
}
//	5 Offer Render method(Boundary, NV, GP, Color)
//void RenderPlanes()

/************************************************************************/
/*                      GeneratePlanes                                  */
/************************************************************************/
void CSession::GeneratePlanes()
{
	// 1. Calculate Normal Vectors of each point
	CalculateNV();
	// 2. Flood-Flow expand according to KD-tree
	CalculatePlanes();
	// 3. Merge Planes which is overlapped along boundary
	AdjustAllPlanes(); // firstly, Adjust these planes
	MergeOverlappedPlanes();
	// 4. Ready to render
	m_bRenderPlaneReady=true;
}
//  1 Calculate Normal Vectors of each point
void CSession::CalculateNV()
{
	// Downsampling pc
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	std::cout<<"Before VoxelGrid m_pc:"<<m_pc->points.size()<<std::endl;
	sor.setInputCloud (m_pc);
	sor.setLeafSize (0.04f, 0.04f, 0.04f);
	sor.filter (*pc_filtered);
	m_pc->points.swap(pc_filtered->points);
	std::cout<<"After VoxelGrid m_pc:"<<m_pc->points.size()<<std::endl;
	// Normal Estimation Using Integral Images
	/*pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	m_pc->height=1;
	m_pc->width=m_pc->points.size();
	//ne.setInputCloud (m_pc);
	ne.setInputCloud(cloud);
	ne.compute (*cloud_normals);
*/

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (m_pc);

	// Set KDT to search
	pcl::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.1);

	// Compute the features
	ne.compute (*cloud_normals);

	// Concatenate points and normals
	pcl::concatenateFields(*m_pc,*cloud_normals,*m_NVP);
}
bool CSession::IsSamePlane(pcl::PointXYZRGBNormal& pnv1,pcl::PointXYZRGBNormal& pnv2)
{
	NormalizeVector(pnv1.normal_x,pnv1.normal_y,pnv1.normal_z);
	NormalizeVector(pnv2.normal_x,pnv2.normal_y,pnv2.normal_z);
	
	if(fabs(PointMultiply(pnv1,pnv2))>=m_dthreshCOS) // angle <= 5 degree
	{
		return true;
	}
	return false;
}
float CSession::PointMultiply(pcl::PointXYZRGBNormal& nv1,pcl::PointXYZRGBNormal& nv2)
{
	return (nv1.normal_x*nv2.normal_x+nv1.normal_y*nv2.normal_y+nv1.normal_z*nv2.normal_z);
}
float CSession::PointMultiply(pcl::PointXYZRGBNormal& nv1,struct CPolygonCurve::Vector_N& nv2)
{
	return (nv1.normal_x*nv2.nx+nv1.normal_y*nv2.ny+nv1.normal_z*nv2.nz);
}
float CSession::PointMultiply(struct CPolygonCurve::Vector_N& nv1, pcl::Normal& nv_p)
{
	return (nv1.nx*nv_p.normal_x+nv1.ny*nv_p.normal_y+nv1.nz*nv_p.normal_z);
}
//	2 Flood-Flow expand according to KD-tree
void CSession::CalculatePlanes()
{
	// build KDT to search neighbors for each point
	pcl::KdTree<pcl::PointXYZRGBNormal>::Ptr p_KDT(new pcl::KdTreeFLANN<pcl::PointXYZRGBNormal>);
	p_KDT->setInputCloud(m_NVP);

	// bitset to demonstrate whether this point has been calculated!
	boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(m_NVP->points.size(),false);
	
	for(int i=0;i<m_NVP->points.size();i++)
	{
		// Neighbors within radius search
		pcl::PointXYZRGBNormal& searchpoint=m_NVP->points[i];
		if(!cur_frame[i]) // this is a new plane
		{
			cur_frame[i]=1;
			PPtr pPlane(new CPolygonCurve(searchpoint));
			m_plans.insert(pPlane);

			std::vector<int> pointIdxRadiusSearch;
			std::vector<float> pointRadiusSquaredDistance;
			std::queue<boost::shared_ptr<pcl::PointXYZRGBNormal> > q_pts;
			boost::shared_ptr<pcl::PointXYZRGBNormal> pPts(new pcl::PointXYZRGBNormal(searchpoint));
			q_pts.push(pPts);
			while(!q_pts.empty())
			{
				boost::shared_ptr<pcl::PointXYZRGBNormal> p_searchpoint=q_pts.front();
				q_pts.pop();

				if(p_KDT->radiusSearch(*p_searchpoint,m_fSearchRadiusKD,pointIdxRadiusSearch,pointRadiusSquaredDistance))
				{
					for(size_t i=0;i<pointIdxRadiusSearch.size();i++)
					{
						if(cur_frame[pointIdxRadiusSearch[i]])	// this point has already been set
							continue;
						pcl::PointXYZRGBNormal& pNeighPt=m_NVP->points[pointIdxRadiusSearch[i]];
						if(!IsSamePlane(*p_searchpoint,pNeighPt))
							continue;
						cur_frame[pointIdxRadiusSearch[i]]=1;			// this point has been set
						pPlane->AddNewPTtoPlane(pNeighPt);				// add this point into plane
						boost::shared_ptr<pcl::PointXYZRGBNormal> pNewPt(new pcl::PointXYZRGBNormal(pNeighPt));
						q_pts.push(pNewPt);							// add this point into queue to flood 
					}
				}
			}
		}
	}
	std::cout<<"N of planes: "<<m_plans.size()<<std::endl;
}

void CSession::AdjustAllPlanes()
{
	//std::set<PPtr>::iterator it_plane=m_local_planes.begin();
	std::vector<boost::shared_ptr<CPolygonCurve> >::iterator it_plane=m_local_planes.begin();
	while(it_plane!=m_local_planes.end())
	{
		(*it_plane)->NormalizeALL();
		it_plane++;
	}
}
#define PLANE_THRESHOLD 500
#define AXIS_PLANE_THRESHOLD 50
void CSession::DeleteInvalidPlanes()
{
	if(m_valid_planes.size()!=m_local_planes.size())
	{
		m_valid_planes.resize(m_local_planes.size());
		m_valid_planes.set();
	}
	std::vector<boost::shared_ptr<CPolygonCurve> >::iterator it_plane=m_local_planes.begin();
	size_t index=0;
	size_t nCout=0;

	// Initialize NV of xoy plane
	struct CPolygonCurve::Vector_N n_xoy,n_yoz,n_xoz;
	n_xoy.nx=0.1; n_xoy.ny=0.1; n_xoy.nz=0.98994949366116653416118210694679;
	n_xoz.nx=0.1; n_xoz.nz=0.1; n_xoz.ny=0.98994949366116653416118210694679;
	n_yoz.ny=0.1; n_yoz.nz=0.1; n_yoz.nx=0.98994949366116653416118210694679;

	while(it_plane!=m_local_planes.end())
	{
		if((*it_plane)->PointMultiply((*it_plane)->NormalV,n_xoy)>=COS10 || \
			(*it_plane)->PointMultiply((*it_plane)->NormalV,n_xoz)>=COS10 || \
			(*it_plane)->PointMultiply((*it_plane)->NormalV,n_yoz)>=COS10) // If this is regular plane
		{
			if((*it_plane)->m_num<AXIS_PLANE_THRESHOLD)
			{
				m_valid_planes[index]=0;	
				nCout++;
			}
		}
		else if((*it_plane)->m_num<PLANE_THRESHOLD)
		{
			m_valid_planes[index]=0;	
			nCout++;
		}
		index++;
		it_plane++;
	}
	cout<<nCout<<" of planes to delete! "<<endl;
}
void CSession::ObtainBoundaryPts()
{
	std::vector<boost::shared_ptr<CPolygonCurve> >::iterator it_plane=m_local_planes.begin();
	while(it_plane!=m_local_planes.end())
	{
		(*it_plane)->ObtainBoundaryPoints();
		it_plane++;
	}
}
void CSession::AdjustBoundaryPts()
{
	std::vector<boost::shared_ptr<CPolygonCurve> >::iterator it_plane=m_local_planes.begin();
	while(it_plane!=m_local_planes.end())
	{
		(*it_plane)->AdjustBoundaryPoints();
		it_plane++;
	}
}
//	3 Merge Planes which is overlapped along boundary
void CSession::MergeOverlappedPlanes()
{
	std::set<PPtr>::iterator it_first_plane=m_plans.begin();
	std::set<PPtr>::iterator it_secnd_plane=m_plans.begin();

	it_secnd_plane++;
	while(it_first_plane!=m_plans.end())
	{
		//it_secnd_plane++;
		if(it_secnd_plane==m_plans.end())
		{
			it_first_plane++;
			it_secnd_plane=it_first_plane;
			it_secnd_plane++;
			continue;
		}
		if((*it_first_plane)->IsSimilarPlane((*it_secnd_plane).get()))
		{
			if((*it_first_plane)->IsOverLapped((*it_secnd_plane).get()))
			{
				(*it_first_plane)->MergeWithPlane((*it_secnd_plane).get());
				it_secnd_plane=m_plans.erase(it_secnd_plane);
				continue;
			}
			else
			{
				it_secnd_plane++;
				continue;
			}
		}
		it_first_plane++;
		it_secnd_plane=it_first_plane;
		it_secnd_plane++;
	}
}
//	4 Offer Render method(Boundary, NV, GP, Color)
void CSession::RenderPlanes()
{
	if(!m_bRenderPlaneReady)
	{
		std::cout<<"not ready to render planes!!!"<<std::endl;
		return;
	}
	for(size_t i=0;i<m_local_planes.size();i++)
	{
		if(!m_valid_planes[i])
			continue;
		m_render->points.insert(m_render->points.end(),m_local_planes[i]->m_pVertex->points.begin(),m_local_planes[i]->m_pVertex->points.end());
	}
	std::cout<<"N of points to Render!!!"<<m_render->points.size()<<std::endl;
}

void CSession::SendPlaneInfo(unsigned char* m_pucImg, float* m_pVertex, int& nCountP)
{
	//nCountP=m_local_planes.size();
	nCountP=m_valid_planes.count();
	unsigned char* ptmpImg=m_pucImg;
	float* ptmpVer=m_pVertex;
	for(size_t i=0;i<m_local_planes.size();i++)
	{
		if(!m_valid_planes[i]) // Invalid planes
			continue;
		
		boost::shared_ptr<CPolygonCurve> pCurve=m_local_planes[i];
		cout<<"this plane has "<<pCurve->m_num<<" points!!"<<endl;
		float N_of_V=(float)(pCurve->m_pVertex->points.size());
		memcpy(ptmpVer,&N_of_V,sizeof(float));
		ptmpVer+=1;
		(*ptmpImg++)=(unsigned char)(pCurve->GravityP.R);
		(*ptmpImg++)=(unsigned char)(pCurve->GravityP.G);
		(*ptmpImg++)=(unsigned char)(pCurve->GravityP.B);
		for(size_t j=0;j<pCurve->m_pVertex->points.size();j++)
		{
			pcl::PointXYZ& sp=pCurve->m_pVertex->points[j];
			memcpy(ptmpVer,&sp.x,4);
			memcpy(ptmpVer+1,&sp.y,4);
			memcpy(ptmpVer+2,&sp.z,4);
			ptmpVer+=3;
		}
	}
}
void CSession::SendPlaneInfoParsePts(unsigned char* m_pucImg, float* m_pVertex, int& nCountP)
{
	//nCountP=m_local_planes.size();
	int nCout=0;
	//nCountP=m_pDyArea2->m_dynamic_cell_map->points.size();
	unsigned char* ptmpImg=m_pucImg;
	float* ptmpVer=m_pVertex;
	for(size_t i=0;i<m_pDyArea2->m_dynamic_cell_map->points.size();i++)
	{
		pcl::PointXYZRGB& sp=m_pDyArea2->m_dynamic_cell_map->points[i];
		if(sp.y< -1.2 || sp.y>1.2)
			continue;
		nCout++;
		(*ptmpImg++)=sp.r;
		(*ptmpImg++)=sp.g;
		(*ptmpImg++)=sp.b;
		(*ptmpVer++)=sp.x;
		(*ptmpVer++)=sp.y;
		(*ptmpVer++)=sp.z;
	}
	nCountP=nCout;
}
