#ifndef	CAREAMAP_H
#define	CAREAMAP_H
#pragma once
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
#include "CPose3D.h"

//#define RX 1200 // X [-6 6]
//#define RY 400  // Y [-2 2]
//#define RZ 1200 // Z [-6 6]

#define RX 600 // X [-6 6]
#define RY 400	// Y [-2 2]
#define RZ 600 // Z [-6 6]

#define L_RX RX/200
#define L_RY RY/200
#define L_RZ RZ/200

#define S_RX RX/2
#define S_RY RY/2
#define S_RZ RZ/2

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 300
#define Y_CELL RY/CELLSIZE // 100
#define Z_CELL RZ/CELLSIZE // 300

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL //

// each Area is scattered cells defined in "InternalDefine.h"
class CArea{
public:
	CArea();
	~CArea();

	void Init();	// Initialize CArea
	void UnInit();  // UnInitialize CArea
	// Main 
	int m_offset_x;		// Offset along x-axis
	int m_offset_y;		// Offset along y-axis
	int m_offset_z;		// Offset along z-axis
	
	double m_upper_x,m_lower_x;  // Bounder along x-axis
	double m_upper_y,m_lower_y;	// Bounder along y-axis
	double m_upper_z,m_lower_z;	// Bounder along z-axis
	
	void calcoffset();		// Calculate offset based on current Basic Point
	void calcbounder();		// Calculate bounder box 

	// Index from (x,y,z)
	inline int getIndexCell(float& x,float& y, float& z){
		if(_isnan(x) || _isnan(y) || _isnan(z))
			return -1;
		if(x>=m_upper_x || y>=m_upper_y || z>=m_upper_z \
			|| x<m_lower_x || y<m_lower_y || z<m_lower_z )
			return -1;
		int lx = floor(( x*100 + S_RX)+0.5);
		lx>>=2;//divide by cell_size
		int ly = floor(( y*100 + S_RY)+0.5); 
		ly>>=2;//divide by cell_size
		int lz = floor(( z*100 + S_RZ)+0.5); 
		lz>>=2;//divide by cell_size

		// Calc offset
		lx-=m_offset_x;
		ly-=m_offset_y;
		lz-=m_offset_z;

		if(lx >= X_CELL || ly>= Y_CELL || lz >= Z_CELL \
			|| lx<0 || ly<0 || lz<0)
		{
			return -1;
		}
		return (lx*X_STEP + ly*Y_STEP + lz);
	}
	/*inline int getIndexCell(float& x,float& y, float& z)
	{
		if(!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
			return -1;
		if(fabs(x) >= 6 || fabs(z) >= 6 || y<=-2 || y>=2 )
			return -1;
		int lx = ( x*100 + RX/2) / CELLSIZE;
		int ly = ( y*100 + RY/2) / CELLSIZE;
		int lz = ( z*100 + RZ/2) / CELLSIZE;
		if(lx >= X_CELL || ly>= Y_CELL || lz >= Z_CELL)
		{
			cout<<"error in indexCell!"<<endl;
			return -1;
		}
		return (lx*X_STEP + ly*Y_STEP + lz);
	}*/

	// Functions!
	void TranslateArea(CPose3D& );
	void FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud); // Fill Area with PC
	void FromCell2Triangles(unsigned char *pucImg,float *pfVertex);		// Render Triangles from Area
	void drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex); // Draw Vertex

	// Members!
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_global_cell_map;			// To display cells
	std::vector<boost::shared_ptr<pcl::PointXYZRGB> > m_global_cells;	// All the discrete cells 5*5*5
	bitset<ALL_CELLS> m_valid_flag;										// Index whether cells are still valid
	int m_num_of_ver;													// Number of vertex to render
	CPose3D m_BasicPose;												// Basic Point of CArea

	static unsigned char *m_pucImg;										// Send rgb info
	static float *m_pfVertex;											// Send vertex info
	bool m_IsTriangle;													// Decide whether draw triangles
};
// m_n_CArea
class CAreaMap{

public:
	CAreaMap(int m_n_of_area=1);
	~CAreaMap();

	void Init(); // Initialize m_n_of_area Areas
	void UnInit();

	int m_n_of_area; // number of area
};


#endif