#ifndef CNEWDYNAMICAREA_H
#define CNEWDYNAMICAREA_H

#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>
#include <vector>
using std::vector;

//typedef struct{
//	union{
//		struct{
//			float x;
//			float y;
//			float z;
//		};
//		float a[3];
//	};
//	union{
//		struct{
//			float nx;
//			float ny;
//			float nz;
//		};
//		float nv[3];
//	};
//}CellElement; 

template<typename PointT>
class CNewDynamicArea{
public:
	CNewDynamicArea();
	~CNewDynamicArea();

	void initArea();
	void unitArea();
	void reset(float l_x=0.,float u_x=0.,float l_y=0.,float u_y=0.,float l_z=0.,float u_z=0.);

	UINT GetCellIndex(float x, float y, float z);
	void FromPC2Cell(boost::shared_ptr<pcl::PointCloud<PointT> >& pCloud);
	vector< vector<int> > m_pCell;
	boost::shared_ptr<pcl::PointCloud<PointT> > m_pPC;
	boost::dynamic_bitset<> m_valid_cell;

	// Cell Size
	UINT m_CellSize;

	// bounding value along each axis
	float m_lower_x,m_upper_x;
	float m_lower_y,m_upper_y;
	float m_lower_z,m_upper_z;

	// for calculate index
	int m_x_offset,m_y_offset,m_z_offset; // offset to make left-down corner match m_dynamic_cells[0]
	int m_x_step,m_y_step; // step of matched location in m_dynamic_cells
	// m_z_step = 1
	int m_x_range,m_y_range,m_z_range; // range along each axis
	int m_x_cell,m_y_cell,m_z_cell; // num of cells along each axis
	int m_all_cells;
	
};

#include "CNewDynamicArea.hpp"

#endif