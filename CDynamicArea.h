#ifndef CDYNAMICAREA_H
#define CDYNAMICAREA_H

#pragma once
#include <boost/shared_ptr.hpp>
#include <boost/dynamic_bitset.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <vector>
//#include "InternalDefine.h"
#include "CPose3D.h"


template <typename PointT>
class CDynamicArea{
public:
	CDynamicArea();
	CDynamicArea(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z);
	~CDynamicArea();

	void initArea();
	void unitArea();
	void reset(float l_x=0.,float u_x=0.,float l_y=0.,float u_y=0.,float l_z=0.,float u_z=0.);
	int getIndexCell(float& x,float& y, float& z);	// Index from (x,y,z)
	int getIndexOfAxis(PointT&,char );
	void FromPC2Cell(boost::shared_ptr<pcl::PointCloud<PointT> >& cloud); //FromPC2Cell

	int m_num_of_ver;
	void drawVertex(boost::shared_ptr<PointT> p[3],unsigned char *pucImg,float *pfVertex);
	void FromCell2Triangles(unsigned char *pucImg,float *pfVertex);

	// indicator whether to use TopView for path-planing
	bool m_bNeedTopView;
	// generate top-view for path-planing
	void initTopView();
	void generateTopView();
	void unitTopView();
	bool m_bIsTopViewAvailable;
	enum Grid_S{UnKnown,Floor,Block,Sink};
	std::vector<std::vector<Grid_S> > m_vGrids;
	std::vector<std::vector<int> > m_vGridsIndicator;
	float m_upper_y_topview;
	float m_lower_y_topview;
	int m_upper_y_topview_cell;
	int m_lower_y_topview_cell;
	std::vector<boost::shared_ptr<PointT> > m_topview_cells;

	// cell_size to dynamically display map at different scale
	int m_s_CellSize;

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

	boost::dynamic_bitset<> m_valid_flag;
	boost::shared_ptr< pcl::PointCloud<PointT> > m_dynamic_cell_map; // to display cells
	std::vector<boost::shared_ptr<PointT> > m_dynamic_cells; // all the discrete cells 4*4*4
	
};

#include "CDynamicArea.hpp"

#endif