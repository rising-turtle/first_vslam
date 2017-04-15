//#include "CDynamicArea.h"

#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}
#define AMEM 1024*1024*300

template <typename PointT>
CDynamicArea<PointT>::CDynamicArea():m_dynamic_cell_map(new pcl::PointCloud<PointT>)
{
	m_lower_x=0;
	m_lower_y=0;
	m_lower_z=0;
	m_upper_x=0;
	m_upper_y=0;
	m_upper_z=0;

	// default cell_size=2 power2 
	m_s_CellSize=2;

	m_upper_y_topview=0.4;
	m_lower_y_topview=-0.4;
	m_upper_y_topview_cell=2;
	m_lower_y_topview_cell=0;
	m_bNeedTopView=false;
	//m_num_of_ver=0;
	initArea();
}
template <typename PointT>
CDynamicArea<PointT>::CDynamicArea(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z):m_dynamic_cell_map(new pcl::PointCloud<PointT>),\
							m_lower_x(l_x),m_upper_x(u_x),m_lower_y(l_y),m_upper_y(u_y),m_lower_z(l_z),m_upper_z(u_z)
{
	initArea();
}
template <typename PointT>
CDynamicArea<PointT>::~CDynamicArea(){}

template <typename PointT>
void CDynamicArea<PointT>::initArea()
{
	m_x_offset=(int)(m_lower_x*100-1);
	m_y_offset=(int)(m_lower_y*100-1);
	m_z_offset=(int)(m_lower_z*100-1);

	m_x_range=(int)((m_upper_x-m_lower_x)*100);
	m_y_range=(int)((m_upper_y-m_lower_y)*100);
	m_z_range=(int)((m_upper_z-m_lower_z)*100);

	m_x_cell=m_x_range>>m_s_CellSize; // cell_size = 2p2=4
	m_y_cell=m_y_range>>m_s_CellSize;
	m_z_cell=m_z_range>>m_s_CellSize;

	m_x_step=m_y_cell*m_z_cell;
	m_y_step=m_z_cell;
	m_all_cells=m_x_cell*m_y_cell*m_z_cell;
	if(m_all_cells!=0)
		cout<<"!!!!!m_all_cells: "<<m_all_cells<<endl;
	if(m_all_cells<0) 
		m_all_cells=0;
	//cout<<"All_Cells: "<<(m_all_cells*4)/1024<<"KB"<<endl;
	m_dynamic_cells.swap(std::vector<boost::shared_ptr<PointT> >());
	m_dynamic_cells.resize(m_all_cells);
	while( m_all_cells*4 >= AMEM || (m_all_cells>0 && m_dynamic_cells.size()==0))
	{
		m_s_CellSize++;
		//cout<<"Cell_Size: "<<m_s_CellSize<<endl;
		m_x_cell=m_x_range>>m_s_CellSize; // cell_size = 2p2=4
		m_y_cell=m_y_range>>m_s_CellSize;
		m_z_cell=m_z_range>>m_s_CellSize;

		m_x_step=m_y_cell*m_z_cell;
		m_y_step=m_z_cell;
		m_all_cells=m_x_cell*m_y_cell*m_z_cell;
		cout<<"All_Cells: "<<((m_all_cells*4)/1024)/1024<<"MB"<<endl;
		m_dynamic_cells.resize(m_all_cells);
	}

	/*while( m_all_cells*4 >= AMEM)
	{
		m_s_CellSize++;
		cout<<"Cell_Size: "<<m_s_CellSize<<endl;
		m_x_cell=m_x_range>>m_s_CellSize; // cell_size = 2p2=4
		m_y_cell=m_y_range>>m_s_CellSize;
		m_z_cell=m_z_range>>m_s_CellSize;

		m_x_step=m_y_cell*m_z_cell;
		m_y_step=m_z_cell;
		m_all_cells=m_x_cell*m_y_cell*m_z_cell;
		cout<<"All_Cells: "<<(m_all_cells*4)/1024<<"KB"<<endl;
	}*/

	//m_dynamic_cells.resize(m_all_cells);
	m_valid_flag.resize(0);
	m_valid_flag.resize(m_all_cells,false);
	m_valid_flag.reset();

	m_num_of_ver=0;
	//cout<<"dynamic_cells.size="<<m_all_cells<<endl;
	//cout<<"m_x_cell:"<<m_x_cell<<endl;
	//cout<<"m_y_cell:"<<m_y_cell<<endl;
	//cout<<"m_z_cell:"<<m_z_cell<<endl;
	//cout<<"m_x_offset:"<<m_x_offset<<endl;
	//cout<<"m_y_offset:"<<m_y_offset<<endl;
	//cout<<"m_z<<offset:"<<m_z_offset<<endl;


	//cout<<"leave CDynamicArea Init()"<<endl;
	if(m_bNeedTopView)
	{
		initTopView();
	}
}
template <typename PointT>
void CDynamicArea<PointT>::unitArea()
{	
	m_dynamic_cells.swap(std::vector<boost::shared_ptr<PointT> >());
	m_dynamic_cell_map->points.clear();
	m_valid_flag.resize(0,false);
}

template <typename PointT>
void CDynamicArea<PointT>::reset(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z)
{
	unitArea();
	static float error_tor=0.08;
	m_lower_x=l_x;//-error_tor;
	m_lower_y=l_y;//-error_tor;
	m_lower_z=l_z;//-error_tor;
	m_upper_x=u_x+error_tor;
	m_upper_y=u_y+error_tor;
	m_upper_z=u_z+error_tor;
	initArea();
}
template <typename PointT>
int CDynamicArea<PointT>::getIndexOfAxis(PointT& pt,char axis)
{
	float fx=pt.x; float fy=pt.y; float fz=pt.z;
	int ret,lx,ly,lz;
	switch(axis)
	{
	case 'x':
		lx = floor(( fx*100 - m_x_offset)+0.5);
		lx>>=m_s_CellSize;
		if(lx >= m_x_cell || lx<0)
		{
			cout<<"boundary exceed at x="<<fx<<endl;
			return -1;
		}
		ret=lx;
		break;
	case 'y':
		ly = floor(( fy*100 - m_y_offset)+0.5);
		ly>>=m_s_CellSize;
		if(ly >= m_y_cell || ly<0)
		{
			cout<<"boundary exceed at y="<<fy<<endl;
			return -1;
		}
		ret=ly;
		break;
	case 'z':
		lz = floor(( fz*100 - m_z_offset)+0.5);
		lz>>=m_s_CellSize;
		if(lz >= m_z_cell || lz<0)
		{
			cout<<"boundary exceed at z="<<fz<<endl;
			return -1;
		}
		ret=lz;
		break;
	default:
		ret=-1;
		break;
	}
	return ret;
}
template <typename PointT>
int CDynamicArea<PointT>::getIndexCell(float& x,float& y, float& z)
{
	if(_isnan(x) || _isnan(y) || _isnan(z))
		return -1;
	if(x>=m_upper_x || y>=m_upper_y || z>=m_upper_z \
		|| x<m_lower_x || y<m_lower_y || z<m_lower_z )
		return -1;
	int lx = floor(( x*100 - m_x_offset)+0.5);
	lx>>=m_s_CellSize;//divide by cell_size
	int ly = floor(( y*100 - m_y_offset)+0.5); 
	ly>>=m_s_CellSize;//divide by cell_size
	int lz = floor(( z*100 - m_z_offset)+0.5); 
	lz>>=m_s_CellSize;//divide by cell_size

	if(lx >= m_x_cell || ly>= m_y_cell || lz >= m_z_cell \
		|| lx<0 || ly<0 || lz<0)
	{
		cout<<"exceed range in CDynamic::getIndex() at "<<"("<<x<<","<<y<<","<<z<<")"<<endl;
		if(lx>=m_x_cell)
			cout<<"lx="<<lx<<",x_cell="<<m_x_cell<<endl;
		if(ly>=m_y_cell)
			cout<<"ly="<<ly<<",y_cell="<<m_y_cell<<endl;
		if(lz>=m_z_cell)
			cout<<"lz="<<lz<<",z_cell="<<m_z_cell<<endl;
		return -1;
	}

	if(m_bNeedTopView)
	{
		if(ly>=m_lower_y_topview_cell && ly<m_upper_y_topview_cell)
			UPPER_ASS(m_vGridsIndicator[lx][lz],ly);
	}

	return (lx*m_x_step + ly*m_y_step + lz);
}
template <typename PointT>
void CDynamicArea<PointT>::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<PointT> >& cloud) //FromPC2Cell
{
	int N = cloud->points.size();
	static boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(m_all_cells,false);
	cur_frame.reset();
	//int num=0;
	//int nDelNum1=0,nDelNum2=0,nDelNum3=0;

	//cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		PointT& sp = cloud->points[i];
		int index = this->getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(this->m_valid_flag[index])
			{/*pc3Dmap->m_global_cells[index].get()!=NULL*/ // this is already painted
				//nDelNum2++;
				continue;
			}// not flush this point using the new point	
			boost::shared_ptr<PointT> p(new PointT(sp));
			/*p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;*/
			if(!cur_frame[index])
			{
				//num++;
				cur_frame.set(index);
				this->m_dynamic_cells[index] = p;
				this->m_dynamic_cell_map->points.push_back(sp);
			}
			
		}
		
	}
	//cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	this->m_valid_flag |= cur_frame; // 
	/*cout<<"num of inserted="<<num<<endl;
	cout<<"del1="<<nDelNum1<<"; del2="<<nDelNum2<<"; del3="<<nDelNum3<<endl;
	cout<<"m_dynamic_cell_map.size="<<this->m_dynamic_cell_map->points.size();*/
}
template <typename PointT>
void CDynamicArea<PointT>::initTopView()
{
	cout<<"initTopView!"<<endl;
	m_vGridsIndicator.resize(m_x_cell);
	for(int i=0;i<m_x_cell;i++)
		m_vGridsIndicator[i].resize(m_z_cell,-1);
	int m_topview_y_cell=(int)((m_upper_y_topview-m_lower_y_topview)*100);// actually it is 20 for [-0.4,0.4]
	m_topview_cells.resize(m_x_cell*m_z_cell*20);

	// find index along y-axis according to [-0.4,0.4]
	m_upper_y_topview_cell=(int)((m_upper_y_topview-m_lower_y)*100);
	m_lower_y_topview_cell=(int)((m_lower_y_topview-m_lower_y)*100);
	if(m_upper_y_topview_cell<=m_lower_y_topview_cell)
		cout<<"error at topview y-range:"<<m_lower_y_topview_cell<<"-"<<m_upper_y_topview_cell<<endl;
	while(m_upper_y_topview_cell-m_lower_y_topview_cell>20)
		m_upper_y_topview_cell--;
	
	m_bIsTopViewAvailable=false;
}
template <typename PointT>
void CDynamicArea<PointT>::generateTopView()
{
	int lx,ly,lz;
	
	m_vGrids.resize(m_x_cell);
	for(int i=0;i<m_x_cell;i++)
		m_vGrids[i].resize(m_z_cell);

	for(lx=0;lx<m_x_cell;lx++)	
		for(lz=0;lz<m_z_cell;lz++)
		{
			ly=m_vGridsIndicator[lx][lz];
			if(ly==-1)
			{
				m_vGrids[lx][lz]=Grid_S::UnKnown;
			}
			else if(ly<m_lower_y_topview_cell)
			{
				m_vGrids[lx][lz]=Grid_S::Sink;
			}
			else if(ly>=m_lower_y_topview_cell && ly<= m_lower_y_topview_cell+2)//m_upper_y_topview_cell)
			{
				m_vGrids[lx][lz]=Grid_S::Floor;
			}
			else
				m_vGrids[lx][lz]=Grid_S::Block;
		}
		m_bIsTopViewAvailable=true;
}
template <typename PointT>
void CDynamicArea<PointT>::unitTopView()
{
	for(int i=0;i<m_x_cell;i++)
	{
		m_vGrids[i].clear();
		m_vGridsIndicator[i].clear();
	}
	m_vGrids.clear();
	m_vGridsIndicator.clear();
	m_bIsTopViewAvailable=false;
}
template <typename PointT>
void  CDynamicArea<PointT>::drawVertex(boost::shared_ptr<PointT> p[3],unsigned char *pucImg,float *pfVertex){
	for(size_t i=0;i<3;i++)
	{
		*(pucImg++) = p[i]->r;
		*(pucImg++) = p[i]->g;
		*(pucImg++) = p[i]->b;
		*(pfVertex++) = p[i]->x;
		*(pfVertex++) = p[i]->y;
		*(pfVertex++) = p[i]->z;
	}
	m_num_of_ver+=3;
	//glColor3ub(p->r,p->g,p->b);
	//glVertex3f(p->x,p->y,p->z);
}

template <typename PointT>
void  CDynamicArea<PointT>::FromCell2Triangles(unsigned char *pucImg,float *pfVertex)
{
	static double color_similar = 3;
	bool d_left,d_behind,d_down;
	boost::shared_ptr<PointT> pself;
	boost::shared_ptr<PointT> pP[3];
	int d_n = 0;
	unsigned char *pImg = pucImg;
	float * pVertex = pfVertex;

	for(int lx=0;lx<m_x_cell;lx++) // traverse along x-axis
		for(int ly=0;ly<m_y_cell;ly++) // traverse along y-axis
			for(int lz=0;lz<m_z_cell;lz++) // traverse along z-axis
			{
				// initilization 

				// find point (lx,ly,lz) 
				int index_self = lx*m_x_step+ly*m_y_step + lz; 
				pself = m_dynamic_cells[index_self];
				pP[0] = pself;//global_cells[index_self];
				if(!m_valid_flag[index_self] || pself.get()==NULL){
					continue;
				}

				// on xoy plane
				//glBegin(GL_TRIANGLE_STRIP);
				//drawVertex(pself,pImg,pVertex);

				bool draw_xoy = true;
				int ll = lx-1; // left 
				int ld = ly-1; // down 
				int lb = lz-1; // before

				int left_p = ll*m_x_step + ly*m_y_step + lz;
				int before_p = lx*m_x_step + ly*m_y_step + lb;
				int down_p = lx*m_x_step + ld*m_y_step + lz;

				// down point on xoy
				if( ll>=0 && ld >=0)
				{
					pP[1] = m_dynamic_cells[ll*m_x_step + ld*m_y_step + lz]; // down-left point
					pP[2] = m_dynamic_cells[down_p]; // down point
				}
				else
					draw_xoy = false;
				if(draw_xoy && m_valid_flag[ll*m_x_step + ld*m_y_step + lz] && m_valid_flag[down_p] && pP[1].get() != NULL && pP[2].get() != NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoy = false;
				if(draw_xoy)
				{
					pP[2] = m_dynamic_cells[left_p]; // left point
					if(m_valid_flag[left_p] && pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}


				// on xoz plane
				bool draw_xoz = true;
				if( lb>= 0 && ll>=0){ 
					pP[1] = m_dynamic_cells[ll*m_x_step + ly*m_y_step + lb]; // left-before point
					pP[2] = m_dynamic_cells[left_p]; // left point
				}
				else
					draw_xoz = false;
				if(draw_xoz && m_valid_flag[ll*m_x_step + ly*m_y_step + lb] && m_valid_flag[left_p] && pP[1].get() != NULL && pP[2].get()!= NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);		
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoz = false;
				if(draw_xoz){
					pP[2] = m_dynamic_cells[before_p]; // before point
					if(m_valid_flag[before_p] && pP[2].get() != NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}

				// on yoz plane
				bool draw_yoz = true;
				if(lb >=0 && ld>=0)
				{
					pP[1] = m_dynamic_cells[lx*m_x_step + ld*m_y_step + lb]; // before-down point
					pP[2] = m_dynamic_cells[down_p];						// down point
				}
				else
					draw_yoz = false;
				if(draw_yoz && m_valid_flag[lx*m_x_step + ld*m_y_step + lb] && m_valid_flag[down_p] && pP[1].get()!=NULL && pP[2].get()!=NULL && fabs(pP[1]->rgb - pP[0]->rgb)<color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_yoz = false;
				if(draw_yoz)
				{
					pP[2] = m_dynamic_cells[before_p]; // before point
					if(m_valid_flag[before_p] && pP[2].get()!=NULL)
					{
						drawVertex(pP,pImg,pVertex);
						pImg +=9;
						pVertex +=9;
					}
				}
			}
			return;
}