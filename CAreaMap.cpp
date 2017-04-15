#include "CAreaMap.h"

unsigned char *CArea::m_pucImg;
float *CArea::m_pfVertex;
CArea::CArea():m_global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB> )
{
	Init();
}
CArea::~CArea()
{
	UnInit();
}

void CArea::Init() // Initialize CArea
{
	m_global_cells.resize(ALL_CELLS);
	m_offset_x=0;
	m_offset_y=0;
	m_offset_z=0;
	calcbounder();
#ifdef TRIANGLE
	m_pucImg = new unsigned char[ALL_CELLS*3];
	m_pfVertex = new float [ALL_CELLS*3];
	m_IsTriangle = true;
#else
	m_pucImg=new unsigned char[640*480*3];
	m_pfVertex=new float[640*480*3];
	m_IsTriangle = false;
#endif
	//m_pucImg=new unsigned char[640*480*3];
	//m_pfVertex=new float[640*480*3];

	return ;
}
void CArea::UnInit() // UnInitialize CArea
{
	m_global_cells.clear(); // clear cells 
	m_global_cell_map->points.clear(); // clear points in cell_map
}
void CArea::calcbounder()
{
	double xyz[3];
	m_BasicPose.getXYZ(xyz);
	m_lower_x=xyz[0]-L_RX;//+0.04;
	m_lower_y=xyz[1]-L_RY;//+0.04;
	m_lower_z=xyz[2]-L_RZ;//+0.04;
	m_upper_x=xyz[0]+L_RX;
	m_upper_y=xyz[1]+L_RY;
	m_upper_z=xyz[2]+L_RZ;
}
void CArea::TranslateArea(CPose3D& trans)
{
	this->m_BasicPose+=trans;
	calcoffset();
	calcbounder();
	// clear points in cell and PC
	m_valid_flag.reset();
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > p_tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->m_global_cell_map.swap(p_tmp_pc);
	FromPC2Cell(p_tmp_pc);
}

// Calculate offset based on current Basic Point
void CArea::calcoffset()
{
	double xyz[3];
	m_BasicPose.getXYZ(xyz);
	m_offset_x = (xyz[0]*100);// + S_RX); //> >2;/// CELLSIZE;
	m_offset_x>>=2;
	m_offset_y = (xyz[1]*100);// + S_RY); //> >2;/// CELLSIZE;
	m_offset_y>>=2;
	m_offset_z = (xyz[2]*100);// + S_RZ); //> >2;/// CELLSIZE;
	m_offset_z>>=2;

	//for debug
	cout<<"current Basic Point:"<<endl;
	m_BasicPose.output(std::cout);
	cout<<"offset:(x,y,z) "<<m_offset_x<<" "<<m_offset_y<<" "<<m_offset_z<<endl;
}

void CArea::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud)
{

	//C3DMap *pc3Dmap=(C3DMap *)m_stClassPtrs.p3Dmap; 
	int N = cloud->points.size();

	static bitset<ALL_CELLS> cur_frame;
	cur_frame.reset();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<cloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = cloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(this->m_valid_flag[index] /*pc3Dmap->m_global_cells[index].get()!=NULL*/) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!cur_frame[index]){
				cur_frame.set(index);
				this->m_global_cells[index] = p;
				this->m_global_cell_map->points.push_back(sp);
				pc->points.push_back(sp);
			}
		}
	}
	cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	this->m_valid_flag |= cur_frame; // 
}

void CArea::drawVertex(boost::shared_ptr<pcl::PointXYZRGB> p[3],unsigned char *pucImg,float *pfVertex){
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
// From Cells to Triangles pucImg->Color Array, pfVertex->Vertex Array
void CArea::FromCell2Triangles(unsigned char *pucImg,float *pfVertex)
{
	static double color_similar = 3;
	bool d_left,d_behind,d_down;
	boost::shared_ptr<pcl::PointXYZRGB> pself;
	boost::shared_ptr<pcl::PointXYZRGB> pP[3];
	int d_n = 0;
	unsigned char *pImg = pucImg;
	float * pVertex = pfVertex;
	//CMapBuilder * pMapbuilder = (CMapBuilder*)this->m_stClassPtrs->pMapBuilder;
	// create ShowList
	/*GLuint cloud_list_index = glGenLists(1);
	if(!cloud_list_index) {
	cout<<"No display list could be created"<<endl;
	return -1;
	}*/
	//glNewList(cloud_list_index, GL_COMPILE);
	//cloud_list_indices.push_back(cloud_list_index);


	for(int lx=0;lx<X_CELL;lx++) // traverse along x-axis
		for(int ly=0;ly<Y_CELL;ly++) // traverse along y-axis
			for(int lz=0;lz<Z_CELL;lz++) // traverse along z-axis
			{
				// initilization 

				// find point (lx,ly,lz) 
				int index_self = lx*X_STEP + ly*Y_STEP + lz; 
				pself = m_global_cells[index_self];
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

				int left_p = ll*X_STEP + ly*Y_STEP + lz;
				int before_p = lx*X_STEP + ly*Y_STEP + lb;
				int down_p = lx*X_STEP + ld*Y_STEP + lz;

				// down point on xoy
				if( ll>=0 && ld >=0)
				{
					pP[1] = m_global_cells[ll*X_STEP + ld*Y_STEP + lz]; // down-left point
					pP[2] = m_global_cells[down_p]; // down point
				}
				else
					draw_xoy = false;
				if(draw_xoy && m_valid_flag[ll*X_STEP + ld*Y_STEP + lz] && m_valid_flag[down_p] && pP[1].get() != NULL && pP[2].get() != NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoy = false;
				if(draw_xoy)
				{
					pP[2] = m_global_cells[left_p]; // left point
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
					pP[1] = m_global_cells[ll*X_STEP + ly*Y_STEP + lb]; // left-before point
					pP[2] = m_global_cells[left_p]; // left point
				}
				else
					draw_xoz = false;
				if(draw_xoz && m_valid_flag[ll*X_STEP + ly*Y_STEP + lb] && m_valid_flag[left_p] && pP[1].get() != NULL && pP[2].get()!= NULL && fabs(pP[1]->rgb - pP[0]->rgb) < color_similar)
				{
					drawVertex(pP,pImg,pVertex);		
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_xoz = false;
				if(draw_xoz){
					pP[2] = m_global_cells[before_p]; // before point
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
					pP[1] = m_global_cells[lx*X_STEP + ld*Y_STEP + lb]; // before-down point
					pP[2] = m_global_cells[down_p];						// down point
				}
				else
					draw_yoz = false;
				if(draw_yoz && m_valid_flag[lx*X_STEP + ld*Y_STEP + lb] && m_valid_flag[down_p] && pP[1].get()!=NULL && pP[2].get()!=NULL && fabs(pP[1]->rgb - pP[0]->rgb)<color_similar)
				{
					drawVertex(pP,pImg,pVertex);
					pImg +=9;
					pVertex +=9;
				}
				else
					draw_yoz = false;
				if(draw_yoz)
				{
					pP[2] = m_global_cells[before_p]; // before point
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
