#define UPPER_ASS(Upp,V){if(V>Upp) Upp=V;}
#define AMEM 1024*1024*300

template <typename PointT>
CNewDynamicArea<PointT>::CNewDynamicArea():m_pPC(new pcl::PointCloud<PointT>)
{
	m_lower_x=0;
	m_lower_y=0;
	m_lower_z=0;
	m_upper_x=0;
	m_upper_y=0;
	m_upper_z=0;

	// default cell_size=2 power2 
	m_CellSize=2;

	initArea();
}

template <typename PointT>
CNewDynamicArea<PointT>::~CNewDynamicArea(){}

template <typename PointT>
void CNewDynamicArea<PointT>::initArea()
{
	m_x_offset=(int)(m_lower_x*100-1);
	m_y_offset=(int)(m_lower_y*100-1);
	m_z_offset=(int)(m_lower_z*100-1);

	m_x_range=(int)((m_upper_x-m_lower_x)*100);
	m_y_range=(int)((m_upper_y-m_lower_y)*100);
	m_z_range=(int)((m_upper_z-m_lower_z)*100);

	m_x_cell=m_x_range>>m_CellSize; // cell_size = 2p2=4
	m_y_cell=m_y_range>>m_CellSize;
	m_z_cell=m_z_range>>m_CellSize;

	m_x_step=m_y_cell*m_z_cell;
	m_y_step=m_z_cell;
	m_all_cells=m_x_cell*m_y_cell*m_z_cell;
	if(m_all_cells!=0)
		cout<<"!!!!!m_all_cells: "<<m_all_cells<<endl;
	if(m_all_cells<0) 
		m_all_cells=0;
	//cout<<"All_Cells: "<<(m_all_cells*4)/1024<<"KB"<<endl;
	m_pCell.swap(std::vector< std::vector<int> >());
	m_pCell.resize(m_all_cells,vector<int>());
	while( m_all_cells*4 >= AMEM || (m_all_cells>0 && m_pCell.size()==0))
	{
		m_CellSize++;
		//cout<<"Cell_Size: "<<m_s_CellSize<<endl;
		m_x_cell=m_x_range>>m_CellSize; // cell_size = 2p2=4
		m_y_cell=m_y_range>>m_CellSize;
		m_z_cell=m_z_range>>m_CellSize;

		m_x_step=m_y_cell*m_z_cell;
		m_y_step=m_z_cell;
		m_all_cells=m_x_cell*m_y_cell*m_z_cell;
		cout<<"All_Cells: "<<((m_all_cells*4)/1024)/1024<<"MB"<<endl;
		m_pCell.resize(m_all_cells);
	}

	//m_dynamic_cells.resize(m_all_cells);
	m_valid_cell.resize(0);
	m_valid_cell.resize(m_all_cells,false);
	m_valid_cell.reset();
}
template <typename PointT>
void CNewDynamicArea<PointT>::unitArea()
{	
	//m_pCell.swap(std::vector<std::vector<int>  >());
	m_pPC->points.clear();
	m_valid_cell.resize(0,false);
}

template <typename PointT>
void CNewDynamicArea<PointT>::reset(float l_x,float u_x,float l_y,float u_y,float l_z,float u_z)
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
UINT CNewDynamicArea<PointT>::GetCellIndex(float x,float y, float z)
{
	if(_isnan(x) || _isnan(y) || _isnan(z))
		return -1;
	if(x>=m_upper_x || y>=m_upper_y || z>=m_upper_z \
		|| x<m_lower_x || y<m_lower_y || z<m_lower_z )
		return -1;
	int lx = floor(( x*100 - m_x_offset)+0.5);
	lx>>=m_CellSize;//divide by cell_size
	int ly = floor(( y*100 - m_y_offset)+0.5); 
	ly>>=m_CellSize;//divide by cell_size
	int lz = floor(( z*100 - m_z_offset)+0.5); 
	lz>>=m_CellSize;//divide by cell_size

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

	return (lx*m_x_step + ly*m_y_step + lz);
}
template <typename PointT>
void CNewDynamicArea<PointT>::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<PointT> >& pCloud) //FromPC2Cell
{
	int N = pCloud->points.size();
	static boost::dynamic_bitset<> cur_frame;
	cur_frame.resize(m_all_cells,false);
	cur_frame.reset();
	
	//cout<<"PC2Cell PC SIZE: "<<cloud->points.size()<<endl;
	for(size_t i=0;i<pCloud->points.size();i++)
	{
		PointT& sp = pCloud->points[i];
		int index = this->GetCellIndex(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if(!this->m_valid_cell[index])
			{
				m_pCell[index].push_back(i);
				this->m_valid_cell[index]=true;
				continue;
			}// not flush this point using the new point	
			m_pCell[index].push_back(i);
		}
	}
	m_pPC.swap(pCloud);
	//this->m_valid_flag |= cur_frame; //
	return ;
}
