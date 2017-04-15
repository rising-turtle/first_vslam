#include "Viewer.h"


void MapBuilder::calcbounder()
{
	double xyz[3];
	m_BasicPose.getXYZ(xyz);
	m_lower_x=xyz[0]-L_RX;
	m_upper_x=xyz[0]+L_RX;
	m_lower_y=xyz[1]-L_RY;
	m_upper_y=xyz[1]+L_RY;
	m_lower_z=xyz[2]-L_RZ;
	m_upper_z=xyz[2]+L_RZ;
}
void MapBuilder::TranslateArea(CPose3D& trans)
{
	this->m_BasicPose+=trans;
	calcoffset();
	calcbounder();
	// clear points in cell and PC
	m_valid_flag.reset();
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > p_tmp_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	this->global_cell_map.swap(p_tmp_pc);
	FromPC2Cell(p_tmp_pc);
}

// Calculate offset based on current Basic Point
void MapBuilder::calcoffset()
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

void MapBuilder::FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud) // Fill Area with PC
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
				this->global_cells[index] = p;
				this->global_cell_map->points.push_back(sp);
				pc->points.push_back(sp);
			}
		}
	}
	cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	this->m_valid_flag |= cur_frame; // 
}
// erase Infinite points
void MapBuilder::eraseInfinitePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	/** \brief The point data. */
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it = pCloud->points.begin();
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it_begin = it;
	std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it_end = it;

	int index = 0;
	bool flag = false;
	int num = 0; // record number of erased points
	for(size_t i=0; i< pCloud->points.size();  ){
		if (!pcl_isfinite ((*it).x) || 
			!pcl_isfinite ((*it).y) || 
			!pcl_isfinite ((*it).z))
		{
			if(!flag){ // first find begin of it
				flag = true;
				it_begin = it;
			}
			num ++;
			//it = pCloud->points.erase(it);
			it++;
			i++;
			continue;
		}
		if(flag) { // second find end of it
			it_end = it;
			flag = false;
			it = pCloud->points.erase(it_begin,it_end);
			i -= num;
			num = 0;
			continue;
		}
		it++;
		i++;
	}
}

bool MapBuilder::IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p){
	 
	 static float error_noise = 1e-2;
	 boost::shared_ptr<pcl::PointXYZRGB> sp;// = global_cells[i];
	 int l_x = index - X_STEP;
	 int r_x = index + Y_STEP;
	 int l_y = index - Y_STEP;
	 int r_y = index + Y_STEP;
	 int l_z = index - 1;
	 int r_z = index + 1;//

	 // we will search longer distance along Z-axis 
	 int range_z = 2;

	 if(l_x >=0 && l_x <ALL_CELLS){
		 sp = global_cells[l_x];
		 if(valid_flag[l_x] && sp.get() != NULL) // first using bit-vector to make judgement
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_x >=0 && r_x <ALL_CELLS){
		 sp = global_cells[r_x];
		 if(valid_flag[r_x] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(l_y >=0 && l_y <ALL_CELLS){
		 sp = global_cells[l_y];
		  if(valid_flag[l_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_y >=0 && r_y <ALL_CELLS){
		 sp = global_cells[r_y];
		  if(valid_flag[r_y] && sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }

	 for(int i= index - range_z; i<= index + range_z; i++)
	 {
		 if( i < 0 || i >=ALL_CELLS  )
			 continue;
		 sp = global_cells[i];
		 if(valid_flag[i] && sp.get()!=NULL)
			 if(fabs(sp->rgb - p->rgb) < error_noise)
				 return true;
	 }
	 /*if(l_z >0 && l_z <ALL_CELLS){
		 sp = global_cells[l_z];
		  if(sp.get() != NULL)
			 if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }
	 if(r_z >0 && r_z <ALL_CELLS){
		 sp = global_cells[r_z];
		  if(sp.get() != NULL)
			if(fabs(sp->rgb - p->rgb)<error_noise)
				 return true;
	 }*/
	return false;
}

// transform pc of this node into a global pc
void MapBuilder::transformPointsofNode(int id_of_node, CPose3D& pose){
	
	Node_pc::iterator it_of_node = Node_To_Cells.find(id_of_node);
	if(it_of_node == Node_To_Cells.end())
	{
		cout<<"!!! id_of node "<<id_of_node<<" does not exist in Cells!!!"<<endl;
		return ;
	}
	std::map<int, CPose3D>::iterator it_cell_pose = Id_Cells_Pose.find(id_of_node);
	if(it_cell_pose == Id_Cells_Pose.end())
	{	
		cout<<"!!! id of node "<<id_of_node<<" does not exist in Pose!!!"<<endl;
		return ;
	}
	

	CPose3D trans_pose = pose - it_cell_pose->second; // trans_pose
	cout<<"This is transform in every Node!!"<<endl;
	trans_pose.output(std::cout);
	it_cell_pose->second = pose;
	boost::shared_ptr<CPose3D> pPose(new CPose3D(trans_pose));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = it_of_node->second;
	cloudRegistration(pPose,pc);
	MergeLMapwithGMap(pGlobalDisplay.get(),pc.get());
}

// transform Cells according to responding Node
void MapBuilder::tarnsformofCells(int id_of_Node, CPose3D& pose){
	
	//// transform cells belong to Node id to new Cells based on pose
	//std::map<int, Cells_set>::iterator it_cell_set = Id_Cells_Set.find(id_of_Node);				// From Id of Node to Cells Set
	//std::map<int, Cells_id>::iterator it_cell_id = Id_Cells_Id.find(id_of_Node);				// From Id of Node to Cells Id
	//std::map<int, CPose3D>::iterator it_cell_pose = Id_Cells_Pose.find(id_of_Node);				// From Id of Node to Cells Pose

	//Cells_set * pCellSet = &(it_cell_set->second);
	//CPose3D trans_pose = pose - it_cell_pose->second; // trans_pose
	//it_cell_pose->second = pose;					  // update pose of this node
	//
	//Cells_id * pCellId = &(it_cell_id->second);
	//
	//for(size_t i=0;i<pCellSet->size();i++)
	//{
	//	int pre_index = (*pCellSet)[i].index_of_cell; // current index of cell
	//	boost::shared_ptr<pcl::PointXYZRGB> sp = (*pCellSet)[i]._p;
	//	double lx = sp->x; double ly = sp->y; double lz = sp->z;
	//	double gx,gy,gz;
	//	trans_pose.transformPoint(lx,ly,lz,gx,gy,gz);
	//	int new_index = getIndexCell(gx,gy,gz);

	//	(*pCellSet)[i].index_of_cell = new_index; // update this point 
	//	sp->x = gx; sp->y = gy; sp->z = gz;

	//	pCellId->reset(pre_index);			// invalid pre_index
	//	pCellId->set(new_index);			// valid new_index

	//	if(!valid_flag[new_index])	 // this new cell is empty
	//	{
	//		boost::shared_ptr<pcl::PointXYZRGB> p = new 
	//	}
	//}

	
}
 // Copy from CPointCloud to Cells
void MapBuilder::fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud,int id_of_node){
	
	int N = pCloud->points.size();
	vector<bool> index_point(N,false);
	vector<int> index_set;
	static bitset<ALL_CELLS> cur_frame;
	cur_frame.reset();

//	std::map<int, Cells_set>::iterator it_cell_set = (Id_Cells_Set.insert(make_pair(id_of_node,vector<cell_info>()))).first;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	for(size_t i=0;i<pCloud->points.size();i++)
	{
		pcl::PointXYZRGB& sp = pCloud->points[i];
		int index = getIndexCell(sp.x,sp.y,sp.z);
		if(index >=0 )
		{
			if( valid_flag[index]/*global_cells[index].get()!=NULL*/) // this is already painted
				continue; // not flush this point using the new point	
			boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
			p->x = sp.x; p->y = sp.y; p->z = sp.z; 
			p->r = sp.r; p->g = sp.g; p->b = sp.b;
			if(!IsNoisePoint(index,p))
			{
				if(!cur_frame[index]) // current frame does not contain this point
				{
					cur_frame.set(index);
					//valid_flag.set(index);
					global_cell_map->points.push_back(sp);
					pc->points.push_back(sp);
				
//					it_cell_set->second.push_back(cell_info(index,p));
				}
				//index_point[i] = true;
				//index_set.push_back(index);
			}
		}
	}
	// insert bitset of cur_frame
//	Id_Cells_Id.insert(make_pair(id_of_node,cur_frame)); // 

	cout<<"!!! N of points into Node: "<< pc->points.size()<<endl;
	Node_To_Cells.insert(make_pair(id_of_node,pc));
	valid_flag |= cur_frame; // 

	//for(size_t i=0,j=0;i<N;i++)
	//	if(index_point[i])
	//	{
	//		pcl::PointXYZRGB& sp = pCloud->points[i];
	//		boost::shared_ptr<pcl::PointXYZRGB> p(new pcl::PointXYZRGB);
	//		p->x = sp.x; p->y = sp.y; p->z = sp.z; 
	//		p->r = sp.r; p->g = sp.g; p->b = sp.b;
	//		int index_cell = index_set[j++];
	//		if(global_cells[index_cell].get()== NULL){
	//			global_cells[index_cell/*index_set[j++]*/] = p;
	//			global_cell_map->points.push_back(sp);
	//			// set valid flag for index_cell
	//			valid_flag.set(index_cell);
	//		}
	//	}
}
// Show globalCells
void MapBuilder::ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud){
	for(size_t i=0;i<global_cells.size();i++){
		boost::shared_ptr<pcl::PointXYZRGB> sp = global_cells[i];
		if(sp.get() == NULL)
			continue;
		pCloud->points.push_back(*sp);
	}
}
