#ifndef VIEWER_H
#define VIEWER_H

#include <boost/thread/thread.hpp>
#include <boost/shared_ptr.hpp>
#include "pcl/common/common_headers.h"
#include "pcl/common/common_headers.h"
#include "pcl/features/normal_3d.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/interactor_style.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include "CPose3D.h"
#include "pcl/registration/ia_ransac.h"
#include "CLogfile.h"

//#define RX 1200 // X [-6 6]
//#define RY 400	// Y [-2 2]
//#define RZ 1200 // Z [-6 6]

#define RX 600 // X [-3 3]
#define RY 400	// Y [-2 2]
#define RZ 600 // Z [-3 3]

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


class MapBuilder{
		
public:
	// to ues this type in .cpp
	/*typedef pcl::PointXYZRGB point_type;
	typedef pcl::PointCloud<point_type> pointcloud_type;*/

	MapBuilder(): viewer("Map Viewer"),new_map(false),blocked(false),refresh_cells(false),
		global_map(new pcl::PointCloud<pcl::PointXYZRGB>),
		global_cell_map(new pcl::PointCloud<pcl::PointXYZRGB>),
		pDisplay(new pcl::PointCloud<pcl::PointXYZRGB>),
		pGlobalDisplay(new pcl::PointCloud<pcl::PointXYZRGB>),
		min_points_distance(0.0004) // 2cm * 2cm as the same point
	{
		global_cells.resize(ALL_CELLS);
		valid_flag .reset();
		m_offset_x=0;
		m_offset_y=0;
		m_offset_z=0;
		calcbounder();
		boost::thread (boost::ref (*this));
	}
	~MapBuilder(){
		//delete pDisplay;
	}
	
	// set local_map, if succeed return true, else return false;
	// this function should be called by father thread
	bool setMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pmap){ 
		static int i=0;
		mutex_map.lock();
		if(blocked)
		{
			mutex_map.unlock();
			return true;
		}
		blocked = !blocked;
		new_map = true;
		local_map = pmap;
		cout<<"set "<<++i<<" th map"<<endl;
		mutex_map.unlock();
		return true;
	}

	bool setGlobalMapForExperiment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pGlobalmap, CPose3D& pose, int id_of_node){
		static int i=0;
		mutex_map.lock();
		new_map = true;
		cout<<"!!!Get id_of_node: "<<id_of_node<<endl;
	
		Id_Cells_Pose.insert(make_pair(id_of_node,pose));
		id_of_cur_node = id_of_node;
		pDisplay.swap(pGlobalmap);
		mutex_map.unlock();
		return true;
	}
	//// interface for input raw_cloudpoint and CPose3D
	bool setrawmapwithtransform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& rawmap, boost::shared_ptr<CPose3D>& pose, int id_of_node=0){
		static int i=0;
		mutex_map.lock();
		//if(blocked) // synchronization with slam
		//{
		//	mutex_map.unlock();
		//	return false;
		//}
		//blocked = !blocked;
		new_map = true;
		//local_map = rawmap;
		local_map_set.push_back(rawmap);
		//_pose = pose;
		_pose_set.push_back(pose);
		//cout<<"set "<<++i<<" th map"<<endl;

		// insert current_pose of node id
		Id_Cells_Pose.insert(make_pair(id_of_node,*pose));
		id_of_cur_node = id_of_node;
		mutex_map.unlock();
		return true;
	}
	void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// --------------------------------------------------------------------------
		//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
		// --------------------------------------------------------------------------
		Eigen::Matrix4f	HM;
		pose->getHomogeneousMatrix(HM);

		/*using pcl::transformation instead*/ 
		pcl::transformPointCloud (*cloud, *cloud, HM);
	}
	void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB> * cloud)
	{
		// --------------------------------------------------------------------------
		//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
		// --------------------------------------------------------------------------
		Eigen::Matrix4f	HM;
		pose->getHomogeneousMatrix(HM);

		/*using pcl::transformation instead*/ 
		pcl::transformPointCloud (*cloud, *cloud, HM);
	}

	// Main 
	int m_offset_x;		// Offset along x-axis
	int m_offset_y;		// Offset along y-axis
	int m_offset_z;		// Offset along z-axis

	double m_upper_x,m_lower_x;  // Bounder along x-axis
	double m_upper_y,m_lower_y;	// Bounder along y-axis
	double m_upper_z,m_lower_z;	// Bounder along z-axis

	void calcoffset();		// Calculate offset based on current Basic Point
	void calcbounder();		// Calculate bounder box 
	void TranslateArea(CPose3D& trans);
	void FromPC2Cell(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &cloud); // Fill Area with PC

	CPose3D m_BasicPose;
	bitset<ALL_CELLS> m_valid_flag;

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
	// erase Infinite points
	void eraseInfinitePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// Copy from CPointCloud to Cells
	void fromPCtoCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud,int id_of_node = 0);
	// Show globalCells
	void ShowGlobalCells(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	bool IsNoisePoint(int index,boost::shared_ptr<pcl::PointXYZRGB>& p);

	// all cells are set invalid
	inline void CleanCells(){
		cells_map.lock();
		global_cell_map->points.clear();
		//pGlobalDisplay->points.clear();
		//pDisplay.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
		valid_flag.reset();		// 
		refresh_cells = true;	// refresh all cells
		cells_map.unlock();
	};

	// Connects Nodes with Cells

	typedef map<int,pcl::PointCloud<pcl::PointXYZRGB>::Ptr > Node_pc;
	Node_pc Node_To_Cells;
	void transformPointsofNode(int id_of_node, CPose3D& pose);
	void RefreshCells(){
		cells_map.lock();
		cout<<"when refresh Cells, update BasicPose of Area!"<<endl;
		TranslateArea(Id_Cells_Pose.rbegin()->second);		// translate pose???
		fromPCtoCells(pGlobalDisplay);
		pGlobalDisplay->points.clear();
		cells_map.unlock();
	};

	typedef bitset<ALL_CELLS> Cells_id;					// index of cell number
	typedef struct Cell_Element{
		int index_of_cell;
		boost::shared_ptr<pcl::PointXYZRGB> _p;
		struct Cell_Element():_p(new pcl::PointXYZRGB){};
		struct Cell_Element(int index,boost::shared_ptr<pcl::PointXYZRGB>& p):index_of_cell(index),_p(p){};
	}cell_info;
	//typedef	std::map<int,pcl::PointXYZRGB> Cells_set;	// Cells set from a Point Cloud
	typedef vector<cell_info> Cells_set;
	std::map<int, Cells_set> Id_Cells_Set;				// From Id of Node to Cells Set
	std::map<int, Cells_id> Id_Cells_Id;				// From Id of Node to Cells Id
	std::map<int, CPose3D> Id_Cells_Pose;				// From Id of Node to Cells Pose
	int id_of_cur_node;

	// generate Cells set from point cloud
	//void insertCellsSet(int id_of_Node,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);
	// transform Cells according to responding Node
	void tarnsformofCells(int id_of_Node, CPose3D& pose);
	
public:
	pcl::visualization::CloudViewer viewer;

	 // thread synchronization
	 boost::mutex mutex_map;
	 boost::mutex cells_map;
	 bool blocked; // if this thread is showing cloud, then, cannot obtain a new map
	 bool new_map ; // indicate whether a new map has been obtained
	 boost::shared_ptr<CPose3D> _pose; // for cloud registration
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_map; //current frame map after slam
	 std::list< pcl::PointCloud<pcl::PointXYZRGB>::Ptr> local_map_set; // all the local maps that should be displayed
	 std::list< boost::shared_ptr<CPose3D> > _pose_set; // for cloud registration
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map; // global map
	 std::vector<int> weighted; // use MRPT weighted point Alignment
	 float min_points_distance; // determine whether these two points are the same point

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cell_map; // to display cells
	 vector<boost::shared_ptr<pcl::PointXYZRGB> > global_cells; // all the discrete cells 5*5*5

	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr pDisplay;
	 pcl::PointCloud<pcl::PointXYZRGB>::Ptr pGlobalDisplay;
	 
	 // whether to refresh all cell-map
	 bool refresh_cells;
	 // index whether cells are still valid
	 bitset<ALL_CELLS> valid_flag;

	 // This is for recording mapbuilder-time-consumed
	 CLogfile mylogfile;

public:
	//void operator() (){  // thread function
	//	while(!viewer.wasStopped()){
	//		mutex_map.lock();
	//		if(refresh_cells){
	//			refresh_cells = false;
	//			RefreshCells(); // Refresh all cells
	//			
	//		}
	//		if(!new_map) // local map has not been updated
	//		{
	//			mutex_map.unlock();
	//			boost::thread::yield();
	//		}
	//		else{
	//			/*if(local_map_set.size() == 0)
	//			{
	//				new_map = false;
	//				mutex_map.unlock();
	//				continue;
	//			}*/

	//			cout<<"mapbuilder: "<<endl;
	//			static int i = 0;
	//			new_map = false;
	//			

	//			if(pDisplay.get()==NULL){
	//				//mutex_map.unlock();
	//				continue;	
	//			}
	//			
	//			cells_map.lock();
	//			cout<<"!!!Record id of node: "<< id_of_cur_node<<endl;
	//			fromPCtoCells(pDisplay,id_of_cur_node); // add this local_map into global_cells
	//			//std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >
	//			if(pDisplay.get()!=	NULL)
	//				pDisplay->points.swap(std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >() );
	//				//pDisplay->points.swap(vector<pcl::PointXYZRGB>());
	//			//pDisplay->delete();
	//			mutex_map.unlock();
	//			viewer.showCloud(global_cell_map);
	//			cells_map.unlock();

	//			cout<<"after show cloud!"<<endl;
	//			// end of mapbuilder
	//			//double end_t_mapbuilder = ::GetTickCount();
	//			//char buf[100];
	//			//sprintf(buf,"\t %0.2f",end_t_mapbuilder - start_t_mapbuilder);
	//			//string str_builder(buf);
	//			//mylogfile.writeintolog(str_builder,true); // write time cost by mapbuilder

	//			//viewer.showCloud(global_map);
	//			//mutex_map.unlock();
	//		}
	//	}
	//}
	void operator() (){  // thread function
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmplocal_map(new pcl::PointCloud<pcl::PointXYZRGB>); 
		while(!viewer.wasStopped()){
			mutex_map.lock();
			if(!new_map) // local map has not been updated
			{
				mutex_map.unlock();
				boost::thread::yield();
			}
			else{
				if(local_map_set.size() == 0)
				{
					new_map = false;
					mutex_map.unlock();
					continue;
				}

				cout<<"mapbuilder: "<<endl;
					static int i = 0;
					//	blocked = false;
					
					// start of mapbuilder
					//double start_t_mapbuilder = ::GetTickCount();

					// get last point_cloud and pose, then delete this Node info
					local_map = *(local_map_set.begin());
					_pose = *(_pose_set.begin());
					
					// delete this Node info
					local_map_set.pop_front();
					_pose_set.pop_front();
				
					mutex_map.unlock();

					if(local_map.get()==NULL){
						//mutex_map.unlock();
						continue;	
					}

					cells_map.lock(); // synchronization with CleanCells()
					// this step is only for registration raw_cloudpoint
					cloudRegistration(_pose,local_map);
					// MergeLocalMap();
					
					//fromPCtoCells(local_map); // add this local_map into global_cells
					//viewer.showCloud(global_cell_map);
					
					tmplocal_map->points.insert(tmplocal_map->points.end(),local_map->points.begin(),local_map->points.end());
					viewer.showCloud(tmplocal_map);

					cells_map.unlock();
					cout<<"after show cloud!"<<endl;
					// end of mapbuilder
					//double end_t_mapbuilder = ::GetTickCount();
					//char buf[100];
					//sprintf(buf,"\t %0.2f",end_t_mapbuilder - start_t_mapbuilder);
					//string str_builder(buf);
					//mylogfile.writeintolog(str_builder,true); // write time cost by mapbuilder

					//viewer.showCloud(global_map);
					//mutex_map.unlock();
			}
		}
	}

public:

	// Merge local with global 
	void MergeLMapwithGMap(pcl::PointCloud<pcl::PointXYZRGB> * global_map,pcl::PointCloud<pcl::PointXYZRGB> * local_map){
			size_t index = global_map->points.size();
			global_map->points.resize(global_map->points.size() + local_map->points.size());
			for(size_t i=0;i<local_map->points.size();i++)
				global_map->points[index+i] = local_map->points[i];
			//global_map = local_map;
			return ;	
	}
	//// Merge local with global 
	void MergeLocalMap(){
		if(global_map->points.size()==0)// this is the first frame
		{
			global_map = local_map;
			weighted.resize(global_map->size(),1);
			return ;
		}

		// Use a KdTree to search for the nearest matches in feature space
		pcl::KdTreeFLANN<pcl::PointXYZRGB> descriptor_kdtree;
		descriptor_kdtree.setInputCloud (global_map);

		// Find the index of the best match for each keypoint, and store it in "correspondences_out"
		const int k = 1;
		std::vector<int> k_indices (k);
		std::vector<float> k_squared_distances (k);

		for (size_t i = 0; i < local_map->points.size (); ++i)
		{
			descriptor_kdtree.nearestKSearch (*local_map, i, k, k_indices, k_squared_distances);

			if(k_squared_distances[0] < min_points_distance) // if this two points are the same point
			{
				pcl::PointXYZRGB& tp = global_map->points[k_indices[0]];
				pcl::PointXYZRGB& sp = local_map->points[i];
				float factor = 1.0/(float)(weighted[k_indices[0]] + 1);
				tp.x = weighted[k_indices[0]]*tp.x * factor + sp.x * factor;
				tp.y = weighted[k_indices[0]]*tp.y * factor + sp.y * factor;
				tp.z = weighted[k_indices[0]]*tp.z * factor + sp.z * factor;
				weighted[k_indices[0]] ++;
			}
			else // this point is a new point
			{
				global_map->points.push_back(local_map->points[i]);
				weighted.push_back(1);
			}
		}	
	}

};

#endif