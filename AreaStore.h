#pragma once
#include <stdio.h>
#include <string>
#include <map>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


typedef struct _BGRXYZPt{
	inline _BGRXYZPt(pcl::PointXYZRGB& p){
		x = p.x; y = p.y; z = p.z;
		r = p.r; g = p.g; b = p.b;
	};
	inline _BGRXYZPt(){
		x = 0; y = 0; z = 0;
		r = 0; g = 0; b = 0;
	};
	unsigned char b;
	unsigned char g;
	unsigned char r;
	float x; 
	float y; 
	float z;
} BGRXYZPt;

typedef struct _XYZPt{
	float x; 
	float y; 
	float z;
} XYZPt;

typedef struct _RPYXYZPt{
	float roll; 
	float pitch; 
	float yaw;
	float x; 
	float y; 
	float z;
} RPYXYZPt;

typedef struct _AreaStoreHdr{
	unsigned long id;
	unsigned long version;
	unsigned long timestamp;
	unsigned long size; // the number of points/nodes/pose in area store 
	unsigned char crc; // crc checksum for the data in area store 
	RPYXYZPt refpose; // the reference pose of point in payload. the abosoluted point = transf(refpose) + transf(point)
	XYZPt boundary3D[4]; // the minimum outer enclosure rectangular parallelepiped. [0]: the axis point, [1,2,3]: the other boundary points. 
} AreaStoreHdr;

// Descriptor is CV::MAT(N,64,float), N depends on the number of features.(We use ONLY non-extended surf descriptor)
// 3D Descriptor is the combination of 3D pose and descrpitor of features.
// The serialization of 3D Descriptor is as follows:
// (x0,y0,z0),(0,0,float),(0,1,float),...,(0,63,float); (x1,y1,z1),(1,0,float),(1,1,float),...,(1,63,float);... ;(xN-1,yN-1,zN-1),(N-1,0,float),(N-1,1,float),...,(N-1,63,float);
typedef struct _Feature3DDesc{
	XYZPt xyz;
	float desc[64];
} Feature3DDesc;

typedef struct _Node3DDesc{
	int id; // node ID
	int n; // the number of features
	Feature3DDesc desc3DList[0];
} Node3DDesc;

typedef struct _NodePose{
	int id; // node ID
	RPYXYZPt pose; 
}NodePose;

typedef AreaStoreHdr Area3DDescStoreHdr;
typedef AreaStoreHdr AreaPathStoreHdr;

typedef std::vector<AreaStoreHdr> AreaStoreHdrList;
typedef std::map<int, std::vector<Feature3DDesc> > Area3DDescMap;
typedef std::vector<NodePose> AreaPathList;

class CAreaStore{
public:
	CAreaStore();
	~CAreaStore();
	bool AreaMap2File(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc, AreaStoreHdr& hdr);
	// The data in pc will NOT be erased. The areamap points will be appended onto the given pointcloud.
	// "*.amf" (Area Map File) 
	bool File2AreaMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc, unsigned long id);
	// All the "*.amf" (Area Map File) files in rootpath will be considered as area store files, and the headers will be parsed!! 
	bool GetStoreHdrList(AreaStoreHdrList& hdrs);
	
	// "*.adf" (Area Descriptor File)
	bool Area3DDesc2File(Area3DDescMap& map, Area3DDescStoreHdr& hdr);
	bool File2Area3DDesc(Area3DDescMap& map, unsigned long id);

	// "*.apf" (Area Path File)
	bool AreaPath2File(AreaPathList& path, AreaPathStoreHdr& hdr);
	bool File2AreaPath(AreaPathList& path, unsigned long id);

	bool LoadConfig(std::string rootpath);
private:
	typedef enum _AreaFileType{
		Map=0,
		Desc,
		Path
	}AreaFileType;
	FILE* OpenAreaFile(unsigned long id, AreaFileType type, const char* mode="r");
	std::string m_RootPath; // e.g. "d:\\areastore"
};
bool TestAreaStore();
void GenTestNodeDesc(std::vector<Feature3DDesc>& descVector);