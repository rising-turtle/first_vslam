#include "CPolygonCurve.h"
#include <iostream>
#include <Eigen/Eigenvalues>
#include <pcl/common/eigen.h>
using namespace std;

CPolygonCurve::HD CPolygonCurve::_heading_vector;
/************************************************************************/
/*   CPolygonCurve        Constructor    from three Points                         */
/************************************************************************/

CPolygonCurve::CPolygonCurve(){};// Yet not finish this constructor
CPolygonCurve::CPolygonCurve(pcl::PointXYZRGBNormal& pt):m_pVertex(new pcl::PointCloud<pcl::PointXYZ>),
m_pColorVertex(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pNormals(new pcl::PointCloud<pcl::Normal>)
{
	pcl::PointXYZ sp;
	pcl::PointXYZRGB spc;
	spc.x=TotalGP.x=sp.x=GravityP.x=pt.x; 
	spc.y=TotalGP.y=sp.y=GravityP.y=pt.y; 
	spc.z=TotalGP.z=sp.z=GravityP.z=pt.z;
	spc.r=TotalGP.R=GravityP.R=pt.r; 
	spc.g=TotalGP.G=GravityP.G=pt.g; 
	spc.b=TotalGP.B=GravityP.B=pt.b;
	m_upper_x=m_lower_x=pt.x;
	m_upper_y=m_lower_y=pt.y;
	m_upper_z=m_lower_z=pt.z;
	TotalNormalV.nx=NormalV.nx=pt.normal_x;
	TotalNormalV.ny=NormalV.ny=pt.normal_y;
	TotalNormalV.nz=NormalV.nz=pt.normal_z;
	m_num=1;
	NormalizeVector(NormalV.nx,NormalV.ny,NormalV.nz);

	m_pVertex->points.push_back(sp);
	m_pColorVertex->points.push_back(spc);
}
CPolygonCurve::CPolygonCurve(pcl::PointXYZRGB& pt):m_pColorVertex(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	TotalGP.x=GravityP.x=pt.x; TotalGP.y=GravityP.y=pt.y; TotalGP.z=GravityP.z=pt.z;
	TotalGP.R=GravityP.R=pt.r; TotalGP.G=GravityP.G=pt.g; TotalGP.B=GravityP.B=pt.b;
	m_upper_x=m_lower_x=pt.x;
	m_upper_y=m_lower_y=pt.y;
	m_upper_z=m_lower_z=pt.z;
	m_num=1;
	m_pColorVertex->points.push_back(pt);
}
CPolygonCurve::~CPolygonCurve(){};
void CPolygonCurve::UpdateVectorWith(CPolygonCurve* pOther)
{
	if(pOther == NULL) {
		cout<<"pOther invalid in UpdateVectorWith!"<<endl;	
		return ;
	}

	//There is possible that these two vectors are in the different directions to form angle
	if( (NormalV.nx * pOther->NormalV.nx + NormalV.ny *pOther->NormalV.ny + NormalV.nz * pOther->NormalV.nz) <0 )
	{
		pOther->NormalV.nx *= -1.f;
		pOther->NormalV.ny *= -1.f;
		pOther->NormalV.nz *= -1.f;
	}

	size_t nThis = pKeyVertex.size();
	size_t nOther = pOther->pKeyVertex.size();
	float fThis = (float)nThis/(float)(nThis + nOther);
	NormalV.nx = NormalV.nx * fThis + (1.f - fThis)*pOther->NormalV.nx;
	NormalV.ny = NormalV.ny * fThis + (1.f - fThis)*pOther->NormalV.ny;
	NormalV.nz = NormalV.nz * fThis + (1.f - fThis)*pOther->NormalV.nz;


	//NormalV.nx = NormalV.nx * 0.5 + 0.5*pOther->NormalV.nx;
	//NormalV.ny = NormalV.ny * 0.5 + 0.5*pOther->NormalV.ny;
	//NormalV.nz = NormalV.nz * 0.5 + 0.5*pOther->NormalV.nz;

	NormalizeVector(NormalV.nx,NormalV.ny,NormalV.nz);
}

void CPolygonCurve::UpdateGravityPointWith(CPolygonCurve* pOther){

	size_t total  = pKeyVertex.size() + pOther->pKeyVertex.size();
	float factor = 1.f / (float) total;
	float factor1 = (float) pKeyVertex.size()*factor;
	float factor2 = (float) pOther->pKeyVertex.size() * factor;

	// Adjust gravity point according to weighted power
	GravityP.x = GravityP.x * factor1 + pOther->GravityP.x * factor2;
	GravityP.y = GravityP.y * factor1 + pOther->GravityP.y * factor2;
	GravityP.z = GravityP.z * factor1 + pOther->GravityP.z * factor2;
}
void CPolygonCurve::CalculateVector(P3D&a, P3D&b, P3D&c) 
{
	// VN = BA (*) CA
	/*float f1x = b.m_coords[0] - a.m_coords[0];
	float f1y = b.m_coords[1] - a.m_coords[1];
	float f1z = b.m_coords[2] - a.m_coords[2];
	float f2x = c.m_coords[0] - a.m_coords[0];
	float f2y = c.m_coords[1] - a.m_coords[1];
	float f2z = c.m_coords[2] - a.m_coords[2];*/

	float f1x = b.x - a.x;
	float f1y = b.y - a.y;
	float f1z = b.z - a.z;
	float f2x = c.x - a.x;
	float f2y = c.y - a.y;
	float f2z = c.z - a.z;
	NormalV.nx = f1y*f2z - f1z*f2y;
	NormalV.ny = -(f1x*f2z - f1z*f2x);
	NormalV.nz = f1x*f2y - f1y*f2x;

	NormalizeVector(NormalV);

}

void CPolygonCurve::CalculateGravity(){
	float sx,sy,sz;
	sx = sy = sz = 0;
	size_t KeyN = pKeyVertex.size();
	float factor = 1.0f/(float)KeyN;//_n;

	for(size_t i=0;i<KeyN;i++)
	{
		sx+=pKeyVertex[i].x;
		sy+=pKeyVertex[i].y;
		sz+=pKeyVertex[i].z;
	}

	/*for(size_t i=0;i<_n;i++)
	{
	sx+=pVertex[i].x;
	sy+=pVertex[i].y;
	sz+=pVertex[i].z;
	}*/
	GravityP.x = sx * factor;
	GravityP.y = sy * factor;
	GravityP.z = sz * factor;
}
void CPolygonCurve::CalculateDistanceBetweenPlane(P3D& o, double& dis) const // Calculate Distance of Point to surface
{
	CalculateDistanceBetweenPlane(o.x,o.y,o.z,dis);
}
void CPolygonCurve::CalculateDistanceBetweenPlane(float& x,float& y, float& z, double& dis) const
{
	float nx = NormalV.nx; float ny = NormalV.ny; float nz = NormalV.nz;
	//double dis_down = sqrt(NormalV.nx * NormalV.nx + NormalV.ny*NormalV.ny + NormalV.nz*NormalV.nz);
	/*if(dis_down == 0.0) {
	dis = 0; 
	return ;
	}*/
	double dis_up = fabs(x*nx + y*ny + z*nz -nx*GravityP.x - ny*GravityP.y - nz*GravityP.z );
	dis = dis_up;
	return ;
}
bool CPolygonCurve::InTheSameSurface(CPolygonCurve& o)
{
	double dis;
	CalculateDistanceBetweenPlane(o.GravityP.x,o.GravityP.y,GravityP.z,dis); // Calculate surface O's gravity 
	dis*= 100; // from m  to cm
	if(dis < thresh_dis) return true; // 12cm will be fuse within one plane

	return false;
}


void CPolygonCurve::MergeWith(CPolygonCurve& o)
{
	/*Add Vertexes from o and Update Gravity*/
	for(size_t i=0;i<o.pVertex.size();i++){
		pVertex.push_back(o.pVertex[i]);
	}
	/*Add all KeyVertexes from o*/
	for(size_t i=0;i<o.pKeyVertex.size();i++)
	{
		pKeyVertex.push_back(o.pKeyVertex[i]);
	}
	CalculateGravity();
	/*Update NormalV*/
	size_t KeyN1,KeyN2;
	KeyN1 = pKeyVertex.size();
	KeyN2 = o.pKeyVertex.size();
	size_t total = KeyN1 + KeyN2; //_n + o._n;
	float factor = 1.f/(float)total; 
	float factor1 =factor * KeyN1;//_n*factor;
	float factor2 = factor * KeyN2;//o._n*factor;
	NormalV.nx = NormalV.nx*factor1 + o.NormalV.nx*factor2;
	NormalV.ny = NormalV.ny*factor1 + o.NormalV.ny*factor2;
	NormalV.nz = NormalV.nz*factor1 + o.NormalV.nz*factor2;
	NormalizeVector(NormalV);
}
void CPolygonCurve::FuseWithPoint(float& fx,float& fy,float& fz)
{
	size_t KeyN = pKeyVertex.size();
	float factor = 1.0f/(float)(KeyN+1);//(_n+1);
	GravityP.x = GravityP.x*factor*KeyN + fx*factor;
	GravityP.y = GravityP.y*factor*KeyN + fy*factor;
	GravityP.z = GravityP.z*factor*KeyN + fz*factor;
}
void CPolygonCurve::InsertWithPoint(float& fx,float& fy,float& fz,float& R, float& G, float& B)
{
	P3D o;
	o.x = fx; o.y = fy; o.z = fz;
	//	pVertex.push_back(o);
	o.R = R; o.G = G; o.B = B;
	//AdjustPoint(fx,fy,fz);
	//o.x = fx; o.y = fy; o.z = fz;
	pKeyVertex.push_back(o);
	//_n++;
	//FuseWithPoint(fx,fy,fz);
}
bool CPolygonCurve::NeedTofuse(float& fx,float& fy, float& fz, int max_num, double err)
{
	// Now we use keyPoint instead of all relative points
	int keyN = pKeyVertex.size();
	if(max_num > keyN) return false;//_n)	return false;
	int count = 0;

	double t_err =0.0;
	//double m_err = 1000;


	for(size_t i=0;i<pKeyVertex.size();i++)//pVertex.size(); i++)
	{
		t_err = pKeyVertex[i]._dis(fx,fy,fz);
		if(t_err < 1e-3) // this point has already been included in this point
			return true;
		if(err > t_err)
		{
			count ++;
			/*Here we do fuse, just find its nearest point fuse with*/
			if(count >= max_num) return true;
		}
	}
	return false;
}
void CPolygonCurve::AdjustPoint(float& fx, float& fy, float& fz)
{
	/*using plane-function to press this point in surface CPolygonCurve*/
	/* nx(x - xg) + ny(y - yg) + nz(z - zg) = 0*/
	float nx = NormalV.nx;
	float ny = NormalV.ny;
	float nz = NormalV.nz;

	/*Project point onto plane*/

	float k = nx*(GravityP.x - fx) + ny*(GravityP.y - fy) + nz*(GravityP.z - fz);
	fx = fx + nx*k;
	fy = fy + ny*k;
	fz = fz + nz*k;
	return ;
}
void CPolygonCurve::AdjustAllPoints()
{
	for(size_t i=0;i<pKeyVertex.size();i++)
		AdjustPoint(pKeyVertex[i].x,pKeyVertex[i].y,pKeyVertex[i].z);
}

// whether these 
bool CPolygonCurve::IsOverLapped( CPolygonCurve* other)
{
	if(other->m_lower_x> this->m_upper_x || \
		other->m_upper_x < this->m_lower_x)
		return false;
	if(other->m_lower_y> this->m_upper_y || \
		other->m_upper_y < this->m_lower_y)
		return false;
	if(other->m_lower_z> this->m_upper_z || \
		other->m_upper_z < this->m_lower_z)
		return false;
	return true;
}
#define COS5 0.99619469809174553229501040247389
#define COS10 0.98480775301220805936674302458952
#define THRESH_DIS 0.15
bool CPolygonCurve::IsSimilarPlane( CPolygonCurve* other)
{
	//if(fabs(PointMultiply(this->NormalV,other->NormalV)) >= COS5)
	//	return true;

	// if not overlapped
	if( (m_upper_x < other->m_lower_x) || (m_lower_x > other->m_upper_x) || \
		(m_upper_y < other->m_lower_y) || (m_lower_y > other->m_upper_y) || \
 		(m_upper_z < other->m_lower_z) || (m_lower_z > other->m_upper_z) )
	{
		return false;
	}
	if(fabs(PointMultiply(this->NormalV,other->NormalV))<COS10)
		return false;
	double dis=0;
	CalculateDistanceBetweenPlane(other->GravityP,dis);
	if(fabs(dis) > THRESH_DIS)
		return false;
	return true;
}
void CPolygonCurve::AddNewPTtoPlane(pcl::PointXYZRGB& pt)
{
	if(UpdateBoundary(pt.x,pt.y,pt.z))
		m_pColorVertex->points.push_back(pt);

	TotalGP.x+=pt.x; TotalGP.y+=pt.y; TotalGP.z+=pt.z;
	TotalGP.R+=pt.r; TotalGP.G+=pt.g; TotalGP.B+=pt.b;

	m_num++;

	GravityP.x = TotalGP.x/(float)m_num;
	GravityP.y = TotalGP.y/(float)m_num;
	GravityP.z = TotalGP.z/(float)m_num;
	GravityP.R = TotalGP.R/(float)m_num;
	GravityP.G = TotalGP.G/(float)m_num;
	GravityP.B = TotalGP.B/(float)m_num;
}
void CPolygonCurve::AddNewPTtoPlane(pcl::PointXYZRGBNormal& pt)
{
	pcl::PointXYZ sp;
	pcl::PointXYZRGB spc;
	pcl::Normal nm_p;
	spc.x=sp.x=pt.x; 
	spc.y=sp.y=pt.y; 
	spc.z=sp.z=pt.z;
	spc.r=pt.r;
	spc.g=pt.g;
	spc.b=pt.b;
	nm_p.normal_x=pt.normal_x;
	nm_p.normal_y=pt.normal_y;
	nm_p.normal_z=pt.normal_z;
	m_pNormals->points.push_back(nm_p);
	
	if(UpdateBoundary(pt.x,pt.y,pt.z))
	{
		m_pVertex->points.push_back(sp);
		m_pColorVertex->points.push_back(spc);
	}
	
	if(PointMultiply(NormalV,pt)<0)
	{
		pt.normal_x*=-1.f;
		pt.normal_y*=-1.f;
		pt.normal_z*=-1.f;
	}
	TotalNormalV.nx+=pt.normal_x;
	TotalNormalV.ny+=pt.normal_y;
	TotalNormalV.nz+=pt.normal_z;

	TotalGP.x+=sp.x; TotalGP.y+=sp.y; TotalGP.z+=sp.z;
	TotalGP.R+=pt.r; TotalGP.G+=pt.g; TotalGP.B+=pt.b;

	m_num++;
	NormalV.nx = TotalNormalV.nx/(float)m_num;
	NormalV.ny = TotalNormalV.ny/(float)m_num;
	NormalV.nz = TotalNormalV.nz/(float)m_num;
	NormalizeVector(NormalV);

	GravityP.x = TotalGP.x/(float)m_num;
	GravityP.y = TotalGP.y/(float)m_num;
	GravityP.z = TotalGP.z/(float)m_num;
	GravityP.R = TotalGP.R/(float)m_num;
	GravityP.G = TotalGP.G/(float)m_num;
	GravityP.B = TotalGP.B/(float)m_num;
}

#define mysqure(x) ((x)*(x))
void CPolygonCurve::CalculateVar()
{
	size_t N=m_pNormals->points.size();
	float sx=0;
	float sy=0;
	float sz=0;
	double var_sum;
	for(size_t i=0;i<N;i++)
	{
		pcl::Normal& nm_p=m_pNormals->points[i];
		if(PointMultiply(NormalV,nm_p)<0)
		{
			nm_p.normal_x*=-1.f;
			nm_p.normal_y*=-1.f;
			nm_p.normal_z*=-1.f;
		}
		sx=mysqure(nm_p.normal_x-NormalV.nx);
		sy=mysqure(nm_p.normal_y-NormalV.ny);
		sz=mysqure(nm_p.normal_z-NormalV.nz);
		var_sum+=(double)(sx+sy+sz);
	}
	m_var=(var_sum)/(double)(N);
	cout<<"Points in Plane: "<<N;
	cout<<" Variance : "<<m_var<<endl;
}
void CPolygonCurve::MergeWithPlane(CPolygonCurve* other)
{
	/*Update NormalV*/
	size_t KeyN1,KeyN2;
	KeyN1 = this->m_num;
	KeyN2 = other->m_num;
	size_t total = KeyN1 + KeyN2; //_n + o._n;
	float factor = 1.f/(float)total; 
	float factor1 = factor * KeyN1;//_n*factor;
	float factor2 = factor * KeyN2;//o._n*factor;
	NormalV.nx = NormalV.nx*factor1 + other->NormalV.nx*factor2;
	NormalV.ny = NormalV.ny*factor1 + other->NormalV.ny*factor2;
	NormalV.nz = NormalV.nz*factor1 + other->NormalV.nz*factor2;
	NormalizeVector(NormalV);

	GravityP.x = GravityP.x*factor1 + other->GravityP.x*factor2;
	GravityP.x = GravityP.y*factor1 + other->GravityP.y*factor2;
	GravityP.x = GravityP.z*factor1 + other->GravityP.z*factor2;
	GravityP.R = GravityP.R*factor1 + other->GravityP.R*factor2;
	GravityP.G = GravityP.G*factor1 + other->GravityP.G*factor2;
	GravityP.B = GravityP.B*factor1 + other->GravityP.B*factor2;
	
	m_pVertex->points.insert(m_pVertex->points.end(),other->m_pVertex->points.begin(),other->m_pVertex->points.end());

	LOWER_ASS(m_lower_x,other->m_lower_x);
	LOWER_ASS(m_lower_y,other->m_lower_y);
	LOWER_ASS(m_lower_z,other->m_lower_z);

	UPPER_ASS(m_upper_x,other->m_upper_x);
	UPPER_ASS(m_upper_y,other->m_upper_y);
	UPPER_ASS(m_upper_z,other->m_upper_z);

	m_num+=other->m_num;
}
/*
set::erase
iterator erase(iterator it);
iterator erase(iterator first, iterator last);
size_type erase(const Key& key);
The first member function removes the element of the controlled sequence pointed to by it. 
The second member function removes the elements in the range [first, last). 
Both return an iterator that designates the first element remaining beyond any elements removed, or end() if no such element exists.
The third member removes the elements with sort keys in the range [lower_bound(key), upper_bound(key)). 
It returns the number of elements it removes.
*/
void CPolygonCurve::ObtainBoundaryPoints()
{
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator it = m_pVertex->points.begin();
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator begin_to_del;
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::iterator end_to_del;

	int state=0;
	for(;it!=m_pVertex->points.end();)
	{
		if(fabs(it->x-m_lower_x)<=1e-5 || fabs(it->x-m_upper_x)<=1e-5  \
			|| fabs(it->y-m_lower_y)<=1e-5 || fabs(it->y-m_upper_y)<=1e-5 \
			|| fabs(it->z-m_lower_z)<=1e-5 || fabs(it->z-m_upper_z)<=1e-5) // not to delete this point
		{
			if(state==1)
			{
				end_to_del=it;
				it=m_pVertex->points.erase(begin_to_del,end_to_del);
				state=0;
			}
			//else
			it++;
		}
		else // delete this point
		{
			if(state==1) // continue to delete next points
			{
				it++;
				//end_to_del=it;
			}
			else if(state==0) // begin to delete these points
			{
				begin_to_del=it;
				it++;
				//end_to_del=it;
				state=1;
			}
		}
	}
	if(state==1){
		end_to_del=m_pVertex->points.end();
		m_pVertex->points.erase(begin_to_del,end_to_del);
	}
}
void CPolygonCurve::AdjustBoundaryPoints()
{
	for(size_t i=0;i<m_pVertex->points.size();i++)
	{
		pcl::PointXYZ& sp=m_pVertex->points[i];
		AdjustPoint(sp.x,sp.y,sp.z);
	}
}
// Calculate NVs through covariance matrix 
void CPolygonCurve::CalcultePlaneParameters(vector<float>& x, vector<float>&y, vector<float>&z, NV& normalV, P3D& gp)
{
	vector<double> nv;
	nv.resize(3,0);		
	EstimateNormals(x,y,z,gp,nv);
	normalV.nx = nv[0];
	normalV.ny = nv[1];
	normalV.nz = nv[2];
	return ;
}
/*Using covariance matrix to calculate surface parameters Nv(Normal Vector) + Gp(Gravity Point)*/
void CPolygonCurve::EstimateNormals(vector<float>&x, vector<float>&y, vector<float>&z,P3D& centroid, vector<double>& out_N ){

	//P3D centroid;
	CalculateCentroid(x,y,z,centroid);

	// Normal is the eigen vector that corresponds to smallest eigen value
	Eigen::Matrix3f S;
	Eigen::Matrix3f Z;
	Eigen::Vector3f D;
	Eigen::Vector3f V;
	//S.setSize(3,3);

	// Calculate Covariance Matrix
	CalculateCovarianceMatrix(x,y,z,centroid,S);

	EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
	EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
	pcl::eigen33 (S, eigen_vectors, eigen_values);
	out_N.resize(3,0);
	out_N[0] = eigen_vectors(0,0);
	out_N[1] = eigen_vectors(1,0);
	out_N[2] = eigen_vectors(2,0);

	//Eigen::EigenSolver< Eigen::Matrix3f > es(S, true);
	//Z = es.eigenvectors().real(); // Keep only the real part of complex matrix
	//V = es.eigenvalues().real();
	//int i_index=0;
	//for(int i=1;i<4;i++)
	//	if(V[i]>V[i_index])
	//		i_index=i;
	////D = es.eigenvalues().real(); // Keep only the real part of complex matrix

	////Calculate Eigen Values and Vectors
	////S.eigenVectors(Z,D);
	////Z.extractCol(0,out_N); // the first column is NV
	//D = Z.block<3,1>(0,i_index); // The first column is NV
	//out_N.resize(3,0);
	//out_N[0]=D[0]; out_N[1]=D[1]; out_N[2]=D[2];

}
void CPolygonCurve::CalculateCentroid(vector<float>& x, vector<float>&y, vector<float>&z, P3D & centroid){

	//size_t cnt = 0;
	float sx,sy,sz;
	sx = sy = sz = 0;

	size_t N = x.size();
	if(N < 3) {
		cout<<"input points less than three!"<<endl;
		return ;
	}

	for(size_t i=0;i<x.size(); i++)
	{
		sx += x[i];
		sy += y[i];
		sz += z[i];
	}
	float factor = 1.f/(float)(N);
	centroid.x = sx * factor;
	centroid.y = sy * factor;
	centroid.z = sz * factor;
}

void CPolygonCurve::CalculateCovarianceMatrix(vector<float>& x, vector<float>&y, vector<float>&z, P3D & centroid, Eigen::Matrix3f& Cov)
{
	float px,py,pz;
	size_t N  = x.size();
	if(N==0) {
		cout<<"N of points in CalculateCovarianceMatrixvector() is 0!"<<endl;
		return ;
	}
	/*Calculate Covariance matrix*/
	//Cov.setSize(3,3);
	Cov.fill(0);
	for(size_t i=0;i<N;i++)
	{
		px = x[i] - centroid.x;
		py = y[i] - centroid.y;
		pz = z[i] - centroid.z;

		Cov(0,0) += px*px; Cov(0,1) += px*py; Cov(0,2) += px*pz;
		/*Cov(1,0) += px*py;*/ Cov(1,1) += py*py; Cov(1,2) += py*pz;
		/*Cov(2,0) += px*pz; Cov(2,1) += py*pz;*/ Cov(2,2) += pz*pz;
	}
	Cov(1,0) = Cov(0,1);
	Cov(2,0) = Cov(0,2);
	Cov(2,1) = Cov(1,2);
}