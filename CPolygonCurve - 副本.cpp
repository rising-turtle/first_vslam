#include "CPolygonCurve.h"

CPolygonCurve::HD CPolygonCurve::_heading_vector;
/************************************************************************/
/*   CPolygonCurve        Constructor    from three Points                         */
/************************************************************************/
//CPolygonCurve::CPolygonCurve(CPoint3D&a, CPoint3D&b, CPoint3D&c)
//{
//	pVertex.push_back(a);
//	pVertex.push_back(b);
//	pVertex.push_back(c);
//	_n = 3;
//	CalculateVector(a,b,c);
//	CalculateGravity();
//}

/*Constructors*/
CPolygonCurve::CPolygonCurve(CPoint3D& a,CPoint3D& b, CPoint3D&c, vector<float>& R, vector<float>& G, vector<float>&B)
{
	if(R.size()<3 ){
		std::cerr<<"error to Initialize CPolygonCurve less than three points!"<<std::endl;
		exit(-1);
	}
	P3D _a[3];
	_a[0].x = a.m_coords[0]; _a[0].y = a.m_coords[1]; _a[0].z = a.m_coords[2];
	_a[1].x = b.m_coords[0]; _a[1].y = b.m_coords[1]; _a[1].z = b.m_coords[2];
	_a[2].x = c.m_coords[0]; _a[2].y = c.m_coords[1]; _a[2].z = c.m_coords[2];

	for(size_t i=0;i<3;i++)
	{
		_a[i].R = R[i];
		_a[i].G = G[i];
		_a[i].B = B[i];
		pKeyVertex.push_back(_a[i]);
	}
	CalculateVector(_a[0],_a[1],_a[2]);
	CalculateAngle(NormalV);
	CalculateGravity();
	CalculateDisToOrigin();
}
/*Construct plane through Covariance , not finished yet*/
CPolygonCurve::CPolygonCurve(vector<CPoint3D>& points, vector<float>& R,vector<float>& G, vector<float>& B)
{
	vector<float> x,y,z;
	size_t N = points.size();
	float factor = 1.f/(float)N;
	x.resize(N); y.resize(N); z.resize(N);

	//pKeyVertex.resize(N); // save vertexes in this plane
	//Here, do not add those neighbour points as vertexes, only use gravity point 
	//pKeyVertex.resize(1);
	float r,g,b;
	r = g = b = 0;

	for(size_t i=0;i<N;i++)
	{
		//pKeyVertex[i].x = x[i] = points[i].m_coords[0]; pKeyVertex[i].y= y[i] = points[i].m_coords[1]; pKeyVertex[i].z = z[i] = points[i].m_coords[2];
		//pKeyVertex[i].R = R[i]; pKeyVertex[i].G = G[i]; pKeyVertex[i].B = B[i];
		x[i] = points[i].m_coords[0];
		y[i] = points[i].m_coords[1];
		z[i] = points[i].m_coords[2];
		r += R[i]; g+= G[i]; b += B[i];
	}
	CalcultePlaneParameters(x,y,z,NormalV,GravityP); // Using Covariance Matrix to calculate NV and GP
	GravityP.R = r*factor; GravityP.G = g* factor; GravityP.B = b*factor;// record color gravity point
	pKeyVertex.push_back(GravityP); // only use gravity point as vertex

	NormalizeVector(NormalV); // Normalization 
	CalculateAngle(NormalV); // Calculate cos() with Heading_Vector
	CalculateDisToOrigin(); // Calculate dis to original coordinate
}
CPolygonCurve::CPolygonCurve(std::vector<float>&x, std::vector<float>&y, std::vector<float>&z,CPoint3D& key, float R, float G, float B){
	if(x.size()<3)
	{
		std::cerr<<"error to Initialize CPolygonCurve less than three points!"<<std::endl;
		exit(-1);
	}
	P3D a[3];
	for(size_t i=0;i<3;i++)
	{
		a[i].x = x[i];
		a[i].y = y[i];
		a[i].z = z[i];
		//	pVertex.push_back(a[i]);
	}
	P3D keyPoint;
	GravityP.x = keyPoint.x = key.m_coords[0];
	GravityP.y = keyPoint.y = key.m_coords[1];
	GravityP.z = keyPoint.z = key.m_coords[2];
	keyPoint.R = R;
	keyPoint.G = G;
	keyPoint.B = B;
	pKeyVertex.push_back(keyPoint);
	_n = 3;

	CalculateVector(a[0],a[1],a[2]);

	//CalculateGravity();
	//Make key_point as the gravity_point

	CalculateAngle(NormalV);

	/*Calculate distance from original point*/
	CalculateDisToOrigin();
	/*Adjust to its plane*/

}

CPolygonCurve::CPolygonCurve(){};// Yet not finish this constructor
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
	CMatrixD S;
	CMatrixD Z, D;
	S.setSize(3,3);

	// Calculate Covariance Matrix
	CalculateCovarianceMatrix(x,y,z,centroid,S);

	//Calculate Eigen Values and Vectors
	S.eigenVectors(Z,D);
	Z.extractCol(0,out_N); // the first column is NVs
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

void CPolygonCurve::CalculateCovarianceMatrix(vector<float>& x, vector<float>&y, vector<float>&z, P3D & centroid, CMatrixD& Cov)
{
	float px,py,pz;
	size_t N  = x.size();
	if(N==0) {
		cout<<"N of points in CalculateCovarianceMatrixvector() is 0!"<<endl;
		return ;
	}
	/*Calculate Covariance matrix*/
	Cov.setSize(3,3);
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
	GravityP.x = GravityP.x * factor* KeyN+ fx*factor;
	GravityP.y = GravityP.y* factor*KeyN + fy*factor;
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