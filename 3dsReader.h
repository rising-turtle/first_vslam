#ifndef T3DSREADER_H
#define T3DSREADER_H
#include "3ds.h"
#include <string>
using std::string;

class C3DSReader{
private:
	STRING_3DS direct;
	FILE *pfile;

	BYTE ReadByte();
	WORD ReadWord();
	UINT ReadUint();
	float ReadFloat();
	UINT ReadString(STRING_3DS &string);
	UINT ReadBRGB(BYTE &red,BYTE &green,BYTE &blue);
	UINT ReadFRGB(float &red,float &green,float &blue);
	UINT ReadBPER(BYTE &per);
	UINT ReadFPER(float &per);

	CHUNK_3DS ReadChunk();
	//
	UINT ReadPrimary(UINT n);
	//
	UINT ReadEdit(UINT n);
	UINT ReadMesh(UINT n);
	UINT ReadMeshInfo(UINT n);
	UINT ReadFacetInfo(UINT n);
	//
	UINT ReadMaterial(UINT n);
	UINT ReadMatDif(UINT n);
	UINT ReadMatMap(UINT n);
	//
	UINT ReadKeyframe(UINT n);
	UINT ReadKeyMesh(UINT n);
	//
	void GetDirect(const char *str);
protected:
	virtual void UserKeyframeID(WORD id);
	virtual void UserKeyframeTrackScl(WORD frame,float x,float y,float z);
	virtual void UserKeyframeTrackRot(WORD frame,float angle,float x,float y,float z);
	virtual void UserKeyframeTrackPos(WORD frame,float x,float y,float z);
	virtual void UserKeyframePivot(float x,float y,float z);
	virtual void UserKeyframeName(const char *name);
	virtual void UserKeyframeParent(WORD id);
	virtual void UserKeyframeRange(UINT start,UINT end);

	virtual void UserMaterialMapName(const char *name);
	virtual void UserMaterialDiffuseGamma(BYTE red,BYTE green,BYTE blue);
	virtual void UserMaterialDiffuse(BYTE red,BYTE green,BYTE blue);
	virtual void UserMaterialName(const char *name);

	virtual void UserMeshMaterialFacet(WORD in);
	virtual void UserMeshMaterialName(const char *name);
	virtual void UserMeshFacet(WORD id1,WORD id2,WORD id3);
	virtual void UserMeshLocalO(float x,float y,float z);
	virtual void UserMeshLocalN(float x,float y,float z);
	virtual void UserMeshLocalV(float x,float y,float z);
	virtual void UserMeshLocalU(float x,float y,float z);
	virtual void UserMeshTexCoord(float u,float v);
	virtual void UserMeshVertex(float x,float y,float z);
	virtual void UserMeshName(const char *name);
public:
	bool Load(const char *file);
	C3DSReader();
	~C3DSReader();
};



#endif