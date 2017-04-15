#ifndef TDSWRITER_H
#define TDSWRITER_H
#include "3ds.h"

//#ifndef UNIT
//#define UNIT unsigned int
//#endif
class C3DSWriter
{
public:
	STRING_3DS m_direct;
	FILE* m_pfile;
	BYTE* m_pContent;
	bool WriteByte(BYTE);
	bool WriteWord(WORD);
	bool WriteUint(UINT);
	bool WriteFloat(float);
	bool WriteString(STRING_3DS& str);
	bool WriteBRGB(BYTE r,BYTE g, BYTE b);
	bool WriteFRGB(float r,float g, float b);
	bool WriteBPER(BYTE per);
	bool WriteFPER(float per);

	void WriteChunk(WORD Id, UINT Len);
	void WriteChunk(CHUNK_3DS chunk);
//
	bool WritePrimary(BYTE* pPriCont,UINT n);
		//
		bool WriteEdit(BYTE* pPriCont,UINT n);
			bool WriteMesh(BYTE* pPriCont,UINT n);
				bool WriteMeshInfo(BYTE* pPriCont,UINT n);
					bool WriteFacetInfo(BYTE* pPriCont,UINT n);
		//
	/*	bool WriteMaterial(UINT n);
			bool WriteMatDif(UINT n);
			bool WriteMatMap(UINT n);*/

		//
	/*	bool WriteKeyFrame(UINT n);
			bool WriteKeyMesh(UINT n);*/
	void GetDirect(const char* str);
protected:
	/*virtual void UserKeyframeID(WORD id);
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
	virtual void UserMeshName(const char *name);*/
	virtual void UserPrepareContent(void* pContent)=0;
public:
	virtual bool Create(const char* file);
	C3DSWriter(){m_pContent=NULL;}
	virtual ~C3DSWriter(){}
};


#endif


