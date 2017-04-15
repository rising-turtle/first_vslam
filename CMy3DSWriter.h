#ifndef CMY3DSWRITER_H
#define CMY3DSWRITER_H
#include "3dsWriter.h"
#include <string>
#include <vector>
using std::string;
using std::vector;

class CMy3DSWriter: public C3DSWriter
{
	using C3DSWriter::m_pContent;
public:
	CMy3DSWriter(){}
	~CMy3DSWriter();

	vector<GROUPV> m_group;
	vector<UINT> L_MESH_VERTEX;
	vector<UINT> L_MESH_FACET;
	vector<UINT> L_EDIT_MESH;
	vector<UINT> L_MESH_INFO;
	UINT m_TotalLen;

	// This function has to be defined in subclass
	void UserPrepareContent(void* pContent=NULL);
	void Reset();
	bool Create(const char* file);

	// Specifically for CTriangleMesh
	void UserPrepareContentTriangleMesh(void* pContent);
	bool dumpTriangleMesh(const char* file);

	// Specifically for CMy3DSReader
	void UserPrepareContentCMyReader(void* pContent);
};


#endif