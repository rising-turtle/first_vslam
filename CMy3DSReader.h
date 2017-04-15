#ifndef CMY3DSREADER_H
#define CMY3DSREADER_H
#include "3ds.h"

class CMy3DSReader{
public:
	CMy3DSReader(){}
	~CMy3DSReader(){}
	
	void GetDirect(const char* file);
	bool Load(const char *file);
	STRING_3DS m_direct;
	FILE *m_pfile;
	FILE *m_outfile;

	vector<GROUPV> m_group;
	void Render();
	float GetScale();

	BYTE ReadByte();
	WORD ReadWord();
	CHUNK_3DS ReadChunk();
	UINT ReadUint();
};

#endif