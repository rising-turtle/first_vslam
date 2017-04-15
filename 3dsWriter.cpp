#include "3dsWriter.h"
#include "iostream"

using namespace std;

void C3DSWriter::GetDirect(const char *str)
{
	int n=strlen(str);
	while(--n>=0)
		if(str[n]=='\\') break;
	strncpy(m_direct.string,str,n+1);
	m_direct.string[n+1]='\0';
}
bool C3DSWriter::Create(const char *file)
{
	if((m_pfile=fopen(file,"wb"))==NULL) return false;
	GetDirect(file);

	BYTE* ptempContent=m_pContent;
	// Write Primary Chunk
	CHUNK_3DS* chunk=(CHUNK_3DS*)ptempContent;
	if(chunk->ID!=PRIM)
	{
		fclose(m_pfile);
		cout<<"error in m_pContent at Primary Chunk!"<<endl;
		return false;
	}
	ptempContent+=6;
	bool ret=WritePrimary(ptempContent,chunk->length-6);
	fclose(m_pfile);
	return ret;
}

bool C3DSWriter::WriteByte(BYTE m)
{
	if(fwrite(&m,1,1,m_pfile)==1)
		return true;
	return false;
}

bool C3DSWriter::WriteWord(WORD w)
{
	BYTE um=(w&0xFF00)>>8;
	BYTE lm=w&0x00FF;
	return (WriteByte(lm) && WriteByte(um));
}

bool C3DSWriter::WriteUint(UINT u)
{
	WORD uw=(u&0xFFFF0000)>>16;
	WORD lw=u&0x0000FFFF;
	return (WriteWord(lw) && WriteWord(uw));
}

bool C3DSWriter::WriteFloat(float f)
{
	if((fwrite(&f,sizeof(float),1,m_pfile))!=sizeof(float))
		return false;
	return true;
}
bool C3DSWriter::WriteString(STRING_3DS& str)
{
	int i=0;
	while(str.string[i]!='\0')
	{
		if(WriteByte(str.string[i])==false)
			return false;
		i++;
	}
	WriteByte(str.string[i]);
	return true;
}
bool C3DSWriter::WriteBRGB(BYTE r,BYTE g, BYTE b)
{
	WORD chunkid=0X0011;
	UINT chunklen=6+3*sizeof(BYTE);
	//fseek(m_pfile,6,SEEK_CUR);
	WriteWord(chunkid);
	WriteUint(chunklen);
	WriteByte(r);
	WriteByte(g);
	WriteByte(b);
	return true;
}
bool C3DSWriter::WriteBPER(BYTE per)
{
	WORD chunkid=0x0030;
	UINT chunklen=6+sizeof(BYTE);
	WriteWord(chunkid);
	WriteUint(chunklen);
	WriteByte(per);
	return true;
}
bool C3DSWriter::WriteFRGB(float r,float g, float b)
{
	WORD chunkid=0X0013;
	UINT chunklen=6+3*sizeof(float);
	//fseek(m_pfile,6,SEEK_CUR);
	WriteWord(chunkid);
	WriteUint(chunklen);
	WriteFloat(r);
	WriteFloat(g);
	WriteFloat(b);
	return true;
}

bool C3DSWriter::WriteFPER(float per)
{
	WORD chunkid=0x0031;
	UINT chunklen=6+sizeof(float);
	WriteWord(chunkid);
	WriteUint(chunklen);
	WriteFloat(per);
	return true;
}

void C3DSWriter::WriteChunk(WORD Id,UINT Len)
{
	WriteWord(Id);
	WriteUint(Len);
}
void C3DSWriter::WriteChunk(CHUNK_3DS chunk)
{
	WriteChunk(chunk.ID,chunk.length);
}
bool C3DSWriter::WritePrimary(BYTE* pPriCont,UINT n)
{
	UINT count=0;
	BYTE* pCurCont=pPriCont;

	while(count<n)
	{
		CHUNK_3DS* pCurChunk=(CHUNK_3DS*) pCurCont;
		switch(pCurChunk->ID)
		{
		case PRIM_EDIT:
			pCurCont+=6;
			WriteEdit(pCurCont,pCurChunk->length-6);
			count+=pCurChunk->length;
			pCurCont=pCurCont+pCurChunk->length-6;
			break;
		case PRIM_KEY:
			/*pCurCont+=6;
			WriteKeyFrame(pCurCont);
			count+=pCurChunk->length;
			pCurCont=pCurCont+pCurChunk->length-6;
			break;*/
		default:
			// This is not correct if considering all chunks
			count+=pCurChunk->length;
			//WriteChunk(pCurChunk->ID,pCurChunk->length);
			pCurCont+=pCurChunk->length;
			break;
		}

	}
	return true;
}

bool C3DSWriter::WriteEdit(BYTE* pCont,UINT n)
{
	BYTE* pCurCont=pCont;
	UINT count=0;
	UINT NextLen=0;
	while(count<n)
	{
		CHUNK_3DS* pCurChunk=(CHUNK_3DS*)pCurCont;
		switch(pCurChunk->ID)
		{
		case EDIT_MESH:
			pCurCont+=6;
			WriteMesh(pCurCont,pCurChunk->length-6);
			count+=pCurChunk->length;
			pCurCont=pCurCont+pCurChunk->length-6;
			break;
		case EDIT_MAT:
			pCurCont+=6;
			NextLen=pCurChunk->length-6;
//			WriteMaterial(pCurCont,NextLen);
			pCurCont+=NextLen;
			count+=pCurChunk->length;
			break;
		default:
			count+=pCurChunk->length;
			pCurCont+=pCurChunk->length;
			break;
		}
	}
	return true;
}

bool C3DSWriter::WriteMesh(BYTE* pCont,UINT n)
{
	UINT count=0;
	BYTE* pCurCont=pCont;
	UINT NextLen=0;
	while(count<n)
	{
		CHUNK_3DS* pCurChunk=(CHUNK_3DS*)pCurCont;
		switch(pCurChunk->ID)
		{
		case MESH_INFO:
			pCurCont+=6;
			NextLen=pCurChunk->length-6;
			WriteMeshInfo(pCurCont,NextLen);
			pCurCont+=NextLen;
			count+=pCurChunk->length;
			break;
		default:
			count+=pCurChunk->length;
			pCurCont+=pCurChunk->length;
			break;
		}
	}
	return true;
}

bool C3DSWriter::WriteMeshInfo(BYTE* pCont,UINT n)
{
	UINT count=0;
	UINT NextLen=0;
	float* f1,*f2,*f3;
	WORD* t;
	BYTE* pCurCont=pCont;
	while(count<n)
	{
		CHUNK_3DS* pCurChunk=(CHUNK_3DS*)pCurCont;
		switch(pCurChunk->ID)
		{
		case MESH_VERTEX:
			pCurCont+=6;
			t=(WORD*)pCurCont;
			pCurCont+=2;
			while(*t>0)
			{
				f1=(float*)pCurCont;
				f2=(float*)(pCurCont+4);
				f3=(float*)(pCurCont+8);
				pCurCont+=12;
				t--;
			}
			count+=pCurChunk->length;
			break;
		case MESH_FACET:
			NextLen=pCurChunk->length-6;
			pCurCont+=6;
			WriteFacetInfo(pCurCont,NextLen);
			pCurCont+=NextLen;
			count+=pCurChunk->length;
			break;
		case MESH_LOCAL:
			// NOT Cared right now
		case MESH_MAPCOORD:
		default:
			pCurCont+=pCurChunk->length;
			count+=pCurChunk->length;
			break;
		}
	}
	return true;
}
bool C3DSWriter::WriteFacetInfo(BYTE* pCont,UINT n)
{
	UINT count=0;
	BYTE* pCurCont=pCont;
	CHUNK_3DS* pCurChunk;
	WORD* w1,*w2,*w3;
	WORD* t;
	// number of facets in this mesh
	t=(WORD*)pCurCont;
	WriteWord(*t);
	pCurCont+=2;
	while(t>0)
	{
		w1=(WORD*)pCurCont;
		WriteWord(*w1);
		w2=(WORD*)pCurCont;
		WriteWord(*w2);
		w3=(WORD*)pCurCont;
		WriteWord(*w3);
		WriteWord((WORD)0);// flag, not used
		count+=8;
		pCurCont+=8;
		t--;
	}
	while(count<n)
	{
		pCurChunk=(CHUNK_3DS*)pCurCont;
		count+=pCurChunk->length;
		pCurCont+=pCurChunk->length;
	}
	return true;
}
//bool C3DSWriter::WriteMaterial(BYTE* pCont,UINT n)
//{
//	return true;
//}

