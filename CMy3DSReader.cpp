#include "CMy3DSReader.h"
#include "iostream" 
#include "glut.h"
#include <fstream>

using namespace std;

void CMy3DSReader::GetDirect(const char* file)
{
	const char* pfile=file;
	int fn=strlen(pfile);
	while(pfile[--fn]!='\\'){}

	strncpy(m_direct.string,pfile,fn+1);
	m_direct.string[fn]='\0';
}

BYTE CMy3DSReader::ReadByte()
{
	BYTE ret;
	fread(&ret,1,1,m_pfile);
	return ret;
}
WORD CMy3DSReader::ReadWord()
{
	return ReadByte()+(ReadByte()<<8);
}
CHUNK_3DS CMy3DSReader::ReadChunk()
{
	CHUNK_3DS ret;
	ret.ID=ReadWord();
	ret.length=ReadUint();
	return ret;
}
UINT CMy3DSReader::ReadUint()
{
	return ReadWord()+(ReadWord()<<16);
}
void CMy3DSReader::Render()
{
	for(size_t i=0;i<m_group.size();i++)
	{
		glBegin(GL_TRIANGLES);
		for(size_t j=0;j<m_group[i].facets.size();j++)
		{
			glVertex3f(m_group[i].vertices[m_group[i].facets[j].u].u,m_group[i].vertices[m_group[i].facets[j].u].v,m_group[i].vertices[m_group[i].facets[j].u].n);
			glVertex3f(m_group[i].vertices[m_group[i].facets[j].v].u,m_group[i].vertices[m_group[i].facets[j].v].v,m_group[i].vertices[m_group[i].facets[j].v].n);
			glVertex3f(m_group[i].vertices[m_group[i].facets[j].n].u,m_group[i].vertices[m_group[i].facets[j].n].v,m_group[i].vertices[m_group[i].facets[j].n].n);
		}
		glEnd();
	}
}

float CMy3DSReader::GetScale()
{
	VECTOR p;
	INDEX id;
	float max=0;
	float min=0;
	int n=m_group.size();
	int m;
	for(int i=0;i<n;i++){
		m=m_group[i].facets.size();
		for(int j=0;j<m;j++){
			id=m_group[i].facets[j];
			for(int k=0;k<3;k++){
				p=m_group[i].vertices[id.a[k]];
				max=(p.u>max)?p.u:max;
				max=(p.v>max)?p.v:max;
				max=(p.n>max)?p.n:max;
				min=(p.u<min)?p.u:min;
				min=(p.v<min)?p.v:min;
				min=(p.n<min)?p.n:min;
			}
		}
	}
	return max-min;
}
bool CMy3DSReader::Load(const char *file)
{
	// open 3ds file
	m_pfile=fopen(file,"rb");
	if(m_pfile==NULL)
	{
		cout<<"Failed to open file: "<<string(file)<<endl;	
		return false;
	}

	// make up outfile
	string outfile(file);
	UINT pos=outfile.find_last_of('.');
	if(pos==string::npos)
	{
		cout<<"this file does not contain '.'"<<endl;
		return false;
	}
	outfile.erase(pos+1,3);
	outfile.append("txt");
	m_outfile=fopen(outfile.c_str(),"wb");

	ofstream m_outfile1(outfile.c_str());

	if(m_outfile==NULL)
	{
		cout<<"failed to open file: "<<outfile<<endl;
		return false;
	}
	// output each chunk info
	CHUNK_3DS pChunk;
	pChunk=ReadChunk();
	if(pChunk.ID!=0x4D4D)
	{
		cout<<"Not .3ds file!"<<endl;
		return false;
	}

	m_outfile1<<"Primary Chunk: "<<hex<<pChunk.ID<<endl;
	m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;

	UINT count=0;
	UINT ALLN=pChunk.length-6;
	string tmpstr;
	GROUPV group;
	while(count<ALLN)
	{
		pChunk=ReadChunk();
		if(m_group.size()==13)
		{
			int debug=0;
		}
		switch(pChunk.ID)
		{
		case PRIM_EDIT:
			m_outfile1<<"Prim_Edit: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			count+=6;
			//fseek(m_pfile,6,SEEK_CUR);
			break;
		case EDIT_MESH:
			m_outfile1<<"EDIT_MESH: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			count+=6;
			//fseek(m_pfile,6,SEEK_CUR);
			STRING_3DS str;
			BYTE c;
			UINT strl;
			strl=0;
			while(1){
				fread(&c,1,1,m_pfile);
				str.string[strl++]=c;
				if(c=='\0')
					break;
			}
			tmpstr.assign(str.string);
			m_outfile1<<"Mesh Name: "<<tmpstr<<endl;
			// record name of this mesh
			group.name=str.string;
			m_group.push_back(group);
			count+=strl;
			break;
		case MESH_INFO:
			m_outfile1<<"MESH_INFO: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			count+=6;
			break;
		case MESH_VERTEX:
			m_outfile1<<"MESH VERTEX: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			WORD N_Of_Vertex;
			N_Of_Vertex=0;
			fread(&N_Of_Vertex,sizeof(WORD),1,m_pfile);
			m_outfile1<<"N Of Vertex: "<<dec<<N_Of_Vertex<<endl;
			
			// Save Vertexes info in m_Group
			size_t t;
			t=m_group.size();
			if(t>0)
			{
				m_group[t-1].vertices.clear();
				m_group[t-1].vertices.resize(N_Of_Vertex);
			}
			float vx,vy,vz;
			for(WORD i=0;i<N_Of_Vertex;i++)
			{
				fread(&vx,sizeof(float),1,m_pfile);
				fread(&vy,sizeof(float),1,m_pfile);
				fread(&vz,sizeof(float),1,m_pfile);
				size_t t = m_group.size();
				VECTOR v;
				v.u=vx;
				v.v=vz;
				v.n=-vy;
				if(t>0)
					m_group[t-1].vertices[i]=v;
			}
			//fseek(m_pfile,pChunk.length-8,SEEK_CUR);
			count+=pChunk.length;
			break;
		case MESH_FACET:
			m_outfile1<<"MESH FACET: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			WORD N_Of_Facets;
			N_Of_Facets=0;
			fread(&N_Of_Facets,sizeof(WORD),1,m_pfile);
			m_outfile1<<"N Of Facets: "<<dec<<N_Of_Facets<<endl;

			//size_t t;
			t=m_group.size();
			m_group[t-1].facets.clear();
			m_group[t-1].facets.resize(N_Of_Facets);
			WORD id1,id2,id3;
			for(WORD i=0;i<N_Of_Facets;i++)
			{
				INDEX v;
				v.u=ReadWord();
				v.v=ReadWord();
				v.n=ReadWord();
				m_group[t-1].facets[i]=v;
				ReadWord();
			}
			//fseek(m_pfile,pChunk.length-8,SEEK_CUR);
			count+=pChunk.length;
			break;
		default:
			m_outfile1<<"UnRecognized Chunk: "<<hex<<pChunk.ID<<endl;
			m_outfile1<<"Len: "<<dec<<pChunk.length<<endl;
			fseek(m_pfile,pChunk.length-6,SEEK_CUR);
			count+=pChunk.length;
			break;
		}
	}
	return true;
}