#include "CMy3DSWriter.h"
#include <iostream>
#include "TriangleMesh.h"
#include "CMy3DSReader.h"

using namespace std;

CMy3DSWriter::~CMy3DSWriter(){
	Reset();
}

void CMy3DSWriter::Reset()
{
	if(m_pContent!=NULL)
	{	
		delete[] m_pContent;
	}
	m_pContent=NULL;
}
// Here we only want to record Vertex, Facets info
// If you want to record more info about materials, key frame, mapcoord,
// you need to overlapped this function to push these info into m_pContent
void CMy3DSWriter::UserPrepareContent(void *pContent)
{
	//BYTE* pTmpCont=m_pContent;
	UINT totallen=2*6; // PRIM chunk && EDIT chunk

	size_t N_GROUP=m_group.size();
	L_EDIT_MESH.resize(N_GROUP);
	L_MESH_VERTEX.resize(N_GROUP);
	L_MESH_FACET.resize(N_GROUP);
	L_MESH_INFO.resize(N_GROUP);

	for(size_t i=0;i<N_GROUP;i++)
	{
		WORD N_OF_Vertex=m_group[i].vertices.size();
		// Length for MESH_VERTEX
		L_MESH_VERTEX[i]=2+6+N_OF_Vertex*3*sizeof(float);
		
		WORD N_OF_Facets=m_group[i].facets.size();
		// Length for MESH_FACET
		L_MESH_FACET[i]=2+6+N_OF_Facets*4*sizeof(WORD);

		// Length for MESH_INFO
		L_MESH_INFO[i]=6+L_MESH_FACET[i]+L_MESH_VERTEX[i];

		// Length for EDIT_MESH
		L_EDIT_MESH[i]=6+L_MESH_INFO[i]+m_group[i].name.size()+1;
		
		totallen+=L_EDIT_MESH[i];
	}
	m_TotalLen=totallen;

	//BYTE* pTmpCont=m_pContent;
	//WORD chunkid;
	//WORD chunklen;
	//// Primary chunk
	//chunkid=0x4D4D;
	//chunklen=totallen;
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=6;

	//// Edit Chunk
	//chunkid=0x3D3D;
	//chunklen-=6;
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=6;

	//// Edit Mesh
	//chunkid=0x4000;
	//chunklen-=6;
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=6;

	//// Mesh Name
	//memcpy(pTmpCont,m_group.name.c_str(),m_group.name.size());
	//pTmpCont+=m_group.name.size();

	//// Mesh info
	//chunkid=0x4100;
	//chunklen-=m_group.name.size();
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=6;

	//// Vertexes 
	//chunkid=0x4110;
	//chunklen-=6;
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=6;
	//WORD N_of_Vertexs=m_group.vertices.size();
	//memcpy(pTmpCont,&N_of_Vertexs,sizeof(WORD));
	//pTmpCont+=2;
	//for(WORD i=0;i<N_of_Vertexs;i++)
	//{
	//	memcpy(pTmpCont,m_group.vertices[i].a,3*sizeof(float));
	//	pTmpCont+=3*sizeof(float);
	//}

	//// Facets
	//chunkid=0x4120;
	//chunklen-=3*N_of_Vertexs*sizeof(float);
	//WORD N_of_Facets=m_group.facets.size();
	//
	//memcpy(pTmpCont,&chunkid,sizeof(WORD));
	//chunklen=N_of_Facets*4*sizeof(WORD);
	//memcpy(pTmpCont+2,&chunklen,sizeof(UINT));
	//pTmpCont+=4;
	//WORD flag=0;
	//for(WORD i=0;i<N_of_Facets;i++)
	//{	
	//	memcpy(pTmpCont,m_group.facets[i].a,3*sizeof(WORD));
	//	memcpy(pTmpCont,&flag,sizeof(WORD));
	//	pTmpCont+=8;
	//}
	//}
}

bool CMy3DSWriter::Create(const char* file)
{
	m_pfile=fopen(file,"wb");
	if(m_pfile==NULL)
		return false;
	// Write PRIM
	CHUNK_3DS pri_chunk;
	pri_chunk.ID=PRIM;
	pri_chunk.length=m_TotalLen;
	WriteChunk(pri_chunk);

	// Write PRIM_EDIT
	CHUNK_3DS edit_chunk;
	edit_chunk.ID=PRIM_EDIT;
	edit_chunk.length=m_TotalLen-6;
	WriteChunk(edit_chunk);
	for(size_t i=0;i<m_group.size();i++)
	{
		// Write EDIT_MESH
		CHUNK_3DS edit_mesh;
		edit_mesh.ID=EDIT_MESH;
		edit_mesh.length=L_EDIT_MESH[i];
		WriteChunk(edit_mesh);

		// Write MESH_NAME
		STRING_3DS mesh_name;
		UINT L_Str=m_group[i].name.size();
		strncpy(mesh_name.string,m_group[i].name.c_str(),L_Str);
		if(L_Str>=128) L_Str=127;
		mesh_name.string[L_Str]='\0';
		WriteString(mesh_name);
		
		// Write MESH_INFO
		CHUNK_3DS mesh_info;
		mesh_info.ID=MESH_INFO;
		mesh_info.length=L_MESH_INFO[i];
		WriteChunk(mesh_info);
		
		// Write MESH_VERTEX
		CHUNK_3DS mesh_vertex;
		mesh_vertex.ID=MESH_VERTEX;
		mesh_vertex.length=L_MESH_VERTEX[i];
		WriteChunk(mesh_vertex);
		
		WORD N_OF_Vertex=m_group[i].vertices.size();
		WriteWord(N_OF_Vertex);
		// Coordinates translation between Robot system and C3DS system
		// rx=cx; ry=-cz; rz=cy;
		for(WORD j=0;j<N_OF_Vertex;j++)
		{
			float cx=m_group[i].vertices[j].u;
			float cy=m_group[i].vertices[j].n;
			float cz=-m_group[i].vertices[j].v;
			WriteFloat(cx);
			WriteFloat(cy);
			WriteFloat(cz);
		}
		// Write MESH_FACETS
		CHUNK_3DS mesh_facet;
		mesh_facet.ID=MESH_FACET;
		mesh_facet.length=L_MESH_FACET[i];
		WriteChunk(mesh_facet);

		WORD N_OF_Facets=m_group[i].facets.size();
		WriteWord(N_OF_Facets);
		for(WORD j=0;j<N_OF_Facets;j++)
		{
			WriteWord(m_group[i].facets[j].u);
			WriteWord(m_group[i].facets[j].v);
			WriteWord(m_group[i].facets[j].n);
			WriteWord(0); // flag not used
		}
	}
	fclose(m_pfile);
	return true;
}

void CMy3DSWriter::UserPrepareContentCMyReader(void* pContent)
{
	CMy3DSReader* pReader=static_cast<CMy3DSReader*>(pContent);
	m_group.clear();
	m_group.resize(pReader->m_group.size());
	for(size_t i=0;i<pReader->m_group.size();i++)
	{
		m_group[i]=pReader->m_group[i];
	}
	UserPrepareContent();
}

// Specifically for CTriangleMesh
void CMy3DSWriter::UserPrepareContentTriangleMesh(void* pContent)
{
	CTriangleMesh* pTriMesh=static_cast<CTriangleMesh*>(pContent);
	INDEX tmpTriIndex;
	GROUPV tmpGroup;// deem all the one Triangle Mesh!
	// dump point cloud and triangle index into m_group
	for(size_t i=0;i<pTriMesh->m_pTriSet->m_vtindex.size();i++){
		boost::shared_ptr<TriangleIndex>& rTriIndex=pTriMesh->m_pTriSet->m_vtindex[i];
		for(int j=0;j<3;j++) 
			tmpTriIndex.a[j]=rTriIndex->m_tindex[j];
		tmpGroup.facets.push_back(tmpTriIndex);
	}
	VECTOR tmpVertex;
	for(size_t i=0;i<pTriMesh->m_pVertex->points.size();i++){
		pcl::PointXYZRGB& sp=pTriMesh->m_pVertex->points[i];
		tmpVertex.a[0]=sp.x; tmpVertex.a[1]=sp.y; tmpVertex.a[2]=sp.z;
		tmpGroup.vertices.push_back(tmpVertex);
	}
	static int NameIndex=1;
	tmpGroup.name.assign("Mesh %d",++NameIndex);
	m_group.push_back(tmpGroup);

	UserPrepareContent();
}
bool CMy3DSWriter::dumpTriangleMesh(const char* file)
{
	m_pfile=fopen(file,"wb");
	if(m_pfile==NULL)
		return false;
	// Write PRIM
	CHUNK_3DS pri_chunk;
	pri_chunk.ID=PRIM;
	pri_chunk.length=m_TotalLen;
	WriteChunk(pri_chunk);

	// Write PRIM_EDIT
	CHUNK_3DS edit_chunk;
	edit_chunk.ID=PRIM_EDIT;
	edit_chunk.length=m_TotalLen-6;
	WriteChunk(edit_chunk);
	for(size_t i=0;i<m_group.size();i++)
	{
		// Write EDIT_MESH
		CHUNK_3DS edit_mesh;
		edit_mesh.ID=EDIT_MESH;
		edit_mesh.length=L_EDIT_MESH[i];
		WriteChunk(edit_mesh);

		// Write MESH_NAME
		STRING_3DS mesh_name;
		UINT L_Str=m_group[i].name.size();
		strncpy(mesh_name.string,m_group[i].name.c_str(),L_Str);
		if(L_Str>=128) L_Str=127;
		mesh_name.string[L_Str]='\0';
		WriteString(mesh_name);

		// Write MESH_INFO
		CHUNK_3DS mesh_info;
		mesh_info.ID=MESH_INFO;
		mesh_info.length=L_MESH_INFO[i];
		WriteChunk(mesh_info);

		// Write MESH_VERTEX
		CHUNK_3DS mesh_vertex;
		mesh_vertex.ID=MESH_VERTEX;
		mesh_vertex.length=L_MESH_VERTEX[i];
		WriteChunk(mesh_vertex);

		WORD N_OF_Vertex=m_group[i].vertices.size();
		WriteWord(N_OF_Vertex);
		// Coordinates translation between Robot system and C3DS system
		// rx=cx; ry=-cz; rz=cy;
		for(WORD j=0;j<N_OF_Vertex;j++)
		{
			float cx=m_group[i].vertices[j].u;
			float cy=m_group[i].vertices[j].n;
			float cz=-m_group[i].vertices[j].v;
			WriteFloat(cx);
			WriteFloat(cy);
			WriteFloat(cz);
		}
		// Write MESH_FACETS
		CHUNK_3DS mesh_facet;
		mesh_facet.ID=MESH_FACET;
		mesh_facet.length=L_MESH_FACET[i];
		WriteChunk(mesh_facet);

		WORD N_OF_Facets=m_group[i].facets.size();
		WriteWord(N_OF_Facets);
		for(WORD j=0;j<N_OF_Facets;j++)
		{
			WriteWord(m_group[i].facets[j].u);
			WriteWord(m_group[i].facets[j].v);
			WriteWord(m_group[i].facets[j].n);
			WriteWord(0); // flag not used
		}
	}
	fclose(m_pfile);
	return true;
}