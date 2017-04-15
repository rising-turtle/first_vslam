#ifndef _3DS
#define _3DS

#include <string>
#include <vector>
using std::string;
using std::vector;

#define  BYTE unsigned char
#define  WORD unsigned short 
#define  UINT unsigned int
//begin primary chunk
const WORD PRIM =0x4D4D;
	//begin edit chunk
	const WORD PRIM_EDIT =0x3D3D;
		//begin mesh chunk
		 const WORD EDIT_MESH =0x4000;
			 const WORD MESH_INFO =0x4100;
				 const WORD MESH_VERTEX =0x4110;
				 const WORD MESH_FACET  =0x4120;
					 const WORD FACET_MAT=0x4130;
					 const WORD FACET_SMOOTH =0x4150;
				 const WORD MESH_MAPCOORD =0x4140;
				 const WORD MESH_LOCAL    =0x4160;
		//end mesh chunk

		//begin material chunk
		const WORD EDIT_MAT =0xafff;
			const WORD MAT_NAME =0xa000;
			const WORD MAT_AMB  =0xa010;
			const WORD MAT_DIF  =0xa020;
			const WORD MAT_SPE  =0xa030;
			const WORD MAT_MAP  =0xa200;
				const WORD MAP_NAME =0xa300;
		//end material chunk

	//end edit chunk

	//begin keyframe chunk
	const WORD PRIM_KEY =0xB000;
		 const WORD KEY_INFO =0xB008;
		 const WORD KEY_MESH =0xB002;
			 const WORD KEYF_HIERARCY =0xB010;
			 const WORD KEYF_DUMMY    =0xB011;
			 const WORD KEYF_PIVOT    =0xB013;
			 const WORD TRACK_POS     =0xB020;
			 const WORD TRACK_ROT     =0xB021;
			 const WORD TRACK_SCL     =0xB022;
			 const WORD KEYF_NODEID   =0xB030;
	//end keyframe chunk

	//some chunks in all others
	const WORD GLOBAL_RGB_FLOAT       =0x0010;
	const WORD GLOBAL_RGB_BYTE        =0x0011;
	const WORD GLOBAL_RGB_BYTE_GAMMA  =0x0012;
	const WORD GLOBAL_RGB_FLOAT_GAMMA =0x0013;
	const WORD GLOBAL_PER_WORD        =0x0030;
	const WORD GLOBAL_PER_FLOAT       =0x0031;
//end primary chunk


struct STRING_3DS{
	char string[128];
};

struct CHUNK_3DS{
	WORD ID;
	UINT length;
};

union INDEX{
	struct{
		int u,v,n;
	};
	int a[3];
};
union VECTOR{
	struct{
		float u,v,n;
	};
	float a[3];
};
///////////////////
struct GROUPV{
	string name;
	vector<VECTOR> vertices;
	vector<INDEX> facets;
};
//
//class C3DSReader{
//private:
//
//	STRING_3DS direct;
//	FILE *pfile;
//
//	BYTE ReadByte();
//	WORD ReadWord();
//	UINT ReadUint();
//	float ReadFloat();
//	UINT ReadString(STRING_3DS &string);
//	UINT ReadBRGB(BYTE &red,BYTE &green,BYTE &blue);
//	UINT ReadFRGB(float &red,float &green,float &blue);
//	UINT ReadBPER(BYTE &per);
//	UINT ReadFPER(float &per);
//	
//	CHUNK_3DS ReadChunk();
////
//	UINT ReadPrimary(UINT n);
//		//
//		UINT ReadEdit(UINT n);
//			UINT ReadMesh(UINT n);
//				UINT ReadMeshInfo(UINT n);
//					UINT ReadFacetInfo(UINT n);
//		//
//		UINT ReadMaterial(UINT n);
//			UINT ReadMatDif(UINT n);
//			UINT ReadMatMap(UINT n);
//		//
//		UINT ReadKeyframe(UINT n);
//			UINT ReadKeyMesh(UINT n);
////
//	void GetDirect(const char *str);
//protected:
//	virtual void UserKeyframeID(WORD id);
//	virtual void UserKeyframeTrackScl(WORD frame,float x,float y,float z);
//	virtual void UserKeyframeTrackRot(WORD frame,float angle,float x,float y,float z);
//	virtual void UserKeyframeTrackPos(WORD frame,float x,float y,float z);
//	virtual void UserKeyframePivot(float x,float y,float z);
//	virtual void UserKeyframeName(const char *name);
//	virtual void UserKeyframeParent(WORD id);
//	virtual void UserKeyframeRange(UINT start,UINT end);
//
//	virtual void UserMaterialMapName(const char *name);
//	virtual void UserMaterialDiffuseGamma(BYTE red,BYTE green,BYTE blue);
//	virtual void UserMaterialDiffuse(BYTE red,BYTE green,BYTE blue);
//	virtual void UserMaterialName(const char *name);
//
//	virtual void UserMeshMaterialFacet(WORD in);
//	virtual void UserMeshMaterialName(const char *name);
//	virtual void UserMeshFacet(WORD id1,WORD id2,WORD id3);
//	virtual void UserMeshLocalO(float x,float y,float z);
//	virtual void UserMeshLocalN(float x,float y,float z);
//	virtual void UserMeshLocalV(float x,float y,float z);
//	virtual void UserMeshLocalU(float x,float y,float z);
//	virtual void UserMeshTexCoord(float u,float v);
//	virtual void UserMeshVertex(float x,float y,float z);
//	virtual void UserMeshName(const char *name);
//public:
//	bool Load(const char *file);
//	C3DSReader();
//	~C3DSReader();
//};
//
//C3DSReader::C3DSReader(){
//}
//
//C3DSReader::~C3DSReader(){
//}
//
//void C3DSReader::GetDirect(const char *str){
//	int l=strlen(str);
//	while(--l>=0)
//		if(str[l]=='/')break;
//	strncpy(direct.string,str,l+1);
//	direct.string[l+1]='\0';
//}
//
//bool C3DSReader::Load(const char *file){
//	if((pfile=fopen(file,"rb"))==NULL)return false;
//	GetDirect(file);
//	CHUNK_3DS chunk;
//	chunk=ReadChunk();
//	if(chunk.ID!=PRIM){
//		fclose(pfile);
//		return false;
//	}
//	ReadPrimary(chunk.length-6);
//	fclose(pfile);
//	return true;
//}
//
//BYTE C3DSReader::ReadByte(){
//	BYTE out;
//	fread(&out,1,1,pfile);
//	return out;
//}
//	
//WORD C3DSReader::ReadWord(){
//	return ReadByte()+(ReadByte()<<8);
//}
//UINT C3DSReader::ReadUint(){
//	return ReadWord()+(ReadWord()<<16);
//}
//float C3DSReader::ReadFloat(){
//	float out;
//	fread(&out,sizeof(float),1,pfile);
//	return out;
//}
//UINT C3DSReader::ReadString(STRING_3DS &str){
//	int i=0;
//	while((str.string[i++]=ReadByte())!=0);
//	return i;
//}
//UINT C3DSReader::ReadBRGB(BYTE &red,BYTE &green,BYTE &blue){
//	fseek(pfile,6,SEEK_CUR);
//	red=ReadByte();
//	green=ReadByte();
//	blue=ReadByte();
//	return 6;
//}
//UINT C3DSReader::ReadBPER(BYTE &per){
//	fseek(pfile,6,SEEK_CUR);
//	per=ReadByte();
//	return 6;
//}
//UINT C3DSReader::ReadFPER(float &per){
//	fseek(pfile,6,SEEK_CUR);
//	per=ReadFloat();
//	return 6;
//}
//UINT C3DSReader::ReadFRGB(float &red,float &green,float &blue){
//	fseek(pfile,6,SEEK_CUR);
//	red=ReadFloat();
//	green=ReadFloat();
//	blue=ReadFloat();
//	return 6;
//}
//CHUNK_3DS C3DSReader::ReadChunk(){
//	CHUNK_3DS chunk;
//	chunk.ID=ReadWord();
//	chunk.length=ReadUint();
//	return chunk;
//}
////
//UINT C3DSReader::ReadPrimary(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case PRIM_EDIT:
//			ReadEdit(chunk.length-6);
//			count+=chunk.length;
//			break;
//		case PRIM_KEY:
//			ReadKeyframe(chunk.length-6);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//UINT C3DSReader::ReadEdit(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case EDIT_MESH:
//			ReadMesh(chunk.length-6);
//			count+=chunk.length;
//			break;
//		case EDIT_MAT:
//			ReadMaterial(chunk.length-6);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//UINT C3DSReader::ReadMesh(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	STRING_3DS str;
//	count+=ReadString(str);
//	UserMeshName(str.string);
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case MESH_INFO:
//			ReadMeshInfo(chunk.length-6);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//
//}
//UINT C3DSReader::ReadMeshInfo(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	float f1,f2,f3;
//	WORD t;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case MESH_VERTEX:
//			t=ReadWord();//number of vertices in this mesh
//			while(t>0){
//				//tranform vertex from 3ds coord to opengl coord
//				f1=ReadFloat();//x coord in 3ds world
//				f2=ReadFloat();//y coord in 3ds world
//				f3=ReadFloat();//z coord in 3ds world
//				UserMeshVertex(f1,f2,f3);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		case MESH_FACET:
//			ReadFacetInfo(chunk.length-6);
//			count+=chunk.length;
//			break;
//		case MESH_MAPCOORD:
//			t=ReadWord();//number of texcoords in this mesh
//			while(t>0){
//				f1=ReadFloat();//u
//				f2=ReadFloat();//v
//				UserMeshTexCoord(f1,f2);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		case MESH_LOCAL:
//			//local transform matrix
//			//anxis U
//			f1=ReadFloat();
//			f2=ReadFloat();
//			f3=ReadFloat();
//			UserMeshLocalU(f1,f2,f3);
//
//			//anxis V
//			f1=ReadFloat();
//			f2=ReadFloat();
//			f3=ReadFloat();
//			UserMeshLocalV(f1,f2,f3);
//
//			//anxis N
//			f1=ReadFloat();
//			f2=ReadFloat();
//			f3=ReadFloat();
//			UserMeshLocalN(f1,f2,f3);
//
//			//origin
//			f1=ReadFloat();
//			f2=ReadFloat();
//			f3=ReadFloat();
//			UserMeshLocalO(f1,f2,f3);
//
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//UINT C3DSReader::ReadFacetInfo(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	STRING_3DS str;
//	WORD w1,w2,w3;
//	WORD t;
//	//number of facets in this mesh
//	t=ReadWord();
//	count+=2;
//	while(t>0){
//		w1=ReadWord();//index pointed to vertex which is 1st corner in facet 
//		count+=2;
//		w2=ReadWord();//index pointed to vertex which is 2sd corner in facet 
//		count+=2;
//		w3=ReadWord();//index pointed to vertex which is 3th corner in facet 
//		UserMeshFacet(w1,w2,w3);
//		count+=2;
//		ReadWord();//facet flag ,not used yet
//		count+=2;
//		t--;
//	}
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case FACET_MAT:
//			ReadString(str);//name of material attached to this mesh
//			UserMeshMaterialName(str.string);
//			t=ReadWord();//number of faces attached to this material
//			while(t>0){
//				w1=ReadWord();//index of facet attached to this material
//				UserMeshMaterialFacet(w1);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//
//}
//
//UINT C3DSReader::ReadMaterial(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	STRING_3DS str;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case MAT_NAME:
//			ReadString(str);//name of material
//			UserMaterialName(str.string);
//			count+=chunk.length;
//			break;
//		case MAT_DIF:
//			ReadMatDif(chunk.length-6);
//			count+=chunk.length;
//			break;
//		case MAT_MAP:
//			ReadMatMap(chunk.length-6);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//UINT C3DSReader::ReadMatDif(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	BYTE b1,b2,b3;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case GLOBAL_RGB_BYTE://diffuse color of material 
//			b1=ReadByte();//red
//			b2=ReadByte();//green
//			b3=ReadByte();//blue
//			UserMaterialDiffuse(b1,b2,b3);
//			count+=chunk.length;
//			break;
//		case GLOBAL_RGB_BYTE_GAMMA://gamma of diffuse color of material 
//			b1=ReadByte();//red
//			b2=ReadByte();//green
//			b3=ReadByte();//blue
//			UserMaterialDiffuseGamma(b1,b2,b3);
//			count+=chunk.length;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//
//UINT C3DSReader::ReadMatMap(UINT n){
//	UINT count=0;
//	STRING_3DS str;
//	CHUNK_3DS chunk;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case MAP_NAME:
//			ReadString(str);//name of texture map file
//			UserMaterialMapName(str.string);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//
//UINT C3DSReader::ReadKeyframe(UINT n){
//	UINT count=0;
//	CHUNK_3DS chunk;
//	UINT i1,i2;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case KEY_INFO:
//			i1=ReadUint();//start frame 
//			i2=ReadUint();//end frame
//			UserKeyframeRange(i1,i2);
//			count+=chunk.length;
//			break;
//		case KEY_MESH:
//			ReadKeyMesh(chunk.length-6);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//UINT C3DSReader::ReadKeyMesh(UINT n){
//	UINT count=0;
//	WORD t,w;
//	STRING_3DS str;
//	float f1,f2,f3,f4;
//	CHUNK_3DS chunk;
//	while(count<n){
//		chunk=ReadChunk();
//		switch(chunk.ID){
//		case KEYF_HIERARCY:
//			ReadString(str);//name of key frame if is "$$$DUNNY" then is dummy
//			UserKeyframeName(str.string);
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			t=ReadWord();//index of parent key frame
//			UserKeyframeParent(t);
//			count+=chunk.length;
//			break;
//		case KEYF_DUMMY:
//			ReadString(str);//dummy name of key frame
//			UserKeyframeName(str.string);
//			count+=chunk.length;
//			break;
//		case KEYF_PIVOT:
//			//pivot
//			f1=ReadFloat();//x
//			f2=ReadFloat();//y
//			f3=ReadFloat();//z
//			UserKeyframePivot(f1,f2,f3);
//			count+=chunk.length;
//			break;
//		case TRACK_POS:
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			t=ReadWord();
//			ReadWord();
//			while(t>0){
//				w=ReadWord();//frame number of this key frame
//				ReadUint();//not used yet
//				f1=ReadFloat();//x
//				f2=ReadFloat();//y
//				f3=ReadFloat();//z
//				UserKeyframeTrackPos(w,f1,f2,f3);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		case TRACK_ROT:
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			t=ReadWord();
//			ReadWord();
//			while(t>0){
//				w=ReadWord();//frame number of this key frame
//				ReadUint();//not used yet
//				f4=ReadFloat();//angle in degree
//				f1=ReadFloat();//x
//				f2=ReadFloat();//y
//				f3=ReadFloat();//z
//				UserKeyframeTrackRot(w,f4,f1,f2,f3);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		case TRACK_SCL:
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			ReadWord();//not used yet
//			t=ReadWord();
//			ReadWord();
//			while(t>0){
//				w=ReadWord();
//				ReadUint();
//				f1=ReadFloat();
//				f2=ReadFloat();
//				f3=ReadFloat();
//				UserKeyframeTrackScl(w,f1,f2,f3);
//				t--;
//			}
//			count+=chunk.length;
//			break;
//		case KEYF_NODEID:
//			t=ReadWord();
//			UserKeyframeID(t);
//			count+=chunk.length;
//			break;
//		default:
//			count+=chunk.length;
//			fseek(pfile,chunk.length-6,SEEK_CUR);
//			break;
//		}
//	}
//	return count;
//}
//void C3DSReader::UserKeyframeID(WORD id){
//}
//void C3DSReader::UserKeyframeTrackScl(WORD frame,float x,float y,float z){
//}
//void C3DSReader::UserKeyframeTrackRot(WORD frame,float angle,float x,float y,float z){
//}
//void C3DSReader::UserKeyframeTrackPos(WORD frame,float x,float y,float z){
//}
//void C3DSReader::UserKeyframePivot(float x,float y,float z){
//}
//void C3DSReader::UserKeyframeName(const char *name){
//}
//void C3DSReader::UserKeyframeParent(WORD id){
//}
//void C3DSReader::UserKeyframeRange(UINT start,UINT end){
//}
//
//void C3DSReader::UserMaterialMapName(const char *name){
//}
//void C3DSReader::UserMaterialDiffuseGamma(BYTE red,BYTE green,BYTE blue){
//}
//void C3DSReader::UserMaterialDiffuse(BYTE red,BYTE green,BYTE blue){
//}
//void C3DSReader::UserMaterialName(const char *name){
//}
//
//void C3DSReader::UserMeshMaterialFacet(WORD in){
//}
//void C3DSReader::UserMeshMaterialName(const char *name){
//}
//void C3DSReader::UserMeshFacet(WORD id1,WORD id2,WORD id3){
//}
//void C3DSReader::UserMeshLocalO(float x,float y,float z){
//}
//void C3DSReader::UserMeshLocalN(float x,float y,float z){
//}
//void C3DSReader::UserMeshLocalV(float x,float y,float z){
//}
//void C3DSReader::UserMeshLocalU(float x,float y,float z){
//}
//void C3DSReader::UserMeshTexCoord(float u,float v){
//}
//void C3DSReader::UserMeshVertex(float x,float y,float z){
//}
//void C3DSReader::UserMeshName(const char *name){
//}
#endif
