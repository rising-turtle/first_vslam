#include <Winsock2.h>
#include <string.h>
#include <windows.h>
#include <stdio.h>


//#define MY_IP "10.0.0.5"
#define MY_IP "192.168.1.100"
#define MY_PORT 5000
#define BACKLOG 10
#define MAX_CLIENT_NUM 10


#define IOT_TMP_BUFF_SIZE 100
#define IOT_MEMORY_POOL_SIZE 1024*1024*10 //10M
#define IOT_RGBBUFF_SIZE 640*480*3
#define IOT_DEPTHBUFF_SIZE 640*480*2
#define IOT_QVGARGBBUFF_SIZE 320*240*3

#define  KINECT_NET_CONF_LEN 28
/*#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageViewer.h>
#include <../io/vtkBMPReader.h>
#include <../Filtering/vtkImageData.h>
#include <../Filtering/vtkPointData.h>

#include <vtkGraphicsFactory.h>
#include <vtkWin32RenderWindowInteractor.h>
#include <vtkCommand.h>*/

#define MEMORY_ALLOCATION(Ptr,Type,Addr,Len,Count){ Ptr=(Type)Addr; Addr+=Len;Count+=Len;}

typedef void (*NetServerCallBack)(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext);


typedef void (*NetServerCallBack_SLAM_IMU)(unsigned char*pucRGB,unsigned short *pusDepth,char *pcIMUData,void *pContext);
typedef void (*NetServerCallBack_SLAM)(unsigned char*pucRGB,unsigned short *pusDepth,void *pContext);
typedef void (*NetServerCallBack_Display)(unsigned char*pucRGB,void *pContext);


typedef struct IoT_NetCallBackSet
{
	NetServerCallBack_SLAM cbSlam;
	void *pSlamContext;

	NetServerCallBack_SLAM_IMU cbSlam_IMU;
	void *pSlamIMUContext;


	NetServerCallBack_Display cbDisplay;
	void *pDisplayContext;

}IoT_NetCallBackSet;

typedef struct IoT_KinectNetConf
{
	char cType;
	char cFrequency;
	char cCompression;
	char cResolution;
	
}IoT_KinectNetConf;


typedef struct IoT_NetConf
{
	IoT_KinectNetConf stSLAMConf;
	IoT_KinectNetConf stVisulizationConf;
}IoT_NetConf;




typedef struct IoT_NetDataPtrSet
{


	char cCurWrtRGBBuff;
	char cCurWrtDepthBuff;

	char cCurReadRGBBuff;
	char cCurReadDepthBuff;

	char *pTmpBuff;
	char *pTmpKinectBuff;
	char *pTmpCmdBuff;
	char *pTmpIDriveBuff;

	unsigned char *pucRGBBuffA;
	unsigned char *pucRGBBuffB;
	unsigned char *pucRGB;
	unsigned char *pucQVGARGB;

	unsigned short *pusDepthBuffA;
	unsigned short *pusDepthBuffB;
	unsigned short *pusDepth;
}IoT_NetDataPtrSet;


typedef struct IoT_RobotInfo
{
	char cName[100];
	char cType[100];
	int nIdx;
}IoT_RobotInfo;

typedef struct IoT_NetClientInfo
{
	HANDLE hThreadCmd;
	HANDLE hThreadKinect;
	HANDLE hThreadIDrive;

	IoT_RobotInfo stRobotInfo;
	struct sockaddr_in ClientAddr;

	int nCmdPort;
	int nKinectPort;
	int nIDrivePort;

	struct sockaddr_in CmdAddr;
	struct sockaddr_in KinectAddr;
	struct sockaddr_in IDriveAddr;

	int nSockCmd;
	int nSockKinect;
	int nSockIDrive;

	LPDWORD ID_Cmd;
	LPDWORD ID_Kinect;
	LPDWORD ID_IDrive;

	int nCmdNewFd;
	int nKinectNewFd;
	int nIDriveNewFd;

	IoT_NetConf stNetConf;

}IoT_NetClientInfo;


typedef struct IoT_NetClientInfoSet
{
	IoT_NetClientInfo stNetClientInfo[MAX_CLIENT_NUM];
	int nOccupyFlag[MAX_CLIENT_NUM];
}IoT_NetClientInfoSet;



