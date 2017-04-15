#pragma once
#define DLL_EXPORT __declspec(dllexport)
#include "IoTRobot_NetServer_Define.h"
#include "IoTRobot_NetProtocol.h"




class DLL_EXPORT IoTRobot_NetServer
{

public:
	NetServerCallBack m_cRegisterCallBack;
	//int NetServer_Init(NetServerCallBack cCallBack);


public:
	IoTRobot_NetServer(void);
	~IoTRobot_NetServer(void);

	

	int m_nSockCmd;
	int m_nSockRGB;
	int m_nSockDepth;

	char *m_pcMemoryPool;
	IoT_NetDataPtrSet m_stDataPtrSet;

	struct sockaddr_in m_MyAddr;


	IoT_NetClientInfoSet m_stNetCleintInfoSet;
	bool m_bStopListen;
	int NetServer_Init(IoT_NetCallBackSet stCallBackSet);
	int NetServer_Run();
	int NetServer_Uninit();
	int NetServer_Conf(int nClientIdx,IoT_NetConf stNetConf);
	int NetServer_GetOneSlamFrame(int nClientIdx);

	char m_cKinectConf[KINECT_NET_CONF_LEN];
	bool m_bNewKinectConf;
	bool m_bGetOneSlamFrame;


//	IoT_NetConf m_stNetConf;


	int AnwserClientRequests(int nSockFD);
	static int RcvStream(int nGoalSize,char *pcTmpBuff,int sockfd);
	int SendStream(int nGoalSize,char *pcTmpBuff,int sockfd);

	IoT_NetClientInfo * GetValidCIPos();
	int OpenThreadForClient(IoT_NetClientInfo * pstCI);

	static UINT ThreadKinectSock(LPVOID lpParam);
	static UINT ThreadCmdSock(LPVOID lpParam);
	static UINT ThreadIDriveSock(LPVOID lpParam);


	static UINT ThreadKinectSendCmd(LPVOID lpParam);

	int VGA2QVGA(unsigned char *pucVGA,unsigned char *pucQVGA);

	IoT_NetCallBackSet m_stCallBackSet;
};
