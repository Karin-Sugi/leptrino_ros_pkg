
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "leptrino/pCommon.h"
#include "leptrino/rs_comm.h"
#include "leptrino/pComResInternal.h"

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

// =============================================================================
//	define macro
// =============================================================================
#define PRG_VER	"Ver 1.0.0"

// =============================================================================
//	define structure
// =============================================================================
typedef struct ST_SystemInfo {
	int com_ok;
} SystemInfo;

// =============================================================================
//	function
// =============================================================================
void App_Init(void);
void App_Close(void);
int GetRcv_to_Cmd( char *rcv, char *prm);
ULONG SendData(UCHAR *pucInput, USHORT usSize);
void GetProductInfo(void);
void GetLimit(void);
void SerialStart(void);
void SerialStop(void);

// =============================================================================
//	modules
// =============================================================================
SystemInfo gSys;
UCHAR CommRcvBuff[256];
UCHAR CommSendBuff[1024];
UCHAR SendBuff[512];

std::string g_com_port = "/dev/ttyACM0";
double conversion_factor[FN_Num];
// ----------------------------------------------------------------------------------
//	main
// ----------------------------------------------------------------------------------
//	arg  	: int argc, char** argv (For ros)
//	return	: non
// ----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "leptrino");
	ros::NodeHandle nh;
	ros::NodeHandle nh_private("~");
	ros::Rate rate(1200);

	int i, l = 0, rt = 0;
	int mode_step = 0;
	int AdFlg = 0, EndF = 0;
	UCHAR strprm[256];
	ST_RES_HEAD *stCmdHead;
	ST_R_DATA_GET_F *stForce;
	ST_R_GET_INF *stGetInfo;
	ST_R_LEP_GET_LIMIT* stGetLimit;

	App_Init();
	
	if (gSys.com_ok == NG) {
		ROS_ERROR("ComPort Open Fail\n");
		exit(0);
	}
	
	// Get information of the product
	GetProductInfo();
	while(ros::ok()) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		// when able to get data
			CommRcvBuff[0]=0; 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stGetInfo = (ST_R_GET_INF *)CommRcvBuff;
				stGetInfo->scFVer[F_VER_SIZE] = 0;
				printf("Version:%s\n", stGetInfo->scFVer);
				stGetInfo->scSerial[SERIAL_SIZE] = 0;
				printf("SerialNo:%s\n", stGetInfo->scSerial);
				stGetInfo->scPName[P_NAME_SIZE] = 0;
				printf("Type:%s\n", stGetInfo->scPName);
				printf("\n");
				EndF = 1;
			}
			
		}
		else if ( EndF==1 ) break;
		else{
			rate.sleep();
		}
	}

  GetLimit();
  while(ros::ok()){
    Comm_Rcv();
    if (Comm_CheckRcv() != 0){ // possible to get data
      CommRcvBuff[0] = 0;

      rt = Comm_GetRcvData(CommRcvBuff);
      if (rt > 0){
        stGetLimit = (ST_R_LEP_GET_LIMIT *)CommRcvBuff;
        for (int i = 0; i < FN_Num; i++)
        {
          ROS_INFO("\tLimit[%d]: %f", i, stGetLimit->fLimit[i]);
          conversion_factor[i] = stGetLimit->fLimit[i] * 1e-4;
        }
        break;
      }
    }
    else
    {
      rate.sleep();
    }
  }
	
	ros::Publisher force_torque_pub = nh_private.advertise<geometry_msgs::WrenchStamped>("force_torque", 1);
	usleep(10000);

	// send signal
	SerialStart();
	EndF = 0;
	while(ros::ok()) {
		Comm_Rcv();
		if ( Comm_CheckRcv() != 0 ) {		//when able to get data
			memset(CommRcvBuff,0,sizeof(CommRcvBuff)); 
			
			rt = Comm_GetRcvData( CommRcvBuff );
			if ( rt>0 ) {
				stForce = (ST_R_DATA_GET_F *)CommRcvBuff;
				ROS_DEBUG_THROTTLE(0.1, "%d,%d,%d,%d,%d,%d",
						stForce->ssForce[0],stForce->ssForce[1],stForce->ssForce[2],
						stForce->ssForce[3],stForce->ssForce[4],stForce->ssForce[5]);
				geometry_msgs::WrenchStampedPtr msg(new geometry_msgs::WrenchStamped);
				msg->header.stamp = ros::Time::now();
				msg->header.frame_id = "leptrino";
				msg->wrench.force.x = stForce->ssForce[0] * conversion_factor[0];
				msg->wrench.force.y = stForce->ssForce[1] * conversion_factor[1];
				msg->wrench.force.z = stForce->ssForce[2] * conversion_factor[2];
				msg->wrench.torque.x = stForce->ssForce[3] * conversion_factor[3];
				msg->wrench.torque.y = stForce->ssForce[4] * conversion_factor[4];
				msg->wrench.torque.z = stForce->ssForce[5] * conversion_factor[5];
				force_torque_pub.publish(msg);
			}
		}
		else if ( EndF==1 ) break;
		else{
			rate.sleep();
		}
		ros::spinOnce();
	}
		
	SerialStop();
	App_Close();
	return 0;
}

// ----------------------------------------------------------------------------------
//	init application
// ----------------------------------------------------------------------------------
//	arg	: non
//	return	: non
// ----------------------------------------------------------------------------------
void App_Init(void)
{
	int rt;
	
	//initialize Comm port
	gSys.com_ok = NG;
	rt = Comm_Open((char*)g_com_port.c_str());
	if ( rt==OK ) {
		Comm_Setup( 460800, PAR_NON, BIT_LEN_8, 0, 0, CHR_ETX);
		gSys.com_ok = OK;
	}

}

// ----------------------------------------------------------------------------------
//	Close application
// ----------------------------------------------------------------------------------
//	arg	: non
//	return	: non
// ----------------------------------------------------------------------------------
void App_Close(void)
{
	printf("Application Close\n");
	
	if ( gSys.com_ok == OK) {
		Comm_Close();
	}
}

/*********************************************************************************
* Function Name  : HST_SendResp
* Description    : form data and send it
* Input          : pucInput (data)
*                : data size
* Output         : 
* Return         : 
*********************************************************************************/
ULONG SendData(UCHAR *pucInput, USHORT usSize)
{
	USHORT usCnt;
	UCHAR ucWork;
	UCHAR ucBCC = 0;
	UCHAR *pucWrite = &CommSendBuff[0];
	USHORT usRealSize;
	
	// form data 
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_STX;					// STX 
	pucWrite++;
	usRealSize =2;
	
	for (usCnt = 0; usCnt < usSize; usCnt++) {
		ucWork = pucInput[usCnt];
		if (ucWork == CHR_DLE) {			
			*pucWrite = CHR_DLE;			// DLE 
			pucWrite++;						// place to write 
			usRealSize++;					// real size
			// BCC�͌v�Z���Ȃ�!
		}
		*pucWrite = ucWork;					// data
		ucBCC ^= ucWork;					// BCC 
		pucWrite++;							// place to write 
		usRealSize++;						// real size
	}
	
	*pucWrite = CHR_DLE;					// DLE 
	pucWrite++;
	*pucWrite = CHR_ETX;					// ETX 
	ucBCC ^= CHR_ETX;						// BCC calcuration
	pucWrite++;
	*pucWrite = ucBCC;						// BCC calcuration
	usRealSize += 3;
	
	Comm_SendData(&CommSendBuff[0], usRealSize);
	
	return OK;
}

void GetProductInfo(void)
{
	USHORT len;
	
	printf("Get SensorInfo\n");
	len = 0x04;								// length  of data
	SendBuff[0] = len;						// length
	SendBuff[1] = 0xFF;						// sensor No.
	SendBuff[2] = CMD_GET_INF;				// command type
	SendBuff[3] = 0;						// no meaning
	
	SendData(SendBuff, len);
}

void GetLimit(void)
{
  USHORT len;

  ROS_INFO("Get sensor limit");
  len = 0x04;
  SendBuff[0] = len; //length
  SendBuff[1] = 0xFF; // sensor No.
  SendBuff[2] = CMD_GET_LIMIT; // command type
  SendBuff[3] = 0; // no meaning

  SendData(SendBuff, len);
}

void SerialStart(void)
{
	USHORT len;
	
	printf("Start\n");
	len = 0x04;								// length  of data
	SendBuff[0] = len;						// length
	SendBuff[1] = 0xFF;						// sensor No.
	SendBuff[2] = CMD_DATA_START;			// command type
	SendBuff[3] = 0;						// no meaning
	
	SendData(SendBuff, len);
}

void SerialStop(void)
{
	USHORT len;
	
	printf("Stop\n");
	len = 0x04;								// length  of data
	SendBuff[0] = len;						// length
	SendBuff[1] = 0xFF;						// sensor No.
	SendBuff[2] = CMD_DATA_STOP;			// command type
	SendBuff[3] = 0;						// no meaning
	
	SendData(SendBuff, len);
}

