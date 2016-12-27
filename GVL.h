/* ========================================
*
* Copyright University of Auckland, 2016
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : N/A
* Creation date : 2016-03-05
* Version       : 1.0 (finished on ...)
* Modifications :
* Known bugs    :
*
*
* Possible Improvements
*
*
* Notes
*
*
* ========================================
*/
  
#include <project.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h> 
#include <math.h> 

//This file consists of all the global variables



//GPS Rx buffer
#define RxBufferSize 100
char	ReceiveBuffer[RxBufferSize]; //Circular Buffer for GPS Rx messages
char	*RxReadIndex	= ReceiveBuffer;
char	*RxWriteIndex	= ReceiveBuffer;
int 	GPS_EOM_flag = 0; //End Of Message Flag : indicate to the main that a GPS message has been completely received
uint8   BuffCompletellyFilled = 0; //just an indication parameter

//DGPS Rx buffer
char	ReceiveBufferDGPS[RxBufferSize]; //Circular Buffer for GPS Rx messages
char	*RxReadIndexDGPS	= ReceiveBufferDGPS;
char	*RxWriteIndexDGPS	= ReceiveBufferDGPS;
int 	DGPS_EOM_flag = 0; //End Of Message Flag : indicate to the main that a DGPS message has been completely received


float 	Temp_gyro               = 0;
float 	Temp_magneto          = 0;

//WU
int 	WaitForWarmUp           = 0;
int 	WarmUpLength            = 60;
int 	cpt_WaitForWarmUp     = 0;

//ZUPT
int 	CalibrateAtStartUp        = 0;
int 	ZUPTLength                 = 60;
int 	cpt_CalibrateAtStartUp  = 0;

//Time
float TimerPeriod = 0.0;
float LastTime = 0.0;
float ThisTime = 0.0;

uint TimerRestarted = 0;

///////////////////////////////////////////////////////////////////Struct/////////////////////////////////////////////////////////////
typedef struct GPS_Struct
{

	/* Normal GPS */
	char   message[RxBufferSize];
	char*   MessageID;
	int     Frame_Type;
	int     hours;
	int     minutes;
	int     seconds;
	int     DataValidityStatus;
	int     milliseconds;
	float   latitudeDegrees;
	float   longitudeDegrees;
	float   magnetic_variation;
	char8   EW_VR;
	char8   attitudeUnit;
	char8   NS_Indicator;
	char8   EW_Indicator;
	float   GS;
	float   GC;
	int     day;
	int     month;
	int     year;
	char8   Mode;
  
    int16 PWM_Left_Orders;
    int16 PWM_Right_Orders;
	char   	DGPSmessage[RxBufferSize];

} GPS_Struct;


GPS_Struct GPS;

typedef struct Nav_Struct
{
	int   RequestedSpeed;
	float DeltaTRK;         // Difference between the desired TRK and the estimated/real TRK
	int   AutopilotMode;
	float TRK;
	float Alpha_Vw;         // Angle of waves       DD : decimal degree  formatting
	float Vw;               // speed of waves       unit : m/s
	float Vw_X;             // projection on North   unit : m/s
	float Vw_Y;             // projection on East   unit : m/s
	float TAS;              // True Air Speed       unit : m/s
	float TAS_X;            // projection on North  unit : m/s
	float TAS_Y;            // projection on East   unit : m/s
	float TAS_Z;            // projection on Up     unit : m/s
	float GS;               // Ground Speed         unit : m/s
	float GS_X;             // projection on North   unit : m/s
	float GS_Y;             // projection on East   unit : m/s
	float TRK_Est;          // Estimated TRK thanks to Acc if no GPS
	float Longitude;		// DD : decimal degree  formatting
	float Latitude;			// DD : decimal degree  formatting
	float Altitude;			//unit : m (above the see)
	float Dt;				// unit : s  time between Nav
	int   IMUEstimation;    // 0 if the GPS last filled this struct, 1 if the IMU that last filled this struct
    
    float AccX;
    float AccY;
    float AccZ;
    float GyrX;
    float GyrY;
    float GyrZ;
    float Pitch;
    float Roll;
	float HDG;
    
    float DistToNextWayPoint; // unit : m
    float Bearing;         // unit : rad
    float CurrentlyFollowedWPLat;
    float CurrentlyFollowedWPLong;

    
    //TEMPORARY
    float RequestedTRK;     // unit : rad
    float WPLong;           // unit : rad
    float WPLat;            // unit : rad
	
}Nav_Struct;