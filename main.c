/* ========================================
*
* Copyright University of Auckland, 2016
* All Rights Reserved
* UNPUBLISHED, LICENSED SOFTWARE.
*
* Metadata
* Written by    : Nathanaël Esnault
* Verified by   : N/A
* Creation date : 2016-11-17
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


//Use the cydwr window to change the priority of an interrupt. Remember that 0 is the highest priority; and 7, the lowest
//priority. The Cortex-M0 supports interrupt nesting; see Nested Interrupts for details.



///////////////////////////////////////////////////////////////////Include/////////////////////////////////////////////////////////////
#include "IMU.h"
#include "IMUCalibration.h"
//#include "GPS.h"
//#include "ID.h"
#include "Autopilot.h"
#include "Broadcast.h"
#include "GVL.h"

#include <project.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>     /* atof */
#include <math.h>       /* fmod */



///////////////////////////////////////////////////////////////////Functions Definitions/////////////////////////////////////////////////////////////
void InitEntireSystem (void);
void KeepTimeLoop(void);

//void ExecuteAutopilot (Nav_Struct *Nav, GPS_Struct *GPS);
//void InfoToNextWayPoint(Nav_Struct *Nav);
//void ExecuteNav (Nav_Struct *Nav);

//MOUVEMENTS
void Advance    (int speed, float TimeToMove);
void GoBack     (int speed, float TimeToTurn);


//MOTORS
void Init_PWMs(void);
void motor_1_Set_Speed(int speed);
void motor_2_Set_Speed(int speed);
void motor_Regulate_Speed(int RequestedSpeed_1, int RequestedSpeed_2);


void GlobalInit_ToMatlab (void);


//GPS BUFFER
uint8	IsCharReady(void);
char	GetRxChar(void);

void PreTest(void);

///////////////////////////////////////////////////////////////////Define/////////////////////////////////////////////////////////////
#define RxBufferSizeDGPS 200
#define RMC_MESSAGE_ID "GPRMC" // Recommended Mininum Navigation Information
float Dt = 0.0;

//DGPS BUFFER
uint8	IsCharReadyDGPS(void);
char	GetRxCharDGPS(void);
void  ParseDGPS (GPS_Struct *GPS);

Vector3 	magRawData= {0, 0, 0}, 		accelRawData= {0, 0, 0}, 	gyroRawData= {0, 0, 0};
Vector3f 	AccValues= {0.0, 0.0, 0.0}, MagValues= {0.0, 0.0, 0.0}, GyroValues= {0.0, 0.0, 0.0}, ArandomThing= {0.0, 0.0, 0.0}, ZUPT_Acc= {0.0, 0.0, 0.0}, ZUPT_Gyro= {0.0, 0.0, 0.0}, ZUPT_ArandomThing= {0.0, 0.0, 0.0};


Nav_Struct Nav;

///////////////////////////////////////////////////////////////////Interrupts handling/////////////////////////////////////////////////////////////

CY_ISR_PROTO(MyRxInt);

CY_ISR_PROTO(MyRxIntDGPS);

CY_ISR_PROTO(MyTimerISR);

///////////////////////////////////////////////////////////////////Main/////////////////////////////////////////////////////////////
int main()
{  
    InitEntireSystem();
	PreTest();
    
	//===========================================================================================
	// Beginning of the loop
	
	for(;;)
	{
    
    KeepTimeLoop();
	
	// int cpt_1 = 0;
    // memset(ReceiveBufferDGPS, '0', RxBufferSize);
	// ReceiveBufferDGPS[cpt_1] = '$'; //Circular Buffer for GPS Rx messages 
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '4'; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '5'; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = ','; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '1'; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '1'; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '7'; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = '*'; //Circular Buffer for GPS Rx messages
    
                      
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = 0x0D; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = 0X0A; //Circular Buffer for GPS Rx messages
    // cpt_1 ++; ReceiveBufferDGPS[cpt_1] = 0x00; //Circular Buffer for GPS Rx messages

    // RxWriteIndexDGPS = ReceiveBufferDGPS + cpt_1;
    // DGPS_EOM_flag = 1; //End Of Message Flag : indicate to the main that a DGPS message has been completely received
	
	
	
	
	
		
		//=========================================================================================
		/* GPS receiving*/
		//CYGlobalIntDisable;
		if(GPS_EOM_flag == 1)
		{
			int cpt = 0;
			GPS_EOM_flag = 0;
			char temp;
			
			do
			{
				temp = GetRxChar();
			}while((RxWriteIndex != RxReadIndex) & (temp != 0x24) );//Depile and dump char ONE BY ONE from buff until empty or "$" found
			
			if (temp == 0x24) // if  "$" found then we got a start of message
			{
				GPS.message[0] = '$';
				cpt++;
				
				do //Depile and save a char from buff until "\n" found (we know it has arrived because of GPS_EOM_flag)
				{
					GPS.message[cpt] = GetRxChar();
					cpt++;
				}while((GPS.message[cpt-1] != 0x0A) ); // (RxWriteIndex != RxReadIndex) &
				
//				detect_GPS_Frame_Type_PP(&GPS);
//				if(GPS.Frame_Type != 99)
//				{
//					parse_RMC_PP (&GPS);
//					//UART_MATLAB_PutString(GPS.message);
//                     //DGPS corrections
//                    if (GPS.DGPS_mess_available == 1)
//                    {
//                        GPS.longitudeDegrees = GPS.longitudeDegrees + GPS.DGPS_Corr_Long;
//                        GPS.latitudeDegrees = GPS.latitudeDegrees + GPS.DGPS_Corr_Lat;
//                        GPS.DGPS_mess_available = 0;
//                    }
//				}

				//sprintf(c, "%i", GPS.Frame_Type);
				//UART_MATLAB_PutString(c);
				//UART_MATLAB_PutString("\n\r");
				//memset(c, '0', 50);
				//sprintf(c2, "| %i |", BuffCompletellyFilled);
				//UART_MATLAB_PutString(c2);
				//UART_MATLAB_PutString("\n\r");

				memset(GPS.message, 0, RxBufferSize); // clean the message field
				//memset(ReceiveBuffer, 0, RxBufferSize);
				//RxReadIndex	= ReceiveBuffer;
				//RxWriteIndex = ReceiveBuffer;

				/* Time */
				ThisTime  = ((float)(TIMER_PERIOD - Timer_MATLAB_ReadCounter()) / CLOCK_FREQUENCY);
				Dt = ThisTime - LastTime;
				
				// Nav.AutopilotMode = 1; //GPS Mode
                // ExecuteNav (&Nav);
                // ExecuteAutopilot (&Nav, &GPS);
				
			}
			BuffCompletellyFilled = 0; // reinit the flag anyway
		}
		
		//=========================================================================================
		/* Receiving orders*/

		if(DGPS_EOM_flag == 1)
		{
			int cptDGPS = 0;
			DGPS_EOM_flag = 0;
			char tempDGPS;
			
			do
			{
				tempDGPS = GetRxCharDGPS();
			}while((RxWriteIndexDGPS != RxReadIndexDGPS) & (tempDGPS != 0x24) );//Depile and dump a char from buff until empty or $ found
			
			if (tempDGPS == 0x24) // if  $ found then we got a start of message
			{
				GPS.DGPSmessage[0] = '$';
				cptDGPS++;
				
				do //Depile and save a char from buff until \n found (we know it has arrived because of DGPS_EOM_flag)
				{
					GPS.DGPSmessage[cptDGPS] = GetRxCharDGPS();
					cptDGPS++;
				}while((GPS.DGPSmessage[cptDGPS-1] != 0x0A));
				
				ParseDGPS(&GPS);
				PWM_Enable_Write(0);
				motor_1_Set_Speed(GPS.PWM_Right_Orders);
				motor_2_Set_Speed(GPS.PWM_Left_Orders);
				PWM_Enable_Write(1);
				memset(GPS.DGPSmessage, 0, RxBufferSizeDGPS); // clean the message field
			}
		}
		
		
		
		
		//=========================================================================================
		/* Rest of the loop*/
		
		/* Time */
		ThisTime  = ((float)(TIMER_PERIOD - Timer_MATLAB_ReadCounter()) / CLOCK_FREQUENCY);
		GetDataFromIMU();
		
		/* Filter */
		ArandomThing = ComplementaryFilter1(AccValues, MagValues, GyroValues, ArandomThing, ThisTime - LastTime);
		ArandomThing.x = ArandomThing.x -ZUPT_ArandomThing.x;
		ArandomThing.y = ArandomThing.y -ZUPT_ArandomThing.y;
		ArandomThing.z = ArandomThing.z -ZUPT_ArandomThing.z;
		
		// Nav.Dt = ThisTime - LastTime;
		
		/* Do Navigation */
		// Nav.AutopilotMode = 2; //IMU + Wave Estimation
		// ExecuteNav (&Nav);
		// ExecuteAutopilot (&Nav, &GPS);
	
		// UART_Transmit_Frame(ThisTime - LastTime);
		UART_Transmit_Frame(AccValues.x, AccValues.y, AccValues.z, ArandomThing.x, ArandomThing.y, ArandomThing.z, GyroValues.x, GyroValues.y, GyroValues.z,Temp_gyro, ThisTime - LastTime);
		//1.344kHz acc IMU, 760Hz gyro, 220Hz magneto

		LastTime = ThisTime;
		
	}//end for(;;)
}//end main

///////////////////////////////////////////////////////////////////Functions/////////////////////////////////////////////////////////////

/* GPS UART */
uint8	IsCharReady(void)
{
	return !(RxWriteIndex == RxReadIndex);
}

char	GetRxChar(void)
{
	char	Result;
	//while(!IsCharReady()) Wait();
	Result = *RxReadIndex;
	//*RxReadIndex =0;
	RxReadIndex++;
	if (RxReadIndex >= ReceiveBuffer + RxBufferSize) RxReadIndex = ReceiveBuffer;
	return Result;
}

CY_ISR(MyRxInt)
{

	char temp = {0};
	// while(UART_GPS_ReadRxStatus() & UART_GPS_RX_STS_FIFO_NOTEMPTY)
	// {
		// temp  = UART_GPS_ReadRxData();

		// if (temp != 0x00)
		// {
			// if (temp == 0x0A)
			// {
				// GPS_EOM_flag = 1;
			// }
			
			// *RxWriteIndex = temp;
			// RxWriteIndex++;
			
			// if (RxWriteIndex >= ReceiveBuffer + RxBufferSize)
			// {
				// RxWriteIndex = ReceiveBuffer;
				// BuffCompletellyFilled = 1;
			// }
		// }
	// }
}

CY_ISR(MyTimerISR)
{
	//When the Timer has finished counting , just restart it
    Timer_MATLAB_Stop();
    Timer_MATLAB_Start();
    TimerRestarted = 1;
	LastTime = 0;//LastTime - TIMER_PERIOD/CLOCK_FREQUENCY;
	//Don't need to clear the ISR because it is connected to the TC port

}


void GlobalInit_ToMatlab(void)
{
    UART_MATLAB_Start();
    UART_MATLAB_1_Start();
	Timer_MATLAB_Start();
}


void KeepTimeLoop(void)
{
    do
	{
		ThisTime = ((float)(TIMER_PERIOD - Timer_MATLAB_ReadCounter()) / CLOCK_FREQUENCY);
		CyDelay(1);
	}while (((ThisTime - LastTime) < DESIRED_DT) && (TimerRestarted == 0)); // keep track of time, make DT constant
    TimerRestarted = 0;
}

void GetDataFromIMU(void)
{
            LSM303_ReadMag(&magRawData);       // reads the magnetometers
			MagValues = megRawToMicroTesla(magRawData);
			
			
			/* Apply calibration on Cpss */
			MagValues.x = (MagValues.x-KCx_0)/KCx_1;
			MagValues.y = (MagValues.y-KCy_0)/KCy_1;
			MagValues.z = (MagValues.z-KCz_0)/KCz_1;
			
			LSM303_ReadAccel(&accelRawData);   // reads the accelerometers
			AccValues = accelRawToMeterPerSecond(accelRawData);
			
			/* Apply calibration on Acc */
			AccValues.x = KAx_0+KAx_1*AccValues.x+KAx_2*AccValues.x*AccValues.x+KAx_3*AccValues.x*AccValues.x*AccValues.x+KAx_C*AccValues.y+KAx_P*AccValues.z+KAx_SC*AccValues.x*AccValues.y+KAx_SP*AccValues.x*AccValues.z - ZUPT_Acc.x;
			AccValues.y = KAy_0+KAy_1*AccValues.y+KAy_2*AccValues.y*AccValues.y+KAy_3*AccValues.y*AccValues.y*AccValues.y+KAy_C*AccValues.z+KAy_P*AccValues.x+KAy_SC*AccValues.y*AccValues.z+KAy_SP*AccValues.y*AccValues.x - ZUPT_Acc.y;
			AccValues.z = KAz_0+KAz_1*AccValues.z+KAz_2*AccValues.z*AccValues.z+KAz_3*AccValues.z*AccValues.z*AccValues.z+KAz_C*AccValues.x+KAz_P*AccValues.y+KAz_SC*AccValues.z*AccValues.x+KAz_SP*AccValues.z*AccValues.y - ZUPT_Acc.z;
			
			//lastA1.x = lastA2.x; lastA2.x = lastA3.x; lastA3.x = AccValues.x;
			//AccValues.x = lastA1.x/3.0 + lastA2.x/3.0 + lastA3.x/3.0;
			//
			//lastA1.y = lastA2.y; lastA2.y = lastA3.y; lastA3.y = AccValues.y;
			//AccValues.y = lastA1.y/3.0 + lastA2.y/3.0 + lastA3.y/3.0;
			//
			//lastA1.z = lastA2.z; lastA2.z = lastA3.z; lastA3.z = AccValues.z;
			//AccValues.z = lastA1.z/3.0 + lastA2.z/3.0 + lastA3.z/3.0;
			
			
			L3GD20_ReadGyro(&gyroRawData);       // reads the gyroscopes
			GyroValues = gyrRawToRadPerSec(gyroRawData);
			
			/* Apply calibration on Gyr */
			GyroValues.x = KGx_0+KGx_1*GyroValues.x+KGx_2*GyroValues.x*GyroValues.x+KGx_3*GyroValues.x*GyroValues.x*GyroValues.x+KGx_C*GyroValues.y+KGx_P*GyroValues.z+KGx_SC*GyroValues.x*GyroValues.y+KGx_SP*GyroValues.x*GyroValues.z - ZUPT_Gyro.x;
			GyroValues.y = KGy_0+KGy_1*GyroValues.y+KGy_2*GyroValues.y*GyroValues.y+KGy_3*GyroValues.y*GyroValues.y*GyroValues.y+KGy_C*GyroValues.z+KGy_P*GyroValues.x+KGy_SC*GyroValues.y*GyroValues.z+KGy_SP*GyroValues.y*GyroValues.x - ZUPT_Gyro.y;
			GyroValues.z = KGz_0+KGz_1*GyroValues.z+KGz_2*GyroValues.z*GyroValues.z+KGz_3*GyroValues.z*GyroValues.z*GyroValues.z+KGz_C*GyroValues.x+KGz_P*GyroValues.y+KGz_SC*GyroValues.z*GyroValues.x+KGz_SP*GyroValues.z*GyroValues.y - ZUPT_Gyro.z;
			
			//lastA1.x = lastG2.x; lastG2.x = lastG3.x; lastG3.x = GyroValues.x;
			//GyroValues.x = lastG1.x/3.0 + lastG2.x/3.0 + lastG3.x/3.0;
			//
			//lastG1.y = lastG2.y; lastG2.y = lastG3.y; lastG3.y = GyroValues.y;
			//GyroValues.y = lastG1.y/3.0 + lastG2.y/3.0 + lastG3.y/3.0;
			//
			//lastG1.z = lastG2.z; lastG2.z = lastG3.z; lastG3.z = GyroValues.z;
			//GyroValues.z = lastG1.z/3.0 + lastG2.z/3.0 + lastG3.z/3.0;
			
			/* Fill the nav struct */
			Nav.Longitude = GPS.longitudeDegrees;
			Nav.Latitude = GPS.latitudeDegrees;
			Nav.Altitude = 0;//temp
			
			Nav.AccX = AccValues.x;	Nav.GyrX = GyroValues.x;
			Nav.AccY = AccValues.y;	Nav.GyrY = GyroValues.y;
			Nav.AccZ = AccValues.z;	Nav.GyrZ = GyroValues.z;

			
			/* Temperatures */
			Temp_magneto = LSM303_ReadTeperature(); // reads and converts the MAGNETO temperature
			Temp_gyro = L3GD20_ReadTeperature(); // reads and converts the GYRO temperature
}


// // // // // // // // // // // // // // // // // // // // void GentleTurn(Nav_Struct *Nav, float DeltaTRK, int ContinueLastSpeed) //differential steering orders
// // // // // // // // // // // // // // // // // // // // {
	// // // // // // // // // // // // // // // // // // // // //possible improvement because DeltaTRK is already in the Nav struct
	
	// // // // // // // // // // // // // // // // // // // // if (DeltaTRK > 0)//Turn Starboard
	// // // // // // // // // // // // // // // // // // // // {
		// // // // // // // // // // // // // // // // // // // // if (floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK) > 0)
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <--
			// // // // // // // // // // // // // // // // // // // // //2 <----
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed((int)floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK));// Slow down the motor1
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(Nav->RequestedSpeed); //Keep the speed for motor2
		// // // // // // // // // // // // // // // // // // // // }
		// // // // // // // // // // // // // // // // // // // // else if (floor(Nav->RequestedSpeed + TRKSpeedRatio*DeltaTRK) > 0)
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <----
			// // // // // // // // // // // // // // // // // // // // //2 <------
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(Nav->RequestedSpeed);// Keep the speed for motor1
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(floor(Nav->RequestedSpeed+TRKSpeedRatio*DeltaTRK)); //Accelerate motor2
			
		// // // // // // // // // // // // // // // // // // // // }
		// // // // // // // // // // // // // // // // // // // // else // we split the apple in 2
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <---
			// // // // // // // // // // // // // // // // // // // // //2 <-----
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK/2));// Slow down the motor1
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(floor(Nav->RequestedSpeed+TRKSpeedRatio*DeltaTRK/2)); //Accelerate motor2
		// // // // // // // // // // // // // // // // // // // // }
	// // // // // // // // // // // // // // // // // // // // }
	// // // // // // // // // // // // // // // // // // // // else if (DeltaTRK < 0)//Turn Port
	// // // // // // // // // // // // // // // // // // // // {
		// // // // // // // // // // // // // // // // // // // // if (floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK) > 0)
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <----
			// // // // // // // // // // // // // // // // // // // // //2     <--
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK));// Slow down the motor2
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(Nav->RequestedSpeed); //Keep the speed for motor1
		// // // // // // // // // // // // // // // // // // // // }
		// // // // // // // // // // // // // // // // // // // // else if (floor(Nav->RequestedSpeed + TRKSpeedRatio*DeltaTRK) > 0)
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <----
			// // // // // // // // // // // // // // // // // // // // //2     <--
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(Nav->RequestedSpeed);// Keep the speed for motor2
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(floor(Nav->RequestedSpeed+TRKSpeedRatio*DeltaTRK)); //Accelerate motor1
			
		// // // // // // // // // // // // // // // // // // // // }
		// // // // // // // // // // // // // // // // // // // // else // we split the apple in 2
		// // // // // // // // // // // // // // // // // // // // {
			// // // // // // // // // // // // // // // // // // // // //1   <-----
			// // // // // // // // // // // // // // // // // // // // //2     <---
			// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(floor(Nav->RequestedSpeed-TRKSpeedRatio*DeltaTRK/2));// Slow down the motor2
			// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(floor(Nav->RequestedSpeed+TRKSpeedRatio*DeltaTRK/2)); //Accelerate motor1
		// // // // // // // // // // // // // // // // // // // // }
		
	// // // // // // // // // // // // // // // // // // // // }
	// // // // // // // // // // // // // // // // // // // // else // DeltaTRK ==0
	// // // // // // // // // // // // // // // // // // // // {
		// // // // // // // // // // // // // // // // // // // // motor_1_Set_Speed(Nav->RequestedSpeed);// Keep the speed for motor1
		// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(Nav->RequestedSpeed); //Keep the speed for motor2
	// // // // // // // // // // // // // // // // // // // // }
	// // // // // // // // // // // // // // // // // // // // if (ContinueLastSpeed)
	// // // // // // // // // // // // // // // // // // // // {
		// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(Nav->RequestedSpeed);
		// // // // // // // // // // // // // // // // // // // // motor_2_Set_Speed(Nav->RequestedSpeed);
	// // // // // // // // // // // // // // // // // // // // }
// // // // // // // // // // // // // // // // // // // // }

void Advance(int speed, float TimeToMove)
{
	if (abs(speed) < 96)
	{
		PWM_Enable_Write(0);
		motor_1_Set_Speed(abs(speed));
		motor_2_Set_Speed(abs(speed));
		PWM_Enable_Write(1);
		CyDelay(TimeToMove);

	}
}

void GoBack(int speed, float TimeToMove)
{

	if (abs(speed) < 96)
	{
		PWM_Enable_Write(0);
		motor_1_Set_Speed(-1*abs(speed));
		motor_2_Set_Speed(-1*abs(speed));
		PWM_Enable_Write(1);
		CyDelay(TimeToMove);

	}

}


void Init_PWMs(void)
{
    PWM_Enable_Write(0);// Disable PWM so motors don't start unexpectedly
    
	PWM_Left_Start();
	PWM_Right_Start();
	
	Direction_Left_Write(0);
	Direction_Right_Write(0);
    
    motor_1_Set_Speed(0);
    motor_2_Set_Speed(0);
}

void motor_1_Set_Speed(int speed)
{ // speed is from -250 to 250, positive is CW, negative is CCW 
    if ((speed < 251) & (speed > -251)) //Check that the desired speed is between acceptable boundaries
    {
		if (speed == 0)
		{
			Direction_Right_Write(0);
			PWM_Right_WriteCompare(0);
		}
		if (speed < 0)
		{
			Direction_Right_Write(1);
			PWM_Right_WriteCompare(abs(speed));
		}
		if (speed > 0)
		{
			Direction_Right_Write(0);
			PWM_Right_WriteCompare(speed);
		}
	}
}  

void motor_2_Set_Speed(int speed)
{ // speed is from -250 to 250, positive is CW, negative is CCW 
    if ((speed < 251) & (speed > -251)) //Check that the desired speed is between acceptable boundaries
    {
		if (speed == 0)
		{
			Direction_Left_Write(0);
			PWM_Left_WriteCompare(0);
		}
		if (speed < 0)
		{
			Direction_Left_Write(1);
			PWM_Left_WriteCompare(abs(speed));
		}
		if (speed > 0)
		{
			Direction_Left_Write(0);
			PWM_Left_WriteCompare(speed);
		}
	}
}

void motor_Regulate_Speed(int RequestedSpeed_1, int RequestedSpeed_2)
{// Avoid by a simple P regulator, P is a "define" variable
    int8 ActualSpeed_1 = PWM_Right_ReadCompare();
    int8 OutPutSpeed_1;
	
	int8 ActualSpeed_2 = PWM_Left_ReadCompare();
    int8 OutPutSpeed_2;
    
	// Mot1-----------------------------------------------------------------------------
	if (RequestedSpeed_1 >  ActualSpeed_1)
	{
		OutPutSpeed_1 = ActualSpeed_1 + SpeedMaxVariation * abs(ActualSpeed_1-RequestedSpeed_1);
	}
	else if (RequestedSpeed_1 <  ActualSpeed_1)
	{
		OutPutSpeed_1 = ActualSpeed_1 - SpeedMaxVariation * abs(ActualSpeed_1-RequestedSpeed_1);
	}
	
	// Mot2-----------------------------------------------------------------------------
	if (RequestedSpeed_2 >  ActualSpeed_2)
	{
		OutPutSpeed_2 = ActualSpeed_2 + SpeedMaxVariation * abs(ActualSpeed_2-RequestedSpeed_2);
	}
	else if (RequestedSpeed_2 <  ActualSpeed_2)
	{
		OutPutSpeed_2 = ActualSpeed_2 - SpeedMaxVariation * abs(ActualSpeed_2-RequestedSpeed_2);
	}
      
	PWM_Enable_Write(0);
    motor_1_Set_Speed(OutPutSpeed_1);
	motor_2_Set_Speed(OutPutSpeed_2);
	PWM_Enable_Write(1);
    
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////IMU//////////////////////////////////////////////////////////////////////

void PreTest(void)
{
    if (WaitForWarmUp)
	{
		Pin_Led_Write(1);
        
		CyDelay(100);
		while (cpt_WaitForWarmUp < WarmUpLength) // 
		{
            LSM303_ReadMag(&magRawData);       // reads the magnetometers
			
			LSM303_ReadAccel(&accelRawData);   // reads the accelerometers
			
			L3GD20_ReadGyro(&gyroRawData);     // reads the gyroscopes
			

			//UART_Transmit_Frame(&GPS, AccValues.x, AccValues.y, AccValues.z, ZUPT_Gyro.x , ZUPT_Gyro.y, ZUPT_Gyro.z, GyroValues.x, GyroValues.y, GyroValues.z, 25, 0.1);
			CyDelay(100);
			cpt_WaitForWarmUp ++;	
		}
    }
    
        Pin_Led_Write(0);
        CyDelay(100);
        Pin_Led_Write(1);
    
    
	if (CalibrateAtStartUp)
	{
		Pin_Led_Write(1);
		CyDelay(1000);
		while (cpt_CalibrateAtStartUp < ZUPTLength) // sec ZUPT
		{
			
			LSM303_ReadAccel(&accelRawData);   // reads the accelerometers
			AccValues = accelRawToMeterPerSecond(accelRawData);
			
			/* Apply calibration on Acc */
			ZUPT_Acc.x = ZUPT_Acc.x + (KAx_0+KAx_1*AccValues.x+KAx_2*AccValues.x*AccValues.x+KAx_3*AccValues.x*AccValues.x*AccValues.x+KAx_C*AccValues.y+KAx_P*AccValues.z+KAx_SC*AccValues.x*AccValues.y+KAx_SP*AccValues.x*AccValues.z)/ZUPTLength;
			ZUPT_Acc.y = ZUPT_Acc.y + (KAy_0+KAy_1*AccValues.y+KAy_2*AccValues.y*AccValues.y+KAy_3*AccValues.y*AccValues.y*AccValues.y+KAy_C*AccValues.z+KAy_P*AccValues.x+KAy_SC*AccValues.y*AccValues.z+KAy_SP*AccValues.y*AccValues.x)/ZUPTLength;
			ZUPT_Acc.z = ZUPT_Acc.z + (KAz_0+KAz_1*AccValues.z+KAz_2*AccValues.z*AccValues.z+KAz_3*AccValues.z*AccValues.z*AccValues.z+KAz_C*AccValues.x+KAz_P*AccValues.y+KAz_SC*AccValues.z*AccValues.x+KAz_SP*AccValues.z*AccValues.y)/ZUPTLength;
			
			
			L3GD20_ReadGyro(&gyroRawData);       // reads the gyroscopes
			GyroValues = gyrRawToRadPerSec(gyroRawData);
			
			/* Apply calibration on Gyr */
			ZUPT_Gyro.x = ZUPT_Gyro.x + (KGx_0+KGx_1*GyroValues.x+KGx_2*GyroValues.x*GyroValues.x+KGx_3*GyroValues.x*GyroValues.x*GyroValues.x+KGx_C*GyroValues.y+KGx_P*GyroValues.z+KGx_SC*GyroValues.x*GyroValues.y+KGx_SP*GyroValues.x*GyroValues.z)/ZUPTLength;
			ZUPT_Gyro.y = ZUPT_Gyro.y + (KGy_0+KGy_1*GyroValues.y+KGy_2*GyroValues.y*GyroValues.y+KGy_3*GyroValues.y*GyroValues.y*GyroValues.y+KGy_C*GyroValues.z+KGy_P*GyroValues.x+KGy_SC*GyroValues.y*GyroValues.z+KGy_SP*GyroValues.y*GyroValues.x)/ZUPTLength;
			ZUPT_Gyro.z = ZUPT_Gyro.z + (KGz_0+KGz_1*GyroValues.z+KGz_2*GyroValues.z*GyroValues.z+KGz_3*GyroValues.z*GyroValues.z*GyroValues.z+KGz_C*GyroValues.x+KGz_P*GyroValues.y+KGz_SC*GyroValues.z*GyroValues.x+KGz_SP*GyroValues.z*GyroValues.y)/ZUPTLength;
			

			//UART_Transmit_Frame(&GPS, AccValues.x, AccValues.y, AccValues.z, ZUPT_Gyro.x , ZUPT_Gyro.y, ZUPT_Gyro.z, GyroValues.x, GyroValues.y, GyroValues.z, 25, 0.1);
			CyDelay(100);
			cpt_CalibrateAtStartUp ++;

			
		}
		ZUPT_Acc.z = ZUPT_Acc.z - GRAVITY;
		Pin_Led_Write(0);
		
		
		cpt_CalibrateAtStartUp = 0;
		// Filter ZUPT, cannot be done in the same time as the zupt 
		Pin_Led_Write(1);
		CyDelay(1000);
		while (cpt_CalibrateAtStartUp < ZUPTLength) // sec ZUPT
		{
			GetDataFromIMU();
		
			/* Filter */
			ArandomThing = ComplementaryFilter1(AccValues, MagValues, GyroValues, ArandomThing, ThisTime - LastTime);
			
			
			ZUPT_ArandomThing.x = ZUPT_ArandomThing.x + ArandomThing.x /ZUPTLength;
			ZUPT_ArandomThing.y = ZUPT_ArandomThing.y + ArandomThing.y/ZUPTLength; 
			ZUPT_ArandomThing.z = ZUPT_ArandomThing.z + ArandomThing.z /ZUPTLength;
			
			//UART_Transmit_Frame(&GPS, AccValues.x, AccValues.y, AccValues.z, ZUPT_Gyro.x , ZUPT_Gyro.y, ZUPT_Gyro.z, GyroValues.x, GyroValues.y, GyroValues.z, 25, 0.1);
			CyDelay(100);
			cpt_CalibrateAtStartUp ++;

			
		}
		ZUPT_Acc.z = ZUPT_Acc.z - GRAVITY;
		Pin_Led_Write(0);
		
		
		
		
		
		
		
		
	}
}

void LSM303_ReadMag(Vector3 *magData)
{

	uint8  statusStart1;
	uint8 xhi = 0x00u;
	uint8 xlo = 0x00u;
	uint8 zhi = 0x00u;
	uint8 zlo = 0x00u;
	uint8 yhi = 0x00u;
	uint8 ylo = 0x00u;
	char output1[4];
	uint8 statusStart0, statusWrite, statusStop0, statusStop1;
	uint8 dataRDY = 17;    
	//Prepare the module for magnetometer output.
	statusStart0 = I2C_MasterSendStart(LSM303_ADDRESS_MAG, I2C_WRITE_XFER_MODE);
	statusWrite = I2C_MasterWriteByte(LSM303_REGISTER_MAG_OUT_X_H_M);       
	statusStop0 = I2C_MasterSendStop();

	//--    
	/*Read Data Manually*/
	//Start the communication in read mode.
	statusStart1 = I2C_MasterSendStart(LSM303_ADDRESS_MAG, I2C_READ_XFER_MODE);   
	if(statusStart1==I2C_MSTR_NO_ERROR){
		xhi = I2C_MasterReadByte(I2C_ACK_DATA);
		xlo = I2C_MasterReadByte(I2C_ACK_DATA);
		zhi = I2C_MasterReadByte(I2C_ACK_DATA);
		zlo = I2C_MasterReadByte(I2C_ACK_DATA);
		yhi = I2C_MasterReadByte(I2C_ACK_DATA);
		ylo = I2C_MasterReadByte(I2C_NAK_DATA);    
	}
	else{
		//        PC_UART_PutString("Fail to read Meg_Data/n");
	}
	statusStop1 = I2C_MasterSendStop();
	
	dataRDY = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_SR_REG_Mg);
	
	uint8 mode = 2;
	//try to read back Meg-setting
	mode = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_MR_REG_M); 
	
	uint8 control_Reg = 0x00;
	//try to read back the control_reg
	control_Reg = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A);
	
	//Read WHOAMI from CRA_REG_M
	uint8 reg1_a = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_CRA_REG_M);
	
	//Read Gain Reg
	uint8 reg_gain = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_CRB_REG_M);

	
	//Put together the different data bytes to form complete
	//integers for storing the output values.
	magData->x = (int16_t)(xlo | (xhi << 8));
	magData->y = (int16_t)(ylo | (yhi << 8));   
	magData->z = (int16_t)(zlo | (zhi << 8));

}



void LSM303_ReadAccel(Vector3 *accelData)
{

	uint8  statusStart1;
	uint8 xhi = 0x00u;
	uint8 xlo = 0x00u;
	uint8 zhi = 0x00u;
	uint8 zlo = 0x00u;
	uint8 yhi = 0x00u;
	uint8 ylo = 0x00u;

	//Prepare the module for Accelelameter output.
	I2C_MasterSendStart(LSM303_ADDRESS_ACCEL, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);       
	I2C_MasterSendStop();
	
	/*Read Data Manually
	//Start the communication in read mode.*/
	statusStart1 = I2C_MasterSendStart(LSM303_ADDRESS_ACCEL, I2C_READ_XFER_MODE);   
	if(statusStart1==I2C_MSTR_NO_ERROR)
	{
		xlo = I2C_MasterReadByte(I2C_ACK_DATA);
		xhi = I2C_MasterReadByte(I2C_ACK_DATA);
		ylo = I2C_MasterReadByte(I2C_ACK_DATA);
		yhi = I2C_MasterReadByte(I2C_ACK_DATA);
		zlo = I2C_MasterReadByte(I2C_ACK_DATA);
		zhi = I2C_MasterReadByte(I2C_NAK_DATA);    
	}
	else
	{

	}
	I2C_MasterSendStop();
	
	// Shift values to create properly formed integer (low byte first)
	accelData->x = (int16_t)(xlo | (xhi << 8)) >> 4;
	accelData->y = (int16_t)(ylo | (yhi << 8)) >> 4;
	accelData->z = (int16_t)(zlo | (zhi << 8)) >> 4;  
}

void L3GD20_ReadGyro(Vector3 *gyroData)
{     
	uint8  statusStart1;
	uint8 xhi = 0x00u;
	uint8 xlo = 0x00u;
	uint8 zhi = 0x00u;
	uint8 zlo = 0x00u;
	uint8 yhi = 0x00u;
	uint8 ylo = 0x00u;
	
	//Prepare the module for Accelelameter output.
	I2C_MasterSendStart(L3GD20_ADDRESS, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(GYRO_REGISTER_OUT_X_L | 0x80);       
	I2C_MasterSendStop();
	
	/*Read Data Manually
	//Start the communication in read mode.*/
	statusStart1 = I2C_MasterSendStart(L3GD20_ADDRESS, I2C_READ_XFER_MODE);   
	if(statusStart1==I2C_MSTR_NO_ERROR){
		xlo = I2C_MasterReadByte(I2C_ACK_DATA);
		xhi = I2C_MasterReadByte(I2C_ACK_DATA);
		ylo = I2C_MasterReadByte(I2C_ACK_DATA);
		yhi = I2C_MasterReadByte(I2C_ACK_DATA);
		zlo = I2C_MasterReadByte(I2C_ACK_DATA);
		zhi = I2C_MasterReadByte(I2C_NAK_DATA);    
	}
	else{
		//        PC_UART_PutString("Fail to read Accel_Data/n");
	}
	I2C_MasterSendStop();
	
	/* Shift values to create properly formed integer (low byte first) */
	gyroData->x = (int16_t)(xlo | (xhi << 8));
	gyroData->y = (int16_t)(ylo | (yhi << 8));
	gyroData->z = (int16_t)(zlo | (zhi << 8));

}

Vector3f gyrRawToRadPerSec(Vector3 raw)
{
	Vector3f gyroData;
	
	int range = readFromSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG4);
	/* Compensate values depending on the resolution */
	switch(range)
	{
	case 0x00:
		gyroData.x = GYRO_SENSITIVITY_250DPS * raw.x;
		gyroData.y = GYRO_SENSITIVITY_250DPS * raw.y;
		gyroData.z = GYRO_SENSITIVITY_250DPS * raw.z;
		break;
	case 0x10:
		gyroData.x = GYRO_SENSITIVITY_500DPS * raw.x;
		gyroData.y = GYRO_SENSITIVITY_500DPS * raw.y;
		gyroData.z = GYRO_SENSITIVITY_500DPS * raw.z;
		break;
	case 0x20:
		gyroData.x = GYRO_SENSITIVITY_2000DPS * raw.x;
		gyroData.y = GYRO_SENSITIVITY_2000DPS * raw.y;
		gyroData.z = GYRO_SENSITIVITY_2000DPS * raw.z;
		break;
	}
	/* Convert values to rad/s */
	gyroData.x *= SENSORS_DPS_TO_RADS;
	gyroData.y *= SENSORS_DPS_TO_RADS;
	gyroData.z *= SENSORS_DPS_TO_RADS;

	return gyroData;
}

//void readSettingReg()//NOT USED
//{
//	uint8 reg;
//	reg = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A);
//
//	
//	reg = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_MAG_CRA_REG_M);
//	//    PC_UART_PutString("LSM303_REGISTER_MAG_CRA_REG_M :");
//	//    sprintf(output1, "0x%x\n", reg);
//	//    PC_UART_PutString(output1);
//	
//	reg = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_MAG_CRB_REG_M);
//	//    PC_UART_PutString("LSM303_REGISTER_MAG_CRB_REG_M :");
//	//    sprintf(output1, "0x%x\n", reg);
//	//    PC_UART_PutString(output1);
//	
//	reg = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_MAG_MR_REG_M);
//	//    PC_UART_PutString("LSM303_REGISTER_MAG_MR_REG_M :");
//	//    sprintf(output1, "0x%x\n", reg);
//	//    PC_UART_PutString(output1);
//}

Vector3f megRawToMicroTesla(Vector3 raw)
{    
	uint8 gain;
	Vector3f microTesla;
	float lsm303Mag_Gauss_LSB_XY,lsm303Mag_Gauss_LSB_Z;
	gain = readFromSlaveReg(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M);
	switch(gain)
	{
	case LSM303_MAGGAIN_1_3:
		lsm303Mag_Gauss_LSB_XY = 1100;
		lsm303Mag_Gauss_LSB_Z  = 980;
		break;
	case LSM303_MAGGAIN_1_9:
		lsm303Mag_Gauss_LSB_XY = 855;
		lsm303Mag_Gauss_LSB_Z  = 760;
		break;
	case LSM303_MAGGAIN_2_5:
		lsm303Mag_Gauss_LSB_XY = 670;
		lsm303Mag_Gauss_LSB_Z  = 600;
		break;
	case LSM303_MAGGAIN_4_0:
		lsm303Mag_Gauss_LSB_XY = 450;
		lsm303Mag_Gauss_LSB_Z  = 400;
		break;
	case LSM303_MAGGAIN_4_7:
		lsm303Mag_Gauss_LSB_XY = 400;
		lsm303Mag_Gauss_LSB_Z  = 355;
		break;
	case LSM303_MAGGAIN_5_6:
		lsm303Mag_Gauss_LSB_XY = 330;
		lsm303Mag_Gauss_LSB_Z  = 295;
		break;
	case LSM303_MAGGAIN_8_1:
		lsm303Mag_Gauss_LSB_XY = 230;
		lsm303Mag_Gauss_LSB_Z  = 205;
		break;
	}
	microTesla.x = raw.x / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	microTesla.y = raw.y / lsm303Mag_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
	microTesla.z = raw.z / lsm303Mag_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;
	return microTesla;
}

float megRawToDegree(Vector3 raw)
{
	float degree = (atan2(raw.y, raw.x) * 180.0) / 3.14159265359;        
	//Map the value to comply with the standard system where the heading
	//is specified as an angle between 0 - 360 degrees.
	if(degree < 0) {
		degree = 360 + degree;   
	}    
	return degree;
}

Vector3f accelRawToMeterPerSecond(Vector3 raw)
{
	float lsm303Accel_MG_LSB     = 0.001F;   // 1, 2, 4 or 12 mg per lsb
	Vector3f gravity;
	gravity.x = raw.x * lsm303Accel_MG_LSB * SENSORS_GRAVITY_EARTH;
	gravity.y = raw.y * lsm303Accel_MG_LSB * SENSORS_GRAVITY_EARTH;
	gravity.z = raw.z * lsm303Accel_MG_LSB * SENSORS_GRAVITY_EARTH;
	
	return gravity;
}

Vector3f ComplementaryFilter1(Vector3f accData, Vector3f magData, Vector3f gyrData, Vector3f attitudeData, float dt)
{
	float pitchAcc, rollAcc, HDGMag;
    float debugA=0, debugB=0;
    // 
	rollAcc = atan2f((float)accData.y, (float)accData.z);
    attitudeData.x = (((float)gyrData.x ) * dt + attitudeData.x )*-1 * CoeffRoll + rollAcc * (1 - CoeffRoll); // attitudeData.x +   
    
    
    // 
	pitchAcc = atan2f((float)accData.x*-1, (float)accData.z);
	attitudeData.y = (((float)gyrData.y*-1 ) * dt + attitudeData.y) * CoeffPitch + pitchAcc*-1.0 * (1 - CoeffPitch); //attitudeData.y + 
    
	// 
	//HDGMag = atan2f((float)magData.y* cos(rollAcc)-(float)magData.z* sin(rollAcc)*cos(pitchAcc), ((float)magData.x* cos(pitchAcc) + (float)magData.z * sin(pitchAcc)* cos(rollAcc)))+pitchAcc; //(float)magData.x* sin(pitchAcc)* sin(rollAcc)+
	pitchAcc = asinf(-1*accData.x/SENSORS_GRAVITY_EARTH);
    rollAcc = asinf((accData.y/SENSORS_GRAVITY_EARTH)/cos(pitchAcc));

    debugA = fabs(((float)magData.x* cos(pitchAcc) + (float)magData.z * sin(pitchAcc)));
    debugB = fabs((float)magData.x* sin(pitchAcc)* sin(rollAcc)+(float)magData.y* cos(rollAcc)-(float)magData.z* sin(rollAcc)*cos(pitchAcc));
    if(isnan(debugA) | isnan(debugB))
    {
        attitudeData.z = 0;//(((float)gyrData.z) * dt + attitudeData.z);
    }
    else
    {
        HDGMag = atan2f((float)magData.x* sin(pitchAcc)* sin(rollAcc)+(float)magData.y* cos(rollAcc)-(float)magData.z * sin(rollAcc)*cos(pitchAcc), ((float)magData.x * cos(pitchAcc) + (float)magData.z * sin(pitchAcc))); //(float)magData.x* sin(pitchAcc)* sin(rollAcc)+
        //HDGMag = ata2f((float)magData.y,(float)magData.x);
        attitudeData.z = (((float)gyrData.z*-1 ) * dt + attitudeData.z) * CoeffHDG + HDGMag * (1 - CoeffHDG); //attitudeData.z +
    }

    //attitudeData.z = PI - attitudeData.z;

	return attitudeData;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////INIT----TEST//////////////////////////////////////////////////////////////////////







void TestPWM(void)
{
	CyDelay(1000);
}

void InitEntireSystem (void)
{
    //Motors
    //----------------------
    Init_PWMs();
    UART_MATLAB_PutString("PWMs initialised\r\n");
	UART_MATLAB_PutString("Starting PWMs test\r\n");
	TestPWM();
    
    //IMU
    //----------------------
    GlobalInit_IMU(GYRO_RANGE_250DPS);
    UART_MATLAB_PutString("IMU intialised\r\n");
    
	//ToMatlab
    //----------------------
    GlobalInit_ToMatlab();
	TimerPeriod = Timer_MATLAB_ReadPeriod();
    UART_MATLAB_PutString("Broadcast initialised\r\n");

	//Init LastTime for sending data to Matlab
	ThisTime  = ((float)(TIMER_PERIOD - Timer_MATLAB_ReadCounter()) / CLOCK_FREQUENCY);
	
	//Timer ISR
	//----------------------
	isr_TC_StartEx(MyTimerISR);
	
	//Base station orders ISR
	//----------------------
	Rx_Int_DGPS_StartEx(MyRxIntDGPS);
    
    CYGlobalIntEnable; //Enable the Global interrupts
    
    //WaitUntilGPSFix();

}

/* DGPS UART */
uint8	IsCharReadyDGPS(void)
{
	return !(RxWriteIndexDGPS == RxReadIndexDGPS);
}

char	GetRxCharDGPS(void)
{
	char	ResultDGPS;
	//while(!IsCharReady()) Wait();
	ResultDGPS = *RxReadIndexDGPS;
	//*RxReadIndex =0;
	RxReadIndexDGPS++;
	if (RxReadIndexDGPS >= ReceiveBufferDGPS + RxBufferSizeDGPS) RxReadIndexDGPS = ReceiveBufferDGPS;
	return ResultDGPS;
}

CY_ISR(MyRxIntDGPS)
{

	volatile char tempDGPS = {0};
	while(UART_MATLAB_ReadRxStatus() & UART_MATLAB_RX_STS_FIFO_NOTEMPTY)
	//if(1)
	{
		tempDGPS  = UART_MATLAB_ReadRxData();

		if (tempDGPS != 0x00)
		{
			if (tempDGPS == 0x0A)
			{
                
				DGPS_EOM_flag = 1;
			}
			
			*RxWriteIndexDGPS = tempDGPS;
			RxWriteIndexDGPS++;
			
			if (RxWriteIndexDGPS >= ReceiveBufferDGPS + RxBufferSizeDGPS)
			{
				RxWriteIndexDGPS = ReceiveBufferDGPS;
			}
		}
	}
}






void ParseDGPS (GPS_Struct *GPS)
{
	if (strstr(GPS->DGPSmessage, "$"))
	{
		
		// found DGPS
		char *p = GPS->DGPSmessage;

		// get PWM1 order
		p = strchr(p, '$')+1;
		GPS->PWM_Left_Orders = atoi(p);
		
		// get PWM2 order
		p = strchr(p, ',')+1;
		GPS->PWM_Right_Orders = atoi(p);

	}
}


/* [] END OF FILE */