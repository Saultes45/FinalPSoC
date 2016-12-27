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

#include "Broadcast.h"
#include <stdio.h>
#include <string.h>


void UART_Transmit_Frame(float gravX, float gravY, float gravZ, float magDataX, float magDataY, float magDataZ, float gyroDataX, float gyroDataY, float gyroDataZ,float Temp, float Dt2)
{

	char My_String [10] ={0};

    Pin_Led_Write(1);
	//Transmit Message through USB
	
	
	///////////////////////////////////////////////////////////9600///////////////////////////////////////////////////
	// Data Check
	UART_MATLAB_PutString("PSoC:");
	
	//PWM1
	sprintf (My_String, "%i;", PWM_Right_ReadCompare());
	UART_MATLAB_PutString(My_String);
    
    //PWM2
    sprintf (My_String, "%i;", PWM_Left_ReadCompare());
	UART_MATLAB_PutString(My_String);
    
    //Motor Enabled ?
    sprintf (My_String, "%i;", PWM_Enable_Read());
	UART_MATLAB_PutString(My_String);
    
    //Time
    sprintf (My_String, "%f;", Dt2);
	UART_MATLAB_PutString(My_String);
	
	//IMU 9DOF signals
	sprintf (My_String, "%f ;", gravX);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%f ;", gravY);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%f ;", gravZ);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%5.4f ;", magDataX);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%5.4f ;", magDataY);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%5.4f ;", magDataZ);
	UART_MATLAB_PutString(My_String);
    sprintf (My_String, "%5.4f ;",gyroDataX);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%5.4f ;", gyroDataY);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%5.4f ;", gyroDataZ);
	UART_MATLAB_PutString(My_String);
	sprintf (My_String, "%3.1f ;", Temp);
	UART_MATLAB_PutString(My_String);
	
	//Send an END OF LINE character
	UART_MATLAB_PutString("\r\n");
    
    
    
    
    
    
    ///////////////////////////////////////////////////////////115200///////////////////////////////////////////////////
    // // // // Data Check
	// // // UART_MATLAB_1_PutString("PSoC:");
	
	// // // //PWM1
	// // // sprintf (My_String, "%i;", PWM_Right_ReadCompare());
	// // // UART_MATLAB_1_PutString(My_String);
    
    // // // //PWM2
    // // // sprintf (My_String, "%i;", PWM_Left_ReadCompare());
	// // // UART_MATLAB_1_PutString(My_String);
	
    // // // //Time
    // // // sprintf (My_String, "%f;", Dt2);
	// // // UART_MATLAB_1_PutString(My_String);
	
		// // // //IMU 9DOF signals
	// // // sprintf (My_String, "%f ;", gravX);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%f ;", gravY);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%f ;", gravZ);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%5.4f ;", magDataX);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%5.4f ;", magDataY);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%5.4f ;", magDataZ);
	// // // UART_MATLAB_1_PutString(My_String);
    // // // sprintf (My_String, "%5.4f ;",gyroDataX);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%5.4f ;", gyroDataY);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%5.4f ;", gyroDataZ);
	// // // UART_MATLAB_1_PutString(My_String);
	// // // sprintf (My_String, "%3.1f ;", Temp);
	// // // UART_MATLAB_1_PutString(My_String);
	
		
	// // // //Send an END OF LINE character
	// // // UART_MATLAB_1_PutString("\r\n");

    
    
    
    Pin_Led_Write(0);
}


/* [] END OF FILE */
