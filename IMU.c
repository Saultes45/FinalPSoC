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

///////////////////////////////////////////////////////////////////Include/////////////////////////////////////////////////////////////
#include <project.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "IMU.h"

void GlobalInit_IMU(gyroRange_t range)
{ 
	I2C_Start();
	CyDelay(1000);// wait to let time to init the I2C
	LSM303_Init();
    UART_MATLAB_PutString("LSM303_Init\r\n");
	L3GD20_Init(range);// Minimum DPS gives the best accuracy
    UART_MATLAB_PutString("L3GD20_Init\r\n");
}


uint8 readFromSlaveReg(uint8 Slave_Add,uint8 Reg_Add)
{
	uint8 data;
	I2C_MasterSendStart(Slave_Add, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(Reg_Add);            
	I2C_MasterSendStop();    

	I2C_MasterSendStart(Slave_Add, I2C_READ_XFER_MODE);
	data = I2C_MasterReadByte(I2C_NAK_DATA);
	I2C_MasterSendStop();
	return data;
}

void writeToSlaveReg(uint8 Slave_Add,uint8 Reg_Add, uint8 byte)
{    
	I2C_MasterSendStart(Slave_Add, I2C_WRITE_XFER_MODE);
	I2C_MasterWriteByte(Reg_Add);
	I2C_MasterWriteByte(byte);        
	I2C_MasterSendStop();
}

//Gyro Initialize
void L3GD20_Init(gyroRange_t range) 
{
	/* Make sure we have the correct chip ID since this checks
	for correct address and that the IC is properly connected */
	uint8_t id = readFromSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_WHO_AM_I);
	//Serial.println(id, HEX);
	if ((id != L3GD20_ID) & (id != L3GD20H_ID))
	{
		//    PC_UART_PutString("Gyro's ID is not match\n");
	}
	/* Set CTRL_REG1 (0x20)
====================================================================
BIT  Symbol    Description                                   Default
---  ------    --------------------------------------------- -------
7-6  DR1/0     Output data rate                                   00
5-4  BW1/0     Bandwidth selection                                00
	3  PD        0 = Power-down mode, 1 = normal/sleep mode          0
	2  ZEN       Z-axis enable (0 = disabled, 1 = enabled)           1
	1  YEN       Y-axis enable (0 = disabled, 1 = enabled)           1
	0  XEN       X-axis enable (0 = disabled, 1 = enabled)           1 */

	/* Reset then switch to normal mode and enable all three channels */
	writeToSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG1, 0x00);
	writeToSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG1, 0x0F);//95Hz
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG2 (0x21)
====================================================================
BIT  Symbol    Description                                   Default
---  ------    --------------------------------------------- -------
5-4  HPM1/0    High-pass filter mode selection                    00
3-0  HPCF3..0  High-pass filter cutoff frequency selection      0000 */

	/* Nothing to do ... keep default values */
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG3 (0x22)
====================================================================
BIT  Symbol    Description                                   Default
---  ------    --------------------------------------------- -------
	7  I1_Int1   Interrupt enable on INT1 (0=disable,1=enable)       0
	6  I1_Boot   Boot status on INT1 (0=disable,1=enable)            0
	5  H-Lactive Interrupt active config on INT1 (0=high,1=low)      0
	4  PP_OD     Push-Pull/Open-Drain (0=PP, 1=OD)                   0
	3  I2_DRDY   Data ready on DRDY/INT2 (0=disable,1=enable)        0
	2  I2_WTM    FIFO wtrmrk int on DRDY/INT2 (0=dsbl,1=enbl)        0
	1  I2_ORun   FIFO overrun int on DRDY/INT2 (0=dsbl,1=enbl)       0
	0  I2_Empty  FIFI empty int on DRDY/INT2 (0=dsbl,1=enbl)         0 */

	/* Nothing to do ... keep default values */
	/* ------------------------------------------------------------------ */

	/* Set CTRL_REG4 (0x23)
====================================================================
BIT  Symbol    Description                                   Default
---  ------    --------------------------------------------- -------
	7  BDU       Block Data Update (0=continuous, 1=LSB/MSB)         0
	6  BLE       Big/Little-Endian (0=Data LSB, 1=Data MSB)          0
5-4  FS1/0     Full scale selection                               00
								00 = 250 dps
								01 = 500 dps
								10 = 2000 dps
								11 = 2000 dps
	0  SIM       SPI Mode (0=4-wire, 1=3-wire)                       0 */

	/* Adjust resolution if requested */
	switch(range)
	{
	case GYRO_RANGE_250DPS:
		writeToSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG4, 0x00);
		break;
	case GYRO_RANGE_500DPS:
		writeToSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG4, 0x10);
		break;
	case GYRO_RANGE_2000DPS:
		writeToSlaveReg(L3GD20_ADDRESS,GYRO_REGISTER_CTRL_REG4, 0x20);
		break;
	}
}

float LSM303_ReadTeperature(void)
{
	uint8  statusStart1;
	uint8 Temperaturehi = 0x00u;
	uint8 Temperaturelo = 0x00u;
	uint8 statusWrite;
	float TempValue;
	int sign;
	
	//Prepare the module for Temp output.
	I2C_MasterSendStart(LSM303_ADDRESS_MAG, I2C_WRITE_XFER_MODE);
	statusWrite = I2C_MasterWriteByte(LSM303_REGISTER_MAG_TEMP_OUT_H_M);       
	I2C_MasterSendStop();
	
	/*Read Data Manually
	//Start the communication in read mode.*/
	statusStart1 = I2C_MasterSendStart(LSM303_ADDRESS_MAG, I2C_READ_XFER_MODE);   
	if(statusStart1==I2C_MSTR_NO_ERROR)
	{
		Temperaturehi = I2C_MasterReadByte(I2C_ACK_DATA); 
		//If (address pointer = 08) then the address pointer = 03
		//Or else, if (address pointer >= 12) then the address pointer = 0
		//Or else, (address pointer) = (address pointer) + 1
		Temperaturelo = I2C_MasterReadByte(I2C_ACK_DATA);
	}
	else
	{

	}
	I2C_MasterSendStop();
	
	// Shift values to create properly formed integer 
//	uint16 T1= (Temperaturehi << 8);
//	uint16 T2= (Temperaturelo | (Temperaturehi << 8));
//	uint16 T3= (Temperaturelo | (Temperaturehi << 8)) >> 4;
	int tempRawData = (int16_t)(Temperaturelo | (Temperaturehi << 8)) >> 4;
	
	//https://www.adafruit.com/datasheets/LSM303DLHC.PDF
	//p39
	
	if (tempRawData > 0x800) //then the temperature is negative (expressed as 2's complement)
	{
		sign = -1;
		tempRawData = (~(tempRawData) + 0x01);
	}
	else
	{
		sign = 1;
	}

	
	TempValue = sign * SENSORS_LSB_TO_TEMPERATURE * tempRawData + (20.75F);
	
	return TempValue;
}

float L3GD20_ReadTeperature(void)
{
	//Temperature refresh rate 1Hz
	//The product is factory calibrated at 3.0 V.
	uint8  statusStart1;
	uint8 tempRawData = 0x00u;
	//uint8 statusWrite;
	float TempValue;
	int sign;
	
	//Prepare the module for Temp output.
	I2C_MasterSendStart(L3GD20_ADDRESS, I2C_WRITE_XFER_MODE);
	//statusWrite = I2C_MasterWriteByte(GYRO_REGISTER_OUT_TEMP);       
	I2C_MasterSendStop();
	
	/*Read Data Manually
	//Start the communication in read mode.*/
	statusStart1 = I2C_MasterSendStart(L3GD20_ADDRESS, I2C_READ_XFER_MODE);   
	if(statusStart1==I2C_MSTR_NO_ERROR)
	{
		tempRawData = I2C_MasterReadByte(I2C_ACK_DATA);
	}
	else
	{

	}
	I2C_MasterSendStop();
	
	// Shift values to create properly formed integer (low byte first)
	//Temperature data (1LSB/deg - 8-bit resolution). The value is expressed as two's complement.
	
	
	//https://www.adafruit.com/datasheets/L3GD20.pdf
	//p35
	if (tempRawData > 0x80) //then the temperature is negative (expressed as 2's complement)
	{
		sign = -1;
		tempRawData = (~(tempRawData) + 0x01);
	}
	else
	{
		sign = 1;
	}
	TempValue = -1 * sign * TEMP_SENSITIVITY * tempRawData + (38);
	
	return TempValue;
}

//Accelerometer and Megnetometer Initialization
void LSM303_Init() 
{
	
	uint8 statusStart0, statusWrite, statusStop0;  
	
	/*------------ Accelerometer ------------*/
	//Write to control reg
	//Specify which register we want to write to.
	//This register is written to for enabling x, y and z
	//output as well as setting datarate and powermode.
	writeToSlaveReg(LSM303_ADDRESS_ACCEL , LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27u);//ODR 10Hz (before 0x57u)
	
	/*------------ Magnetometer ------------*/

	//This register is written to for setting data output rates
	//and controlling measurement flow. Continous Covertsation Mode
	writeToSlaveReg(LSM303_ADDRESS_MAG , LSM303_REGISTER_MAG_MR_REG_M, 0x00u);
	
	//Write to WHOIAM set Hz update rate and enable temperature reading (p36)// ....
	writeToSlaveReg(LSM303_ADDRESS_MAG , LSM303_REGISTER_MAG_CRA_REG_M, 0x90u);//ODR 15Hz
	//    TEMP_EN = 1
	//    (0)
	//    (0)
	//    DO2 = 1
	//    DO1 = 0
	//    DO2 = 0
	//    (0)
	//    (0)
	//    ==> Hz update
	
	//Write Magnetometer Gain Setting to +/-1.3
	writeToSlaveReg(LSM303_ADDRESS_MAG , LSM303_REGISTER_MAG_CRB_REG_M, 0x20u);
	
	uint8 mode = 2;
	//try to read back Meg-setting
	mode = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_MR_REG_M);
	uint8 HPFilterMode = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_ACCEL_CTRL_REG2_A); 

	//try to read back the control_reg
	char address = readFromSlaveReg(LSM303_ADDRESS_ACCEL,LSM303_REGISTER_ACCEL_CTRL_REG1_A);

	//Read WHOAMI from CRA_REG_M
	uint8 reg1_m = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_CRA_REG_M);
	
	//Read Gain Reg
	uint8 reg_gain = readFromSlaveReg(LSM303_ADDRESS_MAG,LSM303_REGISTER_MAG_CRB_REG_M);
	
}

/* [] END OF FILE */
