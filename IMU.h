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

///////////////////////////////////////////////////////////////////Define/////////////////////////////////////////////////////////////
#define PI          3.14159265359
#define CoeffRoll   0.01
#define CoeffPitch  0.01
#define CoeffHDG    0

#define LSM303_ADDRESS_ACCEL                        (0x32 >> 1)
#define LSM303_ADDRESS_MAG                          (0x3C >> 1)

//Accelerometer_Reg                                     // DEFAULT    TYPE
#define LSM303_REGISTER_ACCEL_CTRL_REG1_A          0x20   // 00000111   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG2_A          0x21   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG3_A          0x22   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG4_A          0x23   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG5_A          0x24   // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG6_A          0x25   // 00000000   rw
#define LSM303_REGISTER_ACCEL_REFERENCE_A          0x26   // 00000000   r
#define LSM303_REGISTER_ACCEL_STATUS_REG_A         0x27   // 00000000   r
#define LSM303_REGISTER_ACCEL_OUT_X_L_A            0x28   //            r
#define LSM303_REGISTER_ACCEL_OUT_X_H_A            0x29   //            r
#define LSM303_REGISTER_ACCEL_OUT_Y_L_A            0x2A   //                
#define LSM303_REGISTER_ACCEL_OUT_Y_H_A            0x2B   
#define LSM303_REGISTER_ACCEL_OUT_Z_L_A            0x2C
#define LSM303_REGISTER_ACCEL_OUT_Z_H_A            0x2D
#define LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A      0x2E   //            rw
#define LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A       0x2F
#define LSM303_REGISTER_ACCEL_INT1_CFG_A           0x30   //            rw
#define LSM303_REGISTER_ACCEL_INT1_SOURCE_A        0x31
#define LSM303_REGISTER_ACCEL_INT1_THS_A           0x32                 
#define LSM303_REGISTER_ACCEL_INT1_DURATION_A      0x33
#define LSM303_REGISTER_ACCEL_INT2_CFG_A           0x34
#define LSM303_REGISTER_ACCEL_INT2_SOURCE_A        0x35
#define LSM303_REGISTER_ACCEL_INT2_THS_A           0x36
#define LSM303_REGISTER_ACCEL_INT2_DURATION_A      0x37
#define LSM303_REGISTER_ACCEL_CLICK_CFG_A          0x38
#define LSM303_REGISTER_ACCEL_CLICK_SRC_A          0x39
#define LSM303_REGISTER_ACCEL_CLICK_THS_A          0x3A
#define LSM303_REGISTER_ACCEL_TIME_LIMIT_A         0x3B
#define LSM303_REGISTER_ACCEL_TIME_LATENCY_A       0x3C
#define LSM303_REGISTER_ACCEL_TIME_WINDOW_A        0x3D

//Megnetometer_Reg
#define LSM303_REGISTER_MAG_CRA_REG_M              0x00
#define LSM303_REGISTER_MAG_CRB_REG_M              0x01
#define LSM303_REGISTER_MAG_MR_REG_M               0x02
#define LSM303_REGISTER_MAG_OUT_X_H_M              0x03
#define LSM303_REGISTER_MAG_OUT_X_L_M              0x04
#define LSM303_REGISTER_MAG_OUT_Z_H_M              0x05
#define LSM303_REGISTER_MAG_OUT_Z_L_M              0x06
#define LSM303_REGISTER_MAG_OUT_Y_H_M              0x07
#define LSM303_REGISTER_MAG_OUT_Y_L_M              0x08
#define LSM303_REGISTER_MAG_SR_REG_Mg              0x09
#define LSM303_REGISTER_MAG_IRA_REG_M              0x0A
#define LSM303_REGISTER_MAG_IRB_REG_M              0x0B
#define LSM303_REGISTER_MAG_IRC_REG_M              0x0C
#define LSM303_REGISTER_MAG_TEMP_OUT_H_M           0x31
#define LSM303_REGISTER_MAG_TEMP_OUT_L_M           0x32

/* lsm303MagGain Setting*/
#define LSM303_MAGGAIN_1_3                         0x20  // +/- 1.3
#define LSM303_MAGGAIN_1_9                         0x40  // +/- 1.9
#define LSM303_MAGGAIN_2_5                         0x60  // +/- 2.5
#define LSM303_MAGGAIN_4_0                         0x80  // +/- 4.0
#define LSM303_MAGGAIN_4_7                         0xA0  // +/- 4.7
#define LSM303_MAGGAIN_5_6                         0xC0  // +/- 5.6
#define LSM303_MAGGAIN_8_1                         0xE0  // +/- 8.1

/*Constants*/
#define SENSORS_GRAVITY_EARTH                       (9.7996F)              /**< Earth's gravity in m/s^2 */
#define SENSORS_DPS_TO_RADS                         (0.017453293F)          /**< Degrees/s to rad/s multiplier */
#define SENSORS_GAUSS_TO_MICROTESLA                 (100)                   /**< Gauss to micro-Tesla multiplier */
#define SENSORS_LSB_TO_TEMPERATURE                  (0.125F)                /**< LSB to Celcius degree multiplier */

/*Gyro_Setting*/
#define L3GD20_ADDRESS                          (0x6B)        // 1101011
#define L3GD20_POLL_TIMEOUT                     (100)         // Maximum number of read attempts
#define L3GD20_ID                               0xD4
#define L3GD20H_ID                              0xD7
#define GYRO_SENSITIVITY_250DPS                 (0.00875F)    // Roughly 22/256 for fixed point match
#define GYRO_SENSITIVITY_500DPS                 (0.0175F)     // Roughly 45/256
#define GYRO_SENSITIVITY_2000DPS                (0.070F)      // Roughly 18/256
#define TEMP_SENSITIVITY                        (1.0F)      //Temperature data (1LSB/deg - 8-bit resolution)

/*Gyro_Reg*/
#define GYRO_REGISTER_WHO_AM_I             0x0F   // 11010100   r
#define GYRO_REGISTER_CTRL_REG1            0x20   // 00000111   rw
#define GYRO_REGISTER_CTRL_REG2            0x21   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG3            0x22   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG4            0x23   // 00000000   rw
#define GYRO_REGISTER_CTRL_REG5            0x24   // 00000000   rw
#define GYRO_REGISTER_REFERENCE            0x25   // 00000000   rw
#define GYRO_REGISTER_OUT_TEMP             0x26   //            r
#define GYRO_REGISTER_STATUS_REG           0x27   //            r
#define GYRO_REGISTER_OUT_X_L              0x28   //            r
#define GYRO_REGISTER_OUT_X_H              0x29   //            r
#define GYRO_REGISTER_OUT_Y_L              0x2A   //            r
#define GYRO_REGISTER_OUT_Y_H              0x2B   //            r
#define GYRO_REGISTER_OUT_Z_L              0x2C   //            r
#define GYRO_REGISTER_OUT_Z_H              0x2D   //            r
#define GYRO_REGISTER_FIFO_CTRL_REG        0x2E   // 00000000   rw
#define GYRO_REGISTER_FIFO_SRC_REG         0x2F   //            r
#define GYRO_REGISTER_INT1_CFG             0x30   // 00000000   rw
#define GYRO_REGISTER_INT1_SRC             0x31   //            r
#define GYRO_REGISTER_TSH_XH               0x32   // 00000000   rw
#define GYRO_REGISTER_TSH_XL               0x33   // 00000000   rw
#define GYRO_REGISTER_TSH_YH               0x34   // 00000000   rw
#define GYRO_REGISTER_TSH_YL               0x35   // 00000000   rw
#define GYRO_REGISTER_TSH_ZH               0x36   // 00000000   rw
#define  GYRO_REGISTER_TSH_ZL              0x37   // 00000000   rw
#define GYRO_REGISTER_INT1_DURATION        0x38   // 00000000   rw

///////////////////////////////////////////////////////////////////Struct/////////////////////////////////////////////////////////////
typedef struct Vector3{
    int x;
    int y;
    int z;
} Vector3;

typedef struct Vector3f{
    float x;
    float y;
    float z;
} Vector3f;

typedef enum
{
    GYRO_RANGE_250DPS  = 250,
    GYRO_RANGE_500DPS  = 500,
    GYRO_RANGE_2000DPS = 2000
} gyroRange_t;

///////////////////////////////////////////////////////////////////Functions Definitions/////////////////////////////////////////////////////////////
void GlobalInit_IMU(gyroRange_t range);


uint8       readFromSlaveReg(uint8 Slave_Add,uint8 Reg_Add);
void        writeToSlaveReg(uint8 Slave_Add,uint8 Reg_Add, uint8 byte);
void        L3GD20_Init(gyroRange_t range);
void        LSM303_Init(void);
float       LSM303_ReadTeperature(void);
float       L3GD20_ReadTeperature(void);

void        LSM303_ReadMag(Vector3 *magData);
void        LSM303_ReadAccel(Vector3 *accelData);
void        L3GD20_ReadGyro(Vector3 *gyroData);

Vector3f    megRawToMicroTesla(Vector3 raw);
Vector3f    accelRawToMeterPerSecond(Vector3 raw);
Vector3f    gyrRawToRadPerSec(Vector3 raw);
float       megRawToDegree(Vector3 raw); // we don't use that
Vector3f    ComplementaryFilter1(Vector3f accData, Vector3f MagValues, Vector3f gyrData, Vector3f attitudeData, float dt);

void GetDataFromIMU(void);

/* [] END OF FILE */
