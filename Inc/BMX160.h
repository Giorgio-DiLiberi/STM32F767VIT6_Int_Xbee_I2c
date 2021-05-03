/* Header file of BMX160 IMU sensor*/
#ifndef BMX160_IMU
#define BMX160_IMU

#include "main.h"
#include <stdio.h>
#include <string.h>

/*Definition of registers*/

#define BMX160Address 0x68<<1 //Device address
#define BMX_PM_REG 0x7E //power manager register
#define GYRO_XLSB_REG 0x0C //gyro x LSB register (the following 11 registers contains all other gyro and acc data)
#define GYRO_CONF_REG 0x42
#define GYRO_RANGE_REG 0x43
#define ACC_CONF_REG 0x40
#define ACC_RANGE_REG 0x41
/*Sequences in data arrays:
array of 12 is :
{xLSB, xMSB, yLSB, yMSB, zLSB, zMSB} same for acelerometer
arrays of 3 components are x, y, z
*/

typedef struct {
    I2C_HandleTypeDef* i2c_handler_ptr; // pointer to handler of communication periferal
    UART_HandleTypeDef* uart_hndler_ptr;
    float Omega[3]; //[deg/s] angular velocity in deg/s
    float Acceleration[3]; //[g=9.815m/s^2] accelerations in g unit
    float Omega_1[3]; //[deg/s] reading at previous time step 
    float Acceleration_1[3]; //[g] reading at previous time step
    float gyro_FS_conv; //value to be multiplied to convert 16 bit to full scale float value
    float acc_FS_conv; //value to be multiplied to convert 16 bit to full scale float value

    float gyro_st_bias[3]; //gyro bias static evaluated simply as average of some readings
    float acc_st_bias[3]; //accelerometer bias static evaluated simply as average of some readings
    float gyro_variance[3]; //variance in gyro readings
    float acc_variance[3]; //variance in gyro readings

    //filtered values
    float Omega_LPFd[3]; //3 components of angular velocity low pass filtered
    float Acc_LPFd[3]; //3 components of acceleration low pass filtered
    float Omega_LPFd_1[3]; //previous step filtered values
    float Acc_LPFd_1[3]; //prev step
    float LPF_const_raw; //constant to multiply raw data
    float LPF_const_filt_1; //constant to multiply LPFd data at prev steps
    //these constants are evaluated based on filter specifications, there is a LPFd.m script to calculate 
    //the constants for discrete filers

    float T_samp; //[s] sample time set in init func 

} BMX_IMU_typedef;

void BMX_calibration(BMX_IMU_typedef *BMX_str); //function to execute at the beginning to evaluate the calibration

void BMX_read(BMX_IMU_typedef *BMX_str); //function to update the data

void BMX_LPF(BMX_IMU_typedef *BMX_str); //function to eliminate static bias and low pass filter data

#endif