/* Code used to define functions useful for 

*/

#include "BMX160.h"

void BMX_calibration(BMX_IMU_typedef *BMX_str){
    
    uint8_t tx_buff[4]; //buffer for transmit bytes
    uint8_t data_buffer[48]; //buffer for received data
    int16_t raw_data[6]; 
    int i_count;
    
    //polling for data giving the first reg and ack for receiving that and the following 11 reg
    tx_buff[0] = GYRO_XLSB_REG;
    
    // calibration is performed with 5 thousand subsequent readings 
    for(int i_gyro_cal=0; i_gyro_cal<2000; i_gyro_cal++){

        //zero te timer 1 counter
        LL_TIM_SetCounter(TIM1,0);

        //polling for data giving the first reg and ack for receiving that and the following 11 reg
        HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, tx_buff, 1, HAL_MAX_DELAY); //transmitting 1 byte with the register of the LSBx
        //wait 5 us
        while(LL_TIM_GetCounter(TIM1)<25){}
        HAL_I2C_Master_Receive(BMX_str->i2c_handler_ptr, BMX160Address, data_buffer, 12, HAL_MAX_DELAY);

        //data unpack
        for(i_count=0; i_count<12; i_count++){
            *((uint8_t*)raw_data+i_count) = *(data_buffer+i_count);
        }

        //gyro data definition
        for(i_count=0; i_count<3; i_count++){
            *(BMX_str->Omega+i_count) = (((float)raw_data[i_count]) * BMX_str->gyro_FS_conv); //- BMX_str->gyro_st_bias[i_count];
        }

        //acc data definition
        for(i_count=0; i_count<3; i_count++){
            *(BMX_str->Acceleration+i_count) = (((float)raw_data[i_count+3]) * BMX_str->acc_FS_conv); // - BMX_str->acc_st_bias[i_count];
        }

        //update of bias via a simple sum with the previous value

        //gyro bias update definition
        for(i_count=0; i_count<3; i_count++){
            BMX_str->gyro_st_bias[i_count] += BMX_str->Omega[i_count];
        }

        //acc bias update definition
        for(i_count=0; i_count<3; i_count++){
            BMX_str->acc_st_bias[i_count] += BMX_str->Acceleration[i_count];
        }

        while(LL_TIM_GetCounter(TIM1)<10000){}
    }

    //average the bias value evaluated
    for(i_count=0; i_count<3; i_count++){
        BMX_str->gyro_st_bias[i_count] = BMX_str->gyro_st_bias[i_count]/2000.0f;
        BMX_str->acc_st_bias[i_count] = BMX_str->acc_st_bias[i_count]/2000.0f;
    }
    /*sprintf((char*)buff, "cal. cplt.\n\r");
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "gyrX= %f\n\r", BMX_str->gyro_st_bias[0]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "gyrY= %f\n\r", BMX_str->gyro_st_bias[1]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "gyrZ= %f\n\r", BMX_str->gyro_st_bias[2]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "accX= %f\n\r", BMX_str->acc_st_bias[0]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "accY= %f\n\r", BMX_str->acc_st_bias[1]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);

    sprintf((char*)buff, "accZ= %f\n\r", BMX_str->acc_st_bias[2]);
    HAL_UART_Transmit(&huart4, buff, strlen((char*)buff), HAL_MAX_DELAY);
    */
     
    BMX_str->acc_st_bias[2] += 1.0f; //subtracting 1 g to the average of Z acc reading

}
 
void BMX_read(BMX_IMU_typedef *BMX_str){

    uint8_t tx_buff[4]; //buffer for transmit bytes
    uint8_t data_buffer[48]; //buffer for received data
    int16_t raw_data[6]; 
    int i_count;
    
    //polling for data giving the first reg and ack for receiving that and the following 11 reg
    tx_buff[0] = GYRO_XLSB_REG;
    HAL_I2C_Master_Transmit(BMX_str->i2c_handler_ptr, BMX160Address, tx_buff, 1, HAL_MAX_DELAY); //transmitting 1 byte with the register of the LSBx
    //wait 5 us
    LL_TIM_SetCounter(TIM1, 0);
    while(LL_TIM_GetCounter(TIM1)<5){}
    HAL_I2C_Master_Receive(BMX_str->i2c_handler_ptr, BMX160Address, data_buffer, 12, HAL_MAX_DELAY);

    //data unpack
    for(i_count=0; i_count<12; i_count++){
        *((uint8_t*)raw_data+i_count) = *(data_buffer+i_count);
    }

    //gyro data definition
    for(i_count=0; i_count<3; i_count++){
        *(BMX_str->Omega+i_count) = (((float)raw_data[i_count]) * BMX_str->gyro_FS_conv); // - BMX_str->gyro_st_bias[i_count];
    }

    //acc data definition
    for(i_count=0; i_count<3; i_count++){
        *(BMX_str->Acceleration+i_count) = (((float)raw_data[i_count+3]) * BMX_str->acc_FS_conv); // - BMX_str->acc_st_bias[i_count];
    }
}