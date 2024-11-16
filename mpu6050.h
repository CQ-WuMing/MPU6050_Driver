#ifndef __MPU6050_H__
#define __MPU6050_H__

//这是引用的头文件
#include "stm32f1xx_hal.h"
#include "main.h"
#include "i2c.h"
#include"math.h"
//这一堆都是地址，可以看寄存器的文件
#define MPU6050_ADDR 0xD0//从机地址
#define SMPLRT_DIV_REG 0x19//采样率
#define CONFIG 0x1A
#define GYRO_CONFIG_REG 0x1B//陀螺仪
#define ACCEL_CONFIG_REG 0x1C//加速度
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define PWR_MGMT_2_REG 0x6C
#define WHO_AM_I_REG 0x75

//这是咱们的函数，初始化，读A，读G
void MPU6050_Init(void);
void MPU6050_Read_Accel(void);
void MPU6050_Read_Gyro(void);
void MPU6050_Read_Result(void);//读roll, pitch, yaw


//导出，导出后数据才能被其他程序应用
extern float Ax, Ay, Az, Gx, Gy, Gz,roll, pitch, yaw;//A代表加速度，G代表角速度



#endif
