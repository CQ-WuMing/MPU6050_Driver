/*更新时间 2024年9/17
中秋节、重庆理工大学
B站up：今天你也幸福了吗
B站私信我，欢迎讨论问题*/

#include "mpu6050.h"
#include"math.h"
//定义6个int16_t的数据，一会用于对于接收的数据拼接
//int16_t是有负数的，uint16_t是没负数的
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

//这6个是把拼接好的数据经过计算得到结果
float Ax, Ay, Az, Gx, Gy, Gz;

//使用加速度计算《加速度欧拉角》，先定义几个变量。_a的意思就是用的加速度
float roll_a, pitch_a;

//使用角速度计算《角速度欧拉角》，先定义几个变量。_g的意思就是用的角速度
float roll_g, pitch_g, yaw_g;


//欧拉角
float roll, pitch, yaw;


//以下是函数
void MPU6050_Init(void)//初始化
{
	uint8_t check;
	uint8_t Data;

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000);
	//注意。我的板子I2C1给显示屏了，所以给陀螺仪的是I2C2，故用&hi2c2

	if (check == 0x68)//寄存器文档最后一页写了WHO_AM_I_REG就是0x68
	{
		//电源管理1
		Data = 0x01;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

		//电源管理2（这个写不写无所谓）
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, PWR_MGMT_2_REG, 1, &Data, 1, 1000);

		//滤波（这个写不写无所谓）
		Data = 0x06;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, CONFIG, 1, &Data, 1, 1000);


		//采样频率分频器寄存器
		Data = 0x09;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		//加速度计配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		//陀螺仪配置
		Data = 0x18;
		HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel(void)//读取加速度
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	Ax = Accel_X_RAW / 2048.0;
	Ay = Accel_Y_RAW / 2048.0;
	Az = Accel_Z_RAW  / 2048.0 - 0.5;//此处多减0.5是我的硬件问题，如果你硬件是好的，就不用减
}

void MPU6050_Read_Gyro(void)//读取角速度
{
	uint8_t Rec_Data[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	Gx = Gyro_X_RAW / 16.4;
	Gy = Gyro_Y_RAW / 16.4;
	Gz = Gyro_Z_RAW / 16.4;
}

/*对于yaw roll pitch 的计算我们采用互补滤波法，
 * B站上好多卡尔曼滤波，但是我没看懂（太菜了），
 * CSDN里的讲解有挺多，但是要钱，妈的我可没钱充会员
 互补滤波是看B站up主：铁头山羊
 感谢无私奉献的精神，忠！诚！
 因为我很菜，要是下面的代码有错误，那你自己改改
*/

//此函数是读取结果，当然这个结果是互补滤波法计算的
//如果你想读取yaw roll pitch 仅需调用此函数
void MPU6050_Read_Result(void){

	//复制上面的
	uint8_t Rec_Data_A[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data_A, 6, 1000);
	Accel_X_RAW = (int16_t)(Rec_Data_A[0] << 8 | Rec_Data_A[1]);
	Accel_Y_RAW = (int16_t)(Rec_Data_A[2] << 8 | Rec_Data_A[3]);
	Accel_Z_RAW = (int16_t)(Rec_Data_A[4] << 8 | Rec_Data_A[5]);
	Ax = Accel_X_RAW / 2048.0;
	Ay = Accel_Y_RAW / 2048.0;
	Az = Accel_Z_RAW  / 2048.0 -0.5;//此处多减0.5是我的硬件问题，如果你硬件是好的，就不用减

	//复制上面的
	uint8_t Rec_Data_G[6];
	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data_G, 6, 1000);
	Gyro_X_RAW = (int16_t)(Rec_Data_G[0] << 8 | Rec_Data_G[1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data_G[2] << 8 | Rec_Data_G[3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data_G[4] << 8 | Rec_Data_G[5]);
	Gx = Gyro_X_RAW / 16.384;
	Gy = Gyro_Y_RAW / 16.384;
	Gz = Gyro_Z_RAW / 16.384;


	//使用加速度计算欧拉角，如果用3.1415926，计算结果巨慢，所以，写短些
	roll_a = atan2(Ay, Az) /3.14f * 180;
	pitch_a = - atan2(Ax, Az) /3.14f * 180;

	//使用角速度计算欧拉角
	yaw_g = yaw + Gz * 0.005;
	roll_g = roll + Gx * 0.005;
	pitch_g = pitch + Gy * 0.005;

	//进行互补融合,这是铁头山羊的融合方式
	//const float alpha = 0.95238;
	//yaw = yaw_g;
	//roll = alpha *roll_g +(1 - alpha) * roll_a;
	//pitch = alpha *pitch_g +(1 - alpha) *pitch_a;

	//这是CSDN上的融合方式。
/*https://blog.csdn.net/hbsyaaa/article/details108186892?fromshare=blogdetail&sharetype=blogdetail&sharerId=108186892&sharerefer=PC&sharesource=m0_70625833&sharefrom=from_link*/
	const float alpha = 0.9;//这个0.9可以改，0-1之间的数，自己改改玩玩
	roll = roll + (roll_a - roll_g ) *alpha;
	pitch = pitch + (pitch_a - pitch_g) *alpha;
	yaw = yaw_g;
	/*MPU6050的偏航角yaw是不正常的，
	即使静止不动也会跳动。
	这个纯粹是硬件问题，就算算法怎么牛逼也解决不了。
	只能外加磁力计解决，也可以直接使用MPU9250，这个是自带磁力计的。
	使用时注意不要和电机靠太近，磁场很容易受到干扰*/

	//用STM32C8T6少计算太多浮点数，计算太慢了
}







