#include "sAPP_AHRS.h"

#define SWF_FILTER_WINDOW_SIZE 1


sDRV_MPU6050_Data_t imu;
sDRV_HMC5883L_DATA_t mag;
sAPP_AHRS_Model_t ahrs;

sAPP_AHRS_IMU_StaticBias_t bias; //IMU静态零偏数据

//数据滑窗滤波初步处理
sAPP_FILTER_SWF_t swf_acc_x, swf_acc_y, swf_acc_z;
sAPP_FILTER_SWF_t swf_gyro_x, swf_gyro_y, swf_gyro_z;
sAPP_FILTER_SWF_t swf_mag_x, swf_mag_y, swf_mag_z;
sAPP_FILTER_SWF_t swf_alt;
sAPP_FILTER_SWF_t swf_pitch;


void sAPP_AHRS_Init(){
	//初始化IMU数据滑窗滤波器
	sAPP_FILTER_SWFInit(&swf_acc_x ,SWF_FILTER_WINDOW_SIZE);
    sAPP_FILTER_SWFInit(&swf_acc_y ,SWF_FILTER_WINDOW_SIZE);
    sAPP_FILTER_SWFInit(&swf_acc_z ,SWF_FILTER_WINDOW_SIZE);
    sAPP_FILTER_SWFInit(&swf_gyro_x,SWF_FILTER_WINDOW_SIZE);
    sAPP_FILTER_SWFInit(&swf_gyro_y,SWF_FILTER_WINDOW_SIZE);
    sAPP_FILTER_SWFInit(&swf_gyro_z,SWF_FILTER_WINDOW_SIZE);
	

	//初始化零偏数据,经过两次校准
	bias.gyro_x = - 4.666923 - 0.032979;
    bias.gyro_y = + 0.928401 + 0.022763;
    bias.gyro_z = + 0.986300 + 0.020918 + 0.081;
    bias.acc_x  = + 0.093782 - 0.001867;
    bias.acc_y  = + 0.328636 - 0.658824;
    bias.acc_z  = - 9.574961 + 9.338580;

	// bias.gyro_x = 0;
    // bias.gyro_y = 0;
    // bias.gyro_z = 0;
    // bias.acc_x  = 0;
    // bias.acc_y  = 0;
    // bias.acc_z  = 0;
}


//IMU矫正静态零偏
void sAPP_AHRS_InitStaticBias(){
	#define POINT_COUNT 1000
	//sLogOK("1s后开始IMU零偏校准,请保持小车静止");

	memset(&ahrs,0,sizeof(sAPP_AHRS_Model_t));
	
	

	//OLED_Printf(10,1,8,"PLS STATIC CAR...");
	//sG2D_UpdateScreen();
	//sG2D_SetAllGRAM(0);
    HAL_Delay(1000);
    

	float acc_x_accu = 0;
	float acc_y_accu = 0;
	float acc_z_accu = 0;
	float gyro_x_accu = 0;
	float gyro_y_accu = 0;
	float gyro_z_accu = 0;
	for(uint16_t i = 0; i < POINT_COUNT; i++){
		sDRV_MPU6050_ReadData();
		sDRV_MPU6050_GetDataBuf(&imu);
		acc_x_accu  += imu.AccX;
		acc_y_accu  += imu.AccY;
		acc_z_accu  += imu.AccZ;
		gyro_x_accu += imu.GyroX;
		gyro_y_accu += imu.GyroY;
		gyro_z_accu += imu.GyroZ;
		HAL_Delay(1);
	}
	bias.acc_x  = acc_x_accu  / POINT_COUNT;
	bias.acc_y  = acc_y_accu  / POINT_COUNT;
	bias.acc_z  = acc_z_accu  / POINT_COUNT - 9.81398f;	//重力加速度
	bias.gyro_x = gyro_x_accu / POINT_COUNT;
	bias.gyro_y = gyro_y_accu / POINT_COUNT;
	bias.gyro_z = gyro_z_accu / POINT_COUNT;
	//sLogOK("校准零偏数据: acc_x:%.4f, acc_y:%.4f, acc_z:%.4f, gyro_x:%.4f, gyro_y:%.4f, gyro_z:%.4f\r\n", \
			bias.acc_x, bias.acc_y, bias.acc_z, bias.gyro_x, bias.gyro_y, bias.gyro_z);

	//OLED_Printf(10,2,8,"STATIC BIAS OK");
    //sG2D_UpdateScreen();
	//sG2D_SetAllGRAM(0);
	//delay_ms(1000);
}




//姿态解算
void sAPP_AHRS_EstiUpdate(){
	#define DT_S 0.005f		//精确时间差
	//互补滤波解算姿态
	sAPP_FILTER_ComplementaryFilter();

	//检查是否处于自由落体,acc_z < 1.0 m/s^2
	if(ahrs.acc_z < 5.0f && ahrs.acc_z > -2.0f){
		ahrs.stat.is_free = true;
	}else{
		ahrs.stat.is_free = false;
	}

	// 计算重力在各轴的分量
    float gx =  M_GRAVITY * sin(ahrs.roll * DEG2RAD);
    float gy = -M_GRAVITY * sin(ahrs.pitch * DEG2RAD) * cos(ahrs.roll * DEG2RAD);
    float gz = -M_GRAVITY * cos(ahrs.pitch * DEG2RAD) * cos(ahrs.roll * DEG2RAD);

	nav.x_acc_pure = ahrs.acc_x + gx;
	nav.y_acc_pure = ahrs.acc_y + gy;
	nav.z_acc_pure = ahrs.acc_z + gz;

	//nav.y_spd += nav.y_acc_pure * DT_S;
	//nav.y_pos += nav.y_spd      * DT_S;
	
	


	//使用互补滤波得到姿态角
	//sAPP_FILTER_ComplementaryFilter();
    
    //sHMI_Debug_Printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%u\n",ahrs.pitch,ahrs.gyro_x,ahrs.gyro_y,ahrs.gyro_z,ahrs.acc_x,ahrs.acc_y,ahrs.acc_z,HAL_GetTick());
    
    //sHMI_Debug_Printf("%.3f,%.3f,%.3f\n",ahrs.pitch,ahrs.roll,ahrs.yaw);

	#undef DT_S
}

//AHRS的IMU数据更新,400k速度下,需要623us的处理时间,500k下500us
void sAPP_AHRS_DataUpdate(){

    //获取气压数据
    //sDRV_BMP280_GetMeasure();
	//获取MPU6050数据
	sDRV_MPU6050_ReadData();
    sDRV_MPU6050_GetDataBuf(&imu);
	//获取HMC5883L数据
	sDRV_HMC5883L_Read(&mag);

	//应用偏置
    imu.GyroX -= bias.gyro_x;
    imu.GyroY -= bias.gyro_y;
    imu.GyroZ -= bias.gyro_z;
    imu.AccX  -= bias.acc_x ;
    imu.AccY  -= bias.acc_y ;
    imu.AccZ  -= bias.acc_z ;

	//应用滤波
    ahrs.acc_x    = sAPP_FILTER_SWFUpdate(&swf_acc_x ,imu.AccX);
    ahrs.acc_y    = sAPP_FILTER_SWFUpdate(&swf_acc_y ,imu.AccY);
    ahrs.acc_z    = sAPP_FILTER_SWFUpdate(&swf_acc_z ,imu.AccZ);
	// ahrs.acc_x    = imu.AccX;
    // ahrs.acc_y    = imu.AccY;
    // ahrs.acc_z    = imu.AccZ;

    ahrs.gyro_x   = sAPP_FILTER_SWFUpdate(&swf_gyro_x,imu.GyroX);
    ahrs.gyro_y   = sAPP_FILTER_SWFUpdate(&swf_gyro_y,imu.GyroY);
    ahrs.gyro_z   = sAPP_FILTER_SWFUpdate(&swf_gyro_z,imu.GyroZ);
	// ahrs.gyro_x   = imu.GyroX;
	// ahrs.gyro_y   = imu.GyroY;
	// ahrs.gyro_z   = imu.GyroZ;

	//高度
	//ahrs.bmp_temp = sDRV_BMP280_GetTemp();
	//ahrs.apress   = sDRV_BMP280_GetPress();
	//ahrs.alt_m    = sAPP_FILTER_SWFUpdate(&swf_alt,press2Alt(ahrs.apress));
	
	// //磁力计
	// ahrs.mag_x    = sAPP_FILTER_SWFUpdate(&swf_mag_x,mag.mag_x);
    // ahrs.mag_y    = sAPP_FILTER_SWFUpdate(&swf_mag_y,mag.mag_y);
    // ahrs.mag_z    = sAPP_FILTER_SWFUpdate(&swf_mag_z,mag.mag_z);

	// //交换mag_x,mag_y
	// float temp = ahrs.mag_x;
    // ahrs.mag_x = ahrs.mag_y;
    // ahrs.mag_y = temp;


	//sHMI_Debug_Printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",imu.AccX,imu.AccY,imu.AccZ,ahrs.acc_x,ahrs.acc_y,ahrs.acc_z);	//m/s²
    
}



void sAPP_AHRS_Print(){
	char buf1[20];
	char buf2[20];
	char buf3[20];
	char buf4[20];
	char buf5[20];

	snprintf(buf1,sizeof(buf1),"Roll:  %4.1f deg",ahrs.roll);
	snprintf(buf2,sizeof(buf3),"Pitch: %4.1f deg",ahrs.pitch);
	snprintf(buf3,sizeof(buf3),"Yaw:   %4.1f deg",ahrs.yaw);
	snprintf(buf4,sizeof(buf4),"Alt:   %4.2f m"   ,ahrs.alt_m / 100.0f);
	snprintf(buf5,sizeof(buf5),"Temp:  %4.1f degC",ahrs.bmp_temp);

	sG2D_WriteString(10,10,buf1);
	sG2D_WriteString(10,20,buf2);
	sG2D_WriteString(10,30,buf3);
	sG2D_WriteString(10,40,buf4);
	sG2D_WriteString(10,50,buf5);

	



	//sHMI_Debug_Printf("S::%10.4f,%10.4f,%10.4f,",ahrs.gyro_x,ahrs.gyro_y,ahrs.gyro_z);	//°/s
    //sHMI_Debug_Printf("%10.4f,%10.4f,%10.4f,",ahrs.acc_x,ahrs.acc_y,ahrs.acc_z);	//m/s²
	//sHMI_Debug_Printf("%10.4f,%10.4f,%10.4f,%10.4f,END \r\n",ahrs.mag_x,ahrs.mag_y,ahrs.mag_z,ahrs.alt_m / 100.0f);	//mGa
    //sHMI_Debug_Printf("roll:%.1f,pitch:%.1f,yaw:%.1f\n",roll,pitch,yaw);
	//sHMI_Debug_Printf("roll:%.1f,pitch:%.1f,yaw:%.1f\n",ahrs.roll,ahrs.pitch,ahrs.yaw);
	//sHMI_Debug_Printf("mag_x:%6.2f mGa,mag_y:%6.2f mGa,mag_z:%6.2f mGa\n",ahrs.mag_x,ahrs.mag_y,ahrs.mag_z);

	// sHMI_Debug_Printf("陀螺仪x:%6.1f °/s,y:%6.1f °/s,z:%6.1f °/s,",ahrs.gyro_x,ahrs.gyro_y,ahrs.gyro_z);
    // sHMI_Debug_Printf("加速度x:%6.2f m/s²,y:%6.2f m/s²,z:%6.2f m/s²,温度:%.2f℃,",ahrs.acc_x,ahrs.acc_y,ahrs.acc_z,imu.Temp);
    //sHMI_Debug_Printf("%6.1f,%6.1f,%6.1f\n",ahrs.roll,ahrs.pitch,ahrs.yaw);
}


