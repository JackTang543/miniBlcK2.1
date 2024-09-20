#include "sDRV_MPU6050.h"

/**
 * sDRV_MPU6050.c
 * Sightseer's MPU6050 Driver
 * MPU6050驱动
 * 
 * 更新记录:
 * v1.0 TIME 忘记了
 * 初版
 * 
 * v1.1 TIME 2024.06.16
 * 优化了读取速度
 * 
 */



//用于储存一些配置信息
sDRV_MPU6050_t mpu;
//数据缓冲
sDRV_MPU6050_Data_t data;

#define MPU6050_I2C_ADDR      (0x68U << 1U)

//*****************************************寄存器地址*****************************************
//todo REG 辅助I2C供电电压
#define ADDR_AUX_VDDIO                  (0x01U)
//设置为1:辅助I2C总线高逻辑电平是VDD,设置为0:高逻辑电平是VLOGIC
#define MSK_AUX_VDDIO                   (0b10000000U)

//todo REG 采样率分频器
//采样率 = 陀螺仪输出速率 / (1 + SMPLRT_DIV)
#define ADDR_SMPLRT_DIV                 (0x19U)
//采样率分频器
#define MSK_SMPLRT_DIV                  (0b11111111U)

//todo REG 配置寄存器
#define	ADDR_CONFIG			            (0x1AU)
//外部帧同步
#define MSK_CONFIG_EXT_SYNC_SET         (0b00111000U)
//数字低通滤波器
#define MSK_CONFIG_DLPF_CFG             (0b00000111U)

//todo REG 陀螺仪配置寄存器
//用于触发陀螺仪自检和配置陀螺仪满量程范围,自检响应 = 自检使能时传感器输出 - 自检失能时传感器输出
#define	ADDR_GYRO_CONFIG	            (0x1BU)
//写1启动自检
#define MSK_GYRO_CONFIG_XG_ST           (0b10000000U)
#define MSK_GYRO_CONFIG_YG_ST           (0b01000000U)
#define MSK_GYRO_CONFIG_ZG_ST           (0b00100000U)
//0:满量程250deg/s, 1:满量程500deg/s, 2:满量程1000deg/s, 3:满量程2000deg/s
#define MSK_GYRO_CONFIG_FS_SEL          (0b00011000U)

//todo REG 加速度计配置寄存器
//同上
#define	ADDR_ACCEL_CONFIG	            (0x1CU)
//写1启动自检
#define MSK_ACCEL_CONFIG_XA_ST          (0b10000000U)
#define MSK_ACCEL_CONFIG_YA_ST          (0b01000000U)
#define MSK_ACCEL_CONFIG_ZA_ST          (0b00100000U)
//0:满量程2g, 1:满量程4g, 2:满量程8g, 3:满量程16g
#define MSK_ACCEL_CONFIG_AFS_SEL        (0b00011000U)
//0:复位,关闭HPF, 1:截止频率5Hz, 2:截止频率2.5Hz, 3: 1.25Hz 4: 0.63Hz, 7: 保持:输出的值都是在保持时的值的差
#define MSK_ACCEL_CONFIG_ACCEL_HPF      (0b00000111U)

//todo REG 自由落体加速度阈值
#define ADDR_FF_THR			            (0x1DU)
#define MSK_FF_THR                      (0b11111111U)

//todo REG 自由落体持续时间 
//单位ms    
#define ADDR_FF_DUR			            (0x1EU)
#define MSK_FF_DUR                      (0b11111111U)

//todo REG 运动检测阈值 
#define ADDR_MOT_THR                    (0x1FU)
#define MSK_MOT_THR                     (0b11111111U)

//todo REG 运动检测持续时间 
//单位ms    
#define ADDR_MOT_DUR                    (0x20U)
#define MSK_MOT_DUR                     (0b11111111U)

//todo REG 零运动检测阈值   
#define ADDR_ZRMOT_THR                  (0x21U)
#define MSK_ZRMOT_THR                   (0b11111111U)

//todo REG 零运动检测持续时间   
//1单位 = 64ms  
#define ADDR_ZRMOT_DUR                  (0x22U)
#define MSK_ZRMOT_DUR                   (0b11111111U)

//todo REG FIFO使能 
#define ADDR_FIFO_EN                    (0x23U)
//温度传感器FIFO使能    
#define MSK_FIFO_EN_TEMP_FIFO_EN        (0b10000000U)
#define MSK_FIFO_EN_XG_FIFO_EN          (0b01000000U)
#define MSK_FIFO_EN_YG_FIFO_EN          (0b00100000U)
#define MSK_FIFO_EN_ZG_FIFO_EN          (0b00010000U)
#define MSK_FIFO_EN_ACCEL_FIFO_EN       (0b00001000U)
#define MSK_FIFO_EN_SLV2_FIFO_EN        (0b00000100U)
#define MSK_FIFO_EN_SLV1_FIFO_EN        (0b00000010U)
#define MSK_FIFO_EN_SLV0_FIFO_EN        (0b00000001U)

//todo REG I2C主机控制  
#define ADDR_I2C_MST_CTRL               (0x24U)
#define MSK_I2C_MST_CTRL_MULT_MST_EN    (0b10000000U)
#define MSK_I2C_MST_CTRL_WAIT_FOR_ES    (0b01000000U)
#define MSK_I2C_MST_CTRL_SLV_3_FIFO_EN  (0b00100000U)
#define MSK_I2C_MST_CTRL_I2C_MST_P_NSR  (0b00010000U)
#define MSK_I2C_MST_CTRL_I2C_MST_CLK    (0b00001111U)

//todo REG I2C从机0地址
#define ADDR_I2C_SLV0_ADDR	            (0x25U)

//todo REG I2C从机0的寄存器
#define ADDR_I2C_SLV0_REG               (0x26U)

//todo REG I2C从机0控制
#define ADDR_I2C_SLV0_CTRL              (0x27U)

//todo REG I2C从机1地址
#define ADDR_I2C_SLV1_ADDR              (0x28U)

//todo REG I2C从机1的寄存器
#define ADDR_I2C_SLV1_REG               (0x29U)

//todo REG I2C从机1控制
#define ADDR_I2C_SLV1_CTRL              (0x2AU)

//todo REG I2C从机2地址
#define ADDR_I2C_SLV2_ADDR              (0x2BU)

//todo REG I2C从机2的寄存器
#define ADDR_I2C_SLV2_REG               (0x2CU)

//todo REG I2C从机2控制
#define ADDR_I2C_SLV2_CTRL              (0x2DU)

//todo REG I2C从机3地址
#define ADDR_I2C_SLV3_ADDR              (0x2EU)

//todo REG I2C从机3的寄存器
#define ADDR_I2C_SLV3_REG               (0x2FU)

//todo REG I2C从机3控制
#define ADDR_I2C_SLV3_CTRL              (0x30U)

//todo REG I2C从机4地址
#define ADDR_I2C_SLV4_ADDR              (0x31U)

//todo REG I2C从机4的寄存器
#define ADDR_I2C_SLV4_REG               (0x32U)

//todo REG I2C从机4的输出
#define ADDR_I2C_SLV4_DO                (0x33U)

//todo REG I2C从机4控制
#define ADDR_I2C_SLV4_CTRL              (0x34U)

//todo REG I2C从机4的输入
#define ADDR_I2C_SLV4_DI                (0x35U)

//todo REG I2C从机4状态
#define ADDR_I2C_MST_STATUS             (0x36U)

//todo REG 中断引脚配置
#define ADDR_INT_PIN_CFG                (0x37U)

//todo REG 中断使能
#define ADDR_INT_ENABLE                 (0x38U)

//todo REG 中断状态
#define ADDR_INT_STATUS                 (0x3AU)

//todo REG X轴加速度计MSB
#define	ADDR_ACCEL_XOUT_H	            (0x3BU)
#define MSK_ACCEL_XOUT_H                (0b11111111U)

//todo REG X轴加速度计LSB
#define	ADDR_ACCEL_XOUT_L	            (0x3CU)
#define MSK_ACCEL_XOUT_L                (0b11111111U)

//todo REG Y轴加速度计MSB
#define	ADDR_ACCEL_YOUT_H	            (0x3DU)
#define MSK_ACCEL_YOUT_H                (0b11111111U)

//todo REG Y轴加速度计LSB
#define	ADDR_ACCEL_YOUT_L	            (0x3EU)
#define MSK_ACCEL_YOUT_L                (0b11111111U)

//todo REG Z轴加速度计MSB
#define	ADDR_ACCEL_ZOUT_H	            (0x3FU)
#define MSK_ACCEL_ZOUT_H                (0b11111111U)

//todo REG Z轴加速度计LSB
#define	ADDR_ACCEL_ZOUT_L	            (0x40U)
#define MSK_ACCEL_ZOUT_L                (0b11111111U)

//todo REG 温度计MSB
#define	ADDR_TEMP_OUT_H		            (0x41U)
#define MSK_TEMP_OUT_H                  (0b11111111U)

//todo REG 温度计LSB
#define	ADDR_TEMP_OUT_L		            (0x42U)
#define MSK_TEMP_OUT_L                  (0b11111111U)

//todo REG 陀螺仪X轴MSB
#define	ADDR_GYRO_XOUT_H	            (0x43U)
#define MSK_GYRO_XOUT_H                 (0b11111111U)

//todo REG 陀螺仪X轴LSB
#define	ADDR_GYRO_XOUT_L	            (0x44U)
#define MSK_GYRO_XOUT_L                 (0b11111111U)

//todo REG 陀螺仪Y轴MSB
#define	ADDR_GYRO_YOUT_H	            (0x45U)
#define MSK_GYRO_YOUT_H                 (0b11111111U)

//todo REG 陀螺仪Y轴LSB
#define	ADDR_GYRO_YOUT_L	            (0x46U)
#define MSK_GYRO_YOUT_L                 (0b11111111U)

//todo REG 陀螺仪Z轴MSB
#define	ADDR_GYRO_ZOUT_H	            (0x47U)
#define MSK_GYRO_ZOUT_H                 (0b11111111U)

//todo REG 陀螺仪Z轴LSB
#define	ADDR_GYRO_ZOUT_L	            (0x48U)
#define MSK_GYRO_ZOUT_L                 (0b11111111U)

//todo REG 外部传感器数据0~23
#define ADDR_EXT_SENS_DATA_00           (0x49U)  //0~23

//todo REG 运动检测状态
#define ADDR_MOT_DETECT_STATUS          (0x61U)

//todo REG I2C从机0数据输出
#define ADDR_I2C_SLV0_DO                (0x63U)

//todo REG I2C从机1数据输出
#define ADDR_I2C_SLV1_DO                (0x64U)

//todo REG I2C从机2数据输出
#define ADDR_I2C_SLV2_DO                (0x65U)

//todo REG I2C从机3数据输出
#define ADDR_I2C_SLV3_DO                (0x66U)

//todo REG I2C主机延时控制
#define ADDR_I2C_MST_DELAY_CTRL         (0x67U)

//todo REG 信号路径重置,把MEMS,ADC恢复到初始上电的配置
#define ADDR_SIGNAL_PATH_RESET          (0x68U)
#define MSK_SIGNAL_PATH_RESET_GRYO_RST  (0b00000100U)
#define MSK_SIGNAL_PATH_RESET_ACCEL_RST (0b00000010U)
#define MSK_SIGNAL_PATH_RESET_TEMP_RST  (0b00000001U)

//todo REG 运动检测控制
#define ADDR_MOT_DETECT_CTRL            (0x69U)
#define MSK_MOT_DETECT_CTRL             (0b11111111U)

//todo REG 用户控制
#define ADDR_USER_CTRL                  (0x6AU)
//FIFO使能
#define MSK_USER_CTRL_FIFO_EN           (0b01000000U)
//I2C主机使能
#define MSK_USER_CTRL_I2C_MST_EN        (0b00100000U)
//I2C接口失能 MPU6050无效
#define MSK_USER_CTRL_I2C_IF_DIS        (0b00010000U)
//FIFO复位,写1,如果FIFO_EN=0,就会复位FIFO
#define MSK_USER_CTRL_FIFO_RESET        (0b00000100U)
//I2C主机复位
#define MSK_USER_CTRL_I2C_MST_RESET     (0b00000010U)
//复位所有传感器信号路径,这个操作也会清空传感器寄存器
#define MSK_USER_CTRL_SIG_COND_RESET    (0b00000001U)

//todo REG 电源管理1
#define	ADDR_PWR_MGMT_1		            (0x6BU)
//写1设备复位MPU6050
#define MSK_PWR_MGMT_1_DEVICE_RESET     (0b10000000U)
//写1,进入睡眠模式
#define MSK_PWR_MGMT_1_SLEEP            (0b01000000U)
//写1,如果没有进入睡眠模式,就会睡一会再起来采一遍数据然后再睡,循环往复
#define MSK_PWR_MGMT_1_CYCLE            (0b00100000U)
//写1就会禁用温度传感器
#define MSK_PWR_MGMT_1_TEMP_DIS         (0b00001000U)
//设置时钟源 0:内部8MHz振荡器, 1:PLL+X轴陀螺仪, 2:PLL+Y轴陀螺仪, 3:PLL+Z轴陀螺仪, 4:PLL+外部32.768KHz
//5:PLL+外部19.2MHz, 7:停止时钟
#define MSK_PWR_MGMT_1_CLKSEL           (0b00000111U)

//todo REG 电源管理2
#define	ADDR_PWR_MGMT_2		            (0x6CU)
//设置循环唤醒频率: 0:1.25Hz, 1:2.5Hz, 2:5Hz, 3:10Hz
#define MSK_PWR_MGMT_2_LP_WAKE_CTRL     (0b11000000U)
//设置1将会进入Standby模式
#define MSK_PWR_MGMT_2_STBY_XA          (0b00100000U)
#define MSK_PWR_MGMT_2_STBY_YA          (0b00010000U)
#define MSK_PWR_MGMT_2_STBY_ZA          (0b00001000U)
#define MSK_PWR_MGMT_2_STBY_XG          (0b00000100U)
#define MSK_PWR_MGMT_2_STBY_YG          (0b00000010U)
#define MSK_PWR_MGMT_2_STBY_ZG          (0b00000001U)

//todo REG FIFO计数器MSB
#define ADDR_FIFO_COUNT_H               (0x72U)
#define MSK_FIFO_COUNT_H                (0b11111111U)

//todo REG FIFO计数器LSB
#define ADDR_FIFO_COUNT_L               (0x73U)
#define MSK_FIFO_COUNT_L                (0b11111111U)

//todo REG FIFO读写
#define ADDR_FIFO_R_W                   (0x74U)
#define MSK_FIFO_R_W                    (0b11111111U)

//todo REG 我是谁,默认值是0x68
#define	ADDR_WHO_AM_I		            (0x75U)
#define MSK_WHO_AM_I                    (0b11111111U)

//****************************************常数****************************************************

#define VAL_WHO_AM_I                    (0x68U)


//****************************************寄存器操作************************************************

//获取
#define __GET_REG_BIT(__REG__,__REG_MSK__)       ((__REG__) &  (  __REG_MSK__))
//置位
#define __SET_REG_BIT(__REG__,__REG_MSK__)    do{((__REG__) |= (  __REG_MSK__));}while(0)
//清除
#define __CLR_REG_BIT(__REG__,__REG_MSK__)    do{((__REG__) &= (~ __REG_MSK__));}while(0)

//设置寄存器的特定位为给定值,注意这个__DATA__要与掩码对齐
#define __MODIFY_REG_BIT(__REG__, __REG_MSK__, __DATA__) \
    do { \
        (__REG__) = ((__REG__) & ~(__REG_MSK__)) | ((__DATA__) & (__REG_MSK__)); \
    } while(0)


/**
 * @brief 修改寄存器中的特定位段，只在需要时更新。
 * 这个函数首先计算掩码中最低位的位置，然后将输入数据左移对齐到这个位置。
 * 之后，它会计算需要更新的位，并只更新这些位，其他位保持不变。
 *
 * @param reg_addr 寄存器的地址，通常是一个指向uint8_t的指针。
 * @param mask 指定要修改的位，例如0b00111000表示修改第4到第6位。
 * @param data 要写入寄存器位段的数据，数据需要是从最低位开始并只包含目标位段。
 */
static void __MODIFY_REG(uint8_t* reg_addr, uint8_t mask, uint8_t data) {
    uint8_t pos = 0;
    uint8_t mask_original = mask;
    // 计算掩码最低位的位置
    while ((mask & 0x01) == 0) {
        mask >>= 1;
        pos++;
    }
    // 将数据左移，对齐到掩码指定的位
    uint8_t aligned_data = (data << pos) & mask_original;
    // 读取当前寄存器值
    uint8_t current_value = *reg_addr;
    // 计算需要变更的位
    uint8_t changes = (current_value & mask_original) ^ aligned_data;
    // 只更新变化的位，不影响其他位
    uint8_t new_value = (current_value & ~changes) | (aligned_data & changes);
    // 写回修改后的值到寄存器
    *reg_addr = new_value;
}

//****************************************接口******************************************************

//todo 写入的接口
static inline void write_reg(uint8_t reg_addr,uint8_t data){
    uint8_t tmp = 0;
    tmp = sBSP_I2C1M_MemSendByte(MPU6050_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT,data);
}

//todo 读取的接口
static uint8_t read_reg(uint8_t reg_addr){
	uint8_t tmp;
	tmp = sBSP_I2C1M_MemReadByte(MPU6050_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT);
    return tmp;
}

//todo 连续读取接口
static void read_regs(uint8_t reg_addr,uint8_t* pData,uint8_t len){
	sBSP_I2C1M_MemReadBytes(MPU6050_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT,pData,len);
}

// 对寄存器进行修改,形参:寄存器地址,修改的部分,修改的数据
static void reg_modify(uint8_t reg_addr, uint8_t reg_msk, uint8_t data){
    //读改写
    //首先读出寄存器
    uint8_t tmpreg = read_reg(reg_addr);
	//sHMI_Debug_Printf("读出位置0x%X的内容:0x%X\n",reg_addr,tmpreg);
    //进行修改
    __MODIFY_REG(&tmpreg, reg_msk, data);
	//sHMI_Debug_Printf("位掩码:0x%X,数据:0x%X,修改后的内容:0x%X\n\n",reg_msk,data,tmpreg);
    //写回
    write_reg(reg_addr, tmpreg);
}


//****************************************对MPU6050的操作******************************************************

//设置辅助I2C的逻辑电平,是VDD还是VDDIO
void sDRV_MPU6050_SetAuxI2CLv(uint8_t is_vdd){
	reg_modify(ADDR_AUX_VDDIO,MSK_AUX_VDDIO,!!is_vdd);
}

//设置采样速率分频器,这个选项影响MEMS的输出速率
void sDRV_MPU6050_SetSmplrtDiv(uint8_t div){
	reg_modify(ADDR_SMPLRT_DIV,MSK_SMPLRT_DIV,div);
}

//设置外部同步
void sDRV_MPU6050_SetExtSync(sDRV_MPU6050_EXTSYNC_t ext_sync){
    reg_modify(ADDR_CONFIG,MSK_CONFIG_EXT_SYNC_SET,ext_sync);
}

//设置数字低通滤波器
void sDRV_MPU6050_SetDLPF(sDRV_MPU6050_DLPF_t dlpf){
	reg_modify(ADDR_CONFIG,MSK_CONFIG_DLPF_CFG,dlpf);
}

//设置陀螺仪自检,形参不能用|合并
void sDRV_MPU6050_SetGyroSelfTest(sDRV_MPU6050_SELFTEST_t gyro_selftest,uint8_t is_en){
	if(gyro_selftest == SDRV_MPU6050_SELFTEST_ALL){
		reg_modify(ADDR_GYRO_CONFIG,MSK_GYRO_CONFIG_XG_ST | MSK_GYRO_CONFIG_YG_ST | MSK_GYRO_CONFIG_ZG_ST,is_en?0b111:0b000);
	}else if(gyro_selftest == SDRV_MPU6050_SELFTEST_X){
		reg_modify(ADDR_GYRO_CONFIG,MSK_GYRO_CONFIG_XG_ST,!!is_en);
	}else if(gyro_selftest == SDRV_MPU6050_SELFTEST_Y){
		reg_modify(ADDR_GYRO_CONFIG,MSK_GYRO_CONFIG_YG_ST,!!is_en);
	}else if(gyro_selftest == SDRV_MPU6050_SELFTEST_Z){
		reg_modify(ADDR_GYRO_CONFIG,MSK_GYRO_CONFIG_ZG_ST,!!is_en);
	}
}

//设置陀螺仪满量程范围
void sDRV_MPU6050_SetGyroRange(SDRV_MPU6050_GYRO_FS_t fs){
	mpu.gyro_fs = fs;
	reg_modify(ADDR_GYRO_CONFIG,MSK_GYRO_CONFIG_FS_SEL,fs);
}

//设置加速度计自检,形参不能用|合并
void sDRV_MPU6050_SetAccelSelfTest(sDRV_MPU6050_SELFTEST_t accel_selftest,uint8_t is_en){
	if(accel_selftest == SDRV_MPU6050_SELFTEST_ALL){
		reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_XA_ST | MSK_ACCEL_CONFIG_YA_ST | MSK_ACCEL_CONFIG_ZA_ST,is_en?0b111:0b000);
	}else if(accel_selftest == SDRV_MPU6050_SELFTEST_X){
		reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_XA_ST,!!is_en);
	}else if(accel_selftest == SDRV_MPU6050_SELFTEST_Y){
		reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_YA_ST,!!is_en);
	}else if(accel_selftest == SDRV_MPU6050_SELFTEST_Z){
		reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_ZA_ST,!!is_en);
	}
}

//设置加速度计满量程范围
void sDRV_MPU6050_SetAccelRange(SDRV_MPU6050_ACCEL_FS_t fs){
	mpu.accel_fs = fs;
    reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_AFS_SEL,fs);
}

//设置加速度数字低通滤波器
void sDRV_MPU6050_SetAccelDLPF(sDRV_MPU6050_ACCEL_DLPF_t dlpf){
    reg_modify(ADDR_ACCEL_CONFIG,MSK_ACCEL_CONFIG_ACCEL_HPF,dlpf);
}

//! 自由落体,运动检测配置等这里省略

//FIFO使能,形参不能用|合并
void sDRV_MPU6050_SetFIFOEn(sDRV_MPU6050_FIFO_EN_t fifo,uint8_t is_en){
	if(fifo == SDRV_MPU6050_FIFO_EN_TEMP_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_TEMP_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_XG_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_XG_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_YG_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_YG_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_ZG_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_ZG_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_ACCEL_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_ACCEL_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_SLV2_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_SLV2_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_SLV1_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_SLV1_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_SLV0_EN){
		reg_modify(ADDR_FIFO_EN,MSK_FIFO_EN_SLV0_FIFO_EN,!!is_en);
	}else if(fifo == SDRV_MPU6050_FIFO_EN_MEMS_EN){
		reg_modify(ADDR_FIFO_EN,0b11111000u,is_en?0b11111u:0b00000u);
	}
}

// //读出加速度传感器的原始数据
// void sDRV_MPU6050_ReadRawAccel(int16_t* accel_x,int16_t* accel_y,int16_t* accel_z){
// 	uint16_t dataH,dataL;
// 	//AccX
// 	dataH = read_reg(ADDR_ACCEL_XOUT_H);
// 	dataL = read_reg(ADDR_ACCEL_XOUT_L);
// 	*accel_x = (dataH << 8)	| dataL;
// 	//AccY
// 	dataH = read_reg(ADDR_ACCEL_YOUT_H);
// 	dataL = read_reg(ADDR_ACCEL_YOUT_L);
// 	*accel_y = (dataH << 8)	| dataL;
// 	//AccZ
// 	dataH = read_reg(ADDR_ACCEL_ZOUT_H);
// 	dataL = read_reg(ADDR_ACCEL_ZOUT_L);
// 	*accel_z = (dataH << 8)	| dataL;
// }

// //读出温度传感器
// void sDRV_MPU6050_ReadRawTemp(int16_t* temp){
// 	uint16_t dataH,dataL;
//     //Temp
//     dataH = read_reg(ADDR_TEMP_OUT_H);
//     dataL = read_reg(ADDR_TEMP_OUT_L);
//     *temp = (dataH << 8)    | dataL;
// }

// //读出陀螺仪传感器的原始数据
// void sDRV_MPU6050_ReadRawGyro(int16_t* gyro_x,int16_t* gyro_y,int16_t* gyro_z){
// 	uint16_t dataH,dataL;
// 	//GyroX
// 	dataH = read_reg(ADDR_GYRO_XOUT_H);
// 	dataL = read_reg(ADDR_GYRO_XOUT_L);
// 	*gyro_x = (dataH << 8)	| dataL;
// 	//GyroY
// 	dataH = read_reg(ADDR_GYRO_YOUT_H);
// 	dataL = read_reg(ADDR_GYRO_YOUT_L);
// 	*gyro_y = (dataH << 8)	| dataL;
// 	//GyroZ
// 	dataH = read_reg(ADDR_GYRO_ZOUT_H);
// 	dataL = read_reg(ADDR_GYRO_ZOUT_L);
// 	*gyro_z = (dataH << 8)	| dataL;
// }

//信号路径复位,形参不能用|合并
void sDRV_MPU6050_SignalPathReset(sDRV_MPU6050_SPRst_t sprst){
	if(sprst == SDRV_MPU6050_SPRST_GYRO){
		reg_modify(ADDR_SIGNAL_PATH_RESET,MSK_SIGNAL_PATH_RESET_GRYO_RST,0x01);
	}else if(sprst == SDRV_MPU6050_SPRST_ACCEL){
		reg_modify(ADDR_SIGNAL_PATH_RESET,MSK_SIGNAL_PATH_RESET_ACCEL_RST,0x01);
	}else if(sprst == SDRV_MPU6050_SPRST_TEMP){
		reg_modify(ADDR_SIGNAL_PATH_RESET,MSK_SIGNAL_PATH_RESET_TEMP_RST,0x01);
	}
}

//复位
void sDRV_MPU6050_Reset(){
	reg_modify(ADDR_PWR_MGMT_1,MSK_PWR_MGMT_1_DEVICE_RESET,0x01);
}

//设置睡眠模式
void sDRV_MPU6050_SetSleepMode(uint8_t is_sleep){
	reg_modify(ADDR_PWR_MGMT_1,MSK_PWR_MGMT_1_SLEEP,!!is_sleep);
}

//设置循环唤醒模式,进入此模式要确保不在睡眠模式
void sDRV_MPU6050_SetLoopWakeMode(uint8_t is_lpwake){
	reg_modify(ADDR_PWR_MGMT_1,MSK_PWR_MGMT_1_CYCLE,!!is_lpwake);
}

//设置温度传感器失能
void sDRV_MPU6050_SetTempDis(uint8_t is_dis){
	reg_modify(ADDR_PWR_MGMT_1,MSK_PWR_MGMT_1_TEMP_DIS,!!is_dis);
}

//设置时钟选择
void sDRV_MPU6050_SetClkSel(sDRV_MPU6050_CLKSEL_t clk_sel){
	reg_modify(ADDR_PWR_MGMT_1,MSK_PWR_MGMT_1_CLKSEL,clk_sel);
}

//设置循环唤醒模式的频率
void sDRV_MPU6050_SetLoopWakeFreq(sDRV_MPU6050_LP_WAKE_CTRL_t freq){
	reg_modify(ADDR_PWR_MGMT_2,MSK_PWR_MGMT_2_LP_WAKE_CTRL,freq);
}

//设置MEMS是否进入待机状态(停止)
static void sDRV_MPU6050_SetStandby(sDRV_MPU6050_STBY_TYPE_t stby_type,uint8_t is_stby){
	//针对选择所有MEMS进行特殊处理
	if(stby_type == SDRV_MPU6050_STBY_TYPE_ALL){
		reg_modify(ADDR_PWR_MGMT_2,stby_type,0b00111111);
	}
	reg_modify(ADDR_PWR_MGMT_2,stby_type,is_stby);
}

//获取FIFO计数值(里面存了多少数据了)
static uint16_t GetFIFOCount(){
	return read_reg(ADDR_FIFO_COUNT_H) << 8 | read_reg(ADDR_FIFO_COUNT_L);
}

//从FIFO里读一个字节
static uint8_t FIFOReadByte(){
	return read_reg(ADDR_FIFO_R_W);
}

//读取WHO AM I寄存器
static uint8_t GetWhoAmI(){
	return read_reg(ADDR_WHO_AM_I);
}


//****************************************正文******************************************************

int8_t sDRV_MPU6050_Init(){
	//! 注意在之前要初始化I2C

	//检查通信是否正常
	if(GetWhoAmI() != VAL_WHO_AM_I){
        return -1;
    }

	//关闭睡眠模式
	sDRV_MPU6050_SetSleepMode(0);
	//设置时钟为陀螺仪X轴时钟
    sDRV_MPU6050_SetClkSel(SDRV_MPU6050_CLKSEL_PLLGYROX);
	//设置MEMS处于非待机状态
	sDRV_MPU6050_SetStandby(SDRV_MPU6050_STBY_TYPE_ALL,0);
	//设置低通滤波器
	sDRV_MPU6050_SetDLPF(SDRV_MPU6050_DLPF_4);
	//设置采样率分频器4,更新速率200Hz左右
	sDRV_MPU6050_SetSmplrtDiv(0x04);
	//设置陀螺仪满量程:1000°/s
	sDRV_MPU6050_SetGyroRange(SDRV_MPU6050_GYRO_FS_1000);
	//设置加速度计满量程:2G
	sDRV_MPU6050_SetAccelRange(SDRV_MPU6050_ACCEL_FS_2G);

	return 0;
}

int16_t acc_x,acc_y,acc_z;
int16_t gyro_x,gyro_y,gyro_z;
int16_t temp;

//读取数据并处理
//优化之前读取一次数据需要1.8ms,优化之后在400k下只需要393us,500k需要316us
void sDRV_MPU6050_ReadData(){
	//连续读出原始数据
	static uint8_t tmp[14] = {0};
	read_regs(ADDR_ACCEL_XOUT_H,tmp,14);
	acc_x  = (tmp[ 0] << 8)  | tmp[ 1];
	acc_y  = (tmp[ 2] << 8)  | tmp[ 3];
    acc_z  = (tmp[ 4] << 8)  | tmp[ 5];
	temp   = (tmp[ 6] << 8)  | tmp[ 7];
	gyro_x = (tmp[ 8] << 8)  | tmp[ 9];
	gyro_y = (tmp[10] << 8)  | tmp[11];
    gyro_z = (tmp[12] << 8)  | tmp[13];

	//处理数据
	//处理加速度,单位m/s^2
	if(mpu.accel_fs == SDRV_MPU6050_ACCEL_FS_2G){
		data.AccX = (float)acc_x * ( 2.0f / 32768.0f) * 9.8f;
        data.AccY = (float)acc_y * ( 2.0f / 32768.0f) * 9.8f;
        data.AccZ = (float)acc_z * ( 2.0f / 32768.0f) * 9.8f;
	}else if(mpu.accel_fs == SDRV_MPU6050_ACCEL_FS_4G){
		data.AccX = (float)acc_x * ( 4.0f / 32768.0f) * 9.8f;
        data.AccY = (float)acc_y * ( 4.0f / 32768.0f) * 9.8f;
        data.AccZ = (float)acc_z * ( 4.0f / 32768.0f) * 9.8f;
	}else if(mpu.accel_fs == SDRV_MPU6050_ACCEL_FS_8G){
		data.AccX = (float)acc_x * ( 8.0f / 32768.0f) * 9.8f;
        data.AccY = (float)acc_y * ( 8.0f / 32768.0f) * 9.8f;
        data.AccZ = (float)acc_z * ( 8.0f / 32768.0f) * 9.8f;
	}else if(mpu.accel_fs == SDRV_MPU6050_ACCEL_FS_16G){
		data.AccX = (float)acc_x * (16.0f / 32768.0f) * 9.8f;
        data.AccY = (float)acc_y * (16.0f / 32768.0f) * 9.8f;
        data.AccZ = (float)acc_z * (16.0f / 32768.0f) * 9.8f;
	}
	//处理角速度
	if(mpu.gyro_fs == SDRV_MPU6050_GYRO_FS_250){
		data.GyroX = (float)gyro_x *  (250.0f / 32768.0f);
        data.GyroY = (float)gyro_y *  (250.0f / 32768.0f);
        data.GyroZ = (float)gyro_z *  (250.0f / 32768.0f);
	}else if(mpu.gyro_fs == SDRV_MPU6050_GYRO_FS_500){
		data.GyroX = (float)gyro_x *  (500.0f / 32768.0f);
        data.GyroY = (float)gyro_y *  (500.0f / 32768.0f);
        data.GyroZ = (float)gyro_z *  (500.0f / 32768.0f);
	}else if(mpu.gyro_fs == SDRV_MPU6050_GYRO_FS_1000){
		data.GyroX = (float)gyro_x * (1000.0f / 32768.0f);
        data.GyroY = (float)gyro_y * (1000.0f / 32768.0f);
        data.GyroZ = (float)gyro_z * (1000.0f / 32768.0f);
	}else if(mpu.gyro_fs == SDRV_MPU6050_GYRO_FS_2000){
		data.GyroX = (float)gyro_x * (2000.0f / 32768.0f);
        data.GyroY = (float)gyro_y * (2000.0f / 32768.0f);
        data.GyroZ = (float)gyro_z * (2000.0f / 32768.0f);
	}
	//处理温度
	data.Temp = (float)temp / 340.0f + 36.53f;
}

//从数据缓冲区里获取数据
void sDRV_MPU6050_GetDataBuf(sDRV_MPU6050_Data_t* pData){
	pData->AccX  = data.AccX;
	pData->AccY  = data.AccY;
	pData->AccZ  = data.AccZ;
	pData->GyroX = data.GyroX;
	pData->GyroY = data.GyroY;
	pData->GyroZ = data.GyroZ;
	pData->Temp  = data.Temp;
}


