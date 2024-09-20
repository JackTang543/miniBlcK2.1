#include "sDRV_HMC5883L.h"


sDRV_HMC5883L_RANGE_t range;

#define HMC5883L_I2C_ADDR       (0x1E << 1)


//*****************************************寄存器地址*****************************************
//todo REG 配置寄存器A
#define ADDR_CONFIG_A           (0x00u)
//平均值设置: 0:1个, 1:2个, 2:4个, 3:8个(默认)
#define MSK_CONFIG_A_MA         (0b01100000u)
//数据输出速率设置: 0:0.75Hz, 1:1.5Hz, 2:3Hz, 3:7.5Hz, 4:15Hz(默认), 5:30Hz, 6:75Hz
#define MSK_CONFIG_A_DO         (0b00011100u)
//测量模式设置: 0:正常模式, 1:正向偏置配置, 2:反向偏置配置 这两个是用来校准的
#define MSK_CONFIG_A_MS         (0b00000011u)

//todo REG 配置寄存器B
#define ADDR_CONFIG_B           (0x01u)
//量程设置: 0:+-0.88Ga, 1:1.3Ga(默认), 2:1.9Ga, 3:2.5Ga, 4:4.0Ga, 5:4.7Ga, 6:5.6Ga, 7:8.1Ga
#define MSK_CONFIG_B_GN         (0b11100000u)

//todo REG 模式寄存器
#define ADDR_MODE_REG           (0x02u)
//测量模式: 0:连续测量, 1:单次测量(默认) 2/3:空闲模式
#define MSK_MODE_REG_MD         (0b00000011u)

//todo REG X轴数据读出寄存器MSB
#define ADDR_X_MSB              (0x03u)

//todo REG X轴数据读出寄存器LSB
#define ADDR_X_LSB              (0x04u)

//todo REG Z轴数据读出寄存器MSB
#define ADDR_Z_MSB              (0x05u)

//todo REG Z轴数据读出寄存器LSB
#define ADDR_Z_LSB              (0x06u)

//todo REG Y轴数据读出寄存器MSB
#define ADDR_Y_MSB              (0x07u)

//todo REG Y轴数据读出寄存器LSB
#define ADDR_Y_LSB              (0x08u)

//todo REG 状态寄存器
#define ADDR_STATUS_REG         (0x09u)
//数据输出锁定
#define MSK_STATUS_REG_LOCK     (0b00000010u)
//已经准备好
#define MSK_STATUS_REG_RDY      (0b00000001u)

//todo REG 身份寄存器A
#define ADDR_IDENT_A_REG        (0x0Au)

//todo REG 身份寄存器B
#define ADDR_IDENT_B_REG        (0x0Bu)

//todo REG 身份寄存器C
#define ADDR_IDENT_C_REG        (0x0Cu)

//****************************************常数****************************************************

#define VAL_IDENT_A             (0b01001000u)
#define VAL_IDENT_B             (0b00110100u)
#define VAL_IDENT_C             (0b00110011u)

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
    sBSP_I2C1M_MemSendByte(HMC5883L_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT,data);
}

//todo 读取的接口
static inline uint8_t read_reg(uint8_t reg_addr){
    return sBSP_I2C1M_MemReadByte(HMC5883L_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT);
}

//todo 连续读取接口
static void read_regs(uint8_t reg_addr,uint8_t* pData,uint8_t len){
	sBSP_I2C1M_MemReadBytes(HMC5883L_I2C_ADDR,reg_addr,I2C_MEMADD_SIZE_8BIT,pData,len);
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



//设置采样的平均值数
static void SetAverageNum(sDRV_HMC5883L_AVER_NUM_t num){
    reg_modify(ADDR_CONFIG_A,MSK_CONFIG_A_MA,num);
}

//数据输出速率
static void SetOutputRate(sDRV_HMC5883L_OUTPUT_RATE_t rate){
    reg_modify(ADDR_CONFIG_A,MSK_CONFIG_A_DO,rate);
}

//测量模式设置
static void SetMeasureMode(sDRV_HMC5883L_MEASURE_MODE_t mode){
    reg_modify(ADDR_MODE_REG,MSK_MODE_REG_MD,mode);
}

//量程设置
void sDRV_HMC5883L_SetRange(sDRV_HMC5883L_RANGE_t _range){
    range = _range;
    reg_modify(ADDR_CONFIG_B,MSK_CONFIG_B_GN,_range);
}

//操作模式设置
static void SetOperationMode(sDRV_HMC5883L_OP_MODE_t mode){
    reg_modify(ADDR_MODE_REG,MSK_MODE_REG_MD,mode);
}

//检查数据是否准备好
static int8_t GetDataIsReady(){
    return !__GET_REG_BIT(read_reg(ADDR_STATUS_REG),MSK_STATUS_REG_RDY);
}

//检查数据是否被锁定
static int8_t GetDataIsLocked(){
    return __GET_REG_BIT(read_reg(ADDR_STATUS_REG),MSK_STATUS_REG_LOCK);
}

int8_t sDRV_HMC5883L_Init(){
    //! 在此之前要初始化I2C

    //检查通信是否正常
    if(read_reg(ADDR_IDENT_A_REG) != VAL_IDENT_A){
        return -1;
    }

    //设置数据4个平均一次
    SetAverageNum(SDRV_HMC5883L_AVER_NUM_4);
    //设置输出速率75Hz
    SetOutputRate(SDRV_HMC5883L_OUTPUT_RATE_75HZ);
    //设置正常模式
    SetMeasureMode(SDRV_HMC5883L_MEASURE_MODE_NORMAL);
    //设置量程
    sDRV_HMC5883L_SetRange(SDRV_HMC5883L_RANGE_0D88GA);
    //设置连续测量模式
    SetOperationMode(SDRV_HMC5883L_OP_MODE_CONT_MEAS);

    return 0;
}


void sDRV_HMC5883L_Read(sDRV_HMC5883L_DATA_t* pData){
    //检查数据是否准备好
    // if(!GetDataIsReady()){
    //     return;
    // }

    uint8_t tmp[6] = {0};
    //连续读出
    read_regs(ADDR_X_MSB,tmp,6);
    int16_t X = (tmp[0] << 8) | tmp[1];
    int16_t Y = (tmp[2] << 8) | tmp[3];
    int16_t Z = (tmp[4] << 8) | tmp[5];

    //根据量程转换原数据
    if(range == SDRV_HMC5883L_RANGE_0D88GA){
        pData->mag_x = X / 1370.0f;
        pData->mag_y = Y / 1370.0f;
        pData->mag_z = Z / 1370.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_1D3GA){
        pData->mag_x = X / 1090.0f;
        pData->mag_y = Y / 1090.0f;
        pData->mag_z = Z / 1090.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_1D9GA){
        pData->mag_x = X / 820.0f;
        pData->mag_y = Y / 820.0f;
        pData->mag_z = Z / 820.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_2D5GA){
        pData->mag_x = X / 660.0f;
        pData->mag_y = Y / 660.0f;
        pData->mag_z = Z / 660.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_4D0GA){
        pData->mag_x = X / 440.0f;
        pData->mag_y = Y / 440.0f;
        pData->mag_z = Z / 440.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_4D7GA){
        pData->mag_x = X / 390.0f;
        pData->mag_y = Y / 390.0f;
        pData->mag_z = Z / 390.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_5D6GA){
        pData->mag_x = X / 330.0f;
        pData->mag_y = Y / 330.0f;
        pData->mag_z = Z / 330.0f;
    }
    else if(range == SDRV_HMC5883L_RANGE_8D1GA){
        pData->mag_x = X / 230.0f;
        pData->mag_y = Y / 230.0f;
        pData->mag_z = Z / 230.0f;
    }

    //转换成毫高斯
    pData->mag_x *= 1000.0f;
    pData->mag_y *= 1000.0f;
    pData->mag_z *= 1000.0f;
}

