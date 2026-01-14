#include "da213.h"

#define I2C_TIMEOUT             (10000UL) // 超时计数

/* ---------------- 内部：I2C 硬件初始化 ---------------- */
static void I2C_Hardware_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

    /* 1. 开启时钟 */
    FCG_Fcg1PeriphClockCmd(DA213_I2C_FCG, ENABLE);

    /* 2. 配置 GPIO (开漏 + 上拉) */
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv        = PIN_HIGH_DRV;
    stcGpioInit.u16PinDir        = PIN_DIR_OUT;
    stcGpioInit.u16PullUp        = PIN_PU_ON;           // 必须开上拉
    stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_NMOS;   // 必须开漏 (Open-Drain)
    
    GPIO_Init(DA213_SCL_PORT, DA213_SCL_PIN, &stcGpioInit);
    GPIO_Init(DA213_SDA_PORT, DA213_SDA_PIN, &stcGpioInit);

    /* 3. 配置引脚复用 */
    GPIO_SetFunc(DA213_SCL_PORT, DA213_SCL_PIN, DA213_SCL_FUNC);
    GPIO_SetFunc(DA213_SDA_PORT, DA213_SDA_PIN, DA213_SDA_FUNC);

    /* 4. 配置 I2C 模块 */
    I2C_DeInit(DA213_I2C_UNIT);
    I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV1; // 内部时钟分频
    stcI2cInit.u32Baudrate = 100000UL;     // 100kHz 标准速率
    stcI2cInit.u32SclTime  = 0UL;          // 自动计算 SCL 延时
    I2C_Init(DA213_I2C_UNIT, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(DA213_I2C_UNIT, ENABLE); // 开启总线忙等待
    I2C_Cmd(DA213_I2C_UNIT, ENABLE);        // 使能 I2C
}

/* ---------------- 内部：写寄存器 ---------------- */
/* 时序：Start -> DevAddr(W) -> RegAddr -> Data -> Stop */
static int32_t DA213_Write_Reg(uint8_t reg, uint8_t data)
{
    int32_t i32Ret;
    uint8_t u8Buf[2];

    u8Buf[0] = reg;
    u8Buf[1] = data;

    /* 1. 发送 Start */
    I2C_Start(DA213_I2C_UNIT, I2C_TIMEOUT);

    /* 2. 发送设备地址 (写方向) */
    i32Ret = I2C_TransAddr(DA213_I2C_UNIT, DA213_ADDR, I2C_DIR_TX, I2C_TIMEOUT);
    
    if (LL_OK == i32Ret)
    {
        /* 3. 发送 寄存器地址 + 数据 (共2字节) */
        /* 注意：HC32库可以直接发数组，这比分开发送更高效 */
        i32Ret = I2C_TransData(DA213_I2C_UNIT, u8Buf, 2, I2C_TIMEOUT);
    }

    /* 4. 发送 Stop */
    I2C_Stop(DA213_I2C_UNIT, I2C_TIMEOUT);

    return i32Ret;
}

/* ---------------- 内部：读寄存器 (关键修正) ---------------- */
/* 时序：Start -> DevAddr(W) -> RegAddr -> Restart -> DevAddr(R) -> Data -> Stop */
static int32_t DA213_Read_Buffer(uint8_t reg, uint8_t *buffer, uint32_t len)
{
    int32_t i32Ret;

    /* 1. 发送 Start */
    I2C_Start(DA213_I2C_UNIT, I2C_TIMEOUT);

    /* 2. 发送设备地址 (写方向) */
    i32Ret = I2C_TransAddr(DA213_I2C_UNIT, DA213_ADDR, I2C_DIR_TX, I2C_TIMEOUT);

    if (LL_OK == i32Ret)
    {
        /* 3. 发送寄存器地址 (告诉它我们要读哪) */
        i32Ret = I2C_TransData(DA213_I2C_UNIT, &reg, 1, I2C_TIMEOUT);
    }

    if (LL_OK == i32Ret)
    {
        /* 4. 发送 Restart (重复起始条件) */
        i32Ret = I2C_Restart(DA213_I2C_UNIT, I2C_TIMEOUT);
    }

    if (LL_OK == i32Ret)
    {
        /* 5. 发送设备地址 (读方向) */
        i32Ret = I2C_TransAddr(DA213_I2C_UNIT, DA213_ADDR, I2C_DIR_RX, I2C_TIMEOUT);
    }

    if (LL_OK == i32Ret)
    {
        /* 6. 读取数据并自动发送 Stop */
        /* HC32 库函数 MasterReceiveDataAndStop 会自动处理 ACK/NACK 和 STOP */
        i32Ret = I2C_MasterReceiveDataAndStop(DA213_I2C_UNIT, buffer, len, I2C_TIMEOUT);
    }
    else
    {
        /* 出错时手动停止 */
        I2C_Stop(DA213_I2C_UNIT, I2C_TIMEOUT);
    }

    return i32Ret;
}

/* ---------------- 用户层函数 ---------------- */

void DA213_Init(void)
{
    I2C_Hardware_Init();
    DDL_DelayMS(10); // 等待芯片上电稳定

    /* 开启 DA213 */
    DA213_Write_Reg(NSA_REG_POWERMODE_BW, 0x30); // Enable chip
    DDL_DelayMS(5);

    /* 配置 ODR 和 Enable 位 */
    DA213_Write_Reg(DA213_REG_CONF, 0xC0); // 0x80 | 0x40
}

uint8_t DA213_ReadID(void)
{
    uint8_t id = 0;
    DA213_Read_Buffer(DA213_REG_CHIP_ID, &id, 1);
    return id;
}

void DA213_Read_XYZ(short *x, short *y, short *z)
{
    uint8_t buffer[6];
    
    /* 连续读取 6 个字节 */
    DA213_Read_Buffer(DA213_REG_X_L, buffer, 6);
    
    /* 数据拼接 */
    *x = (short)((buffer[1] << 8) | buffer[0]);
    *y = (short)((buffer[3] << 8) | buffer[2]);
    *z = (short)((buffer[5] << 8) | buffer[4]);
    
    /* 处理数据对齐 (假设是14位数据，左对齐，通常需右移) */
    /* 具体取决于 DA213 的寄存器配置，通常建议右移 2 位 */
    *x >>= 2;
    *y >>= 2;
    *z >>= 2;
}
