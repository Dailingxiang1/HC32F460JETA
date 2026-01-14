#ifndef __DA213_H
#define __DA213_H

#include "main.h" 

/* ---------------- 硬件接口配置 (根据你的实际引脚修改) ---------------- */
#define DA213_I2C_UNIT          (CM_I2C1)
#define DA213_I2C_FCG           (FCG1_PERIPH_I2C1)

/* SCL = PB00, SDA = PA07 (请确认你的原理图) */
#define DA213_SCL_PORT          (GPIO_PORT_B)
#define DA213_SCL_PIN           (GPIO_PIN_00)
#define DA213_SCL_FUNC          (GPIO_FUNC_49) // Func_I2C1_SCL

#define DA213_SDA_PORT          (GPIO_PORT_A)
#define DA213_SDA_PIN           (GPIO_PIN_07)
#define DA213_SDA_FUNC          (GPIO_FUNC_48) // Func_I2C1_SDA

/* DA213B 地址 (SDO接VCC=0x27, 接GND=0x26) */
#define DA213_ADDR              (0x27) 

/* 寄存器定义 */
#define DA213_REG_CHIP_ID       0x01
#define DA213_REG_X_L           0x02
#define DA213_REG_CONF          0x0F 
#define NSA_REG_POWERMODE_BW    0x11

/* 函数声明 */
void DA213_Init(void);
uint8_t DA213_ReadID(void);
void DA213_Read_XYZ(short *x, short *y, short *z);

#endif

