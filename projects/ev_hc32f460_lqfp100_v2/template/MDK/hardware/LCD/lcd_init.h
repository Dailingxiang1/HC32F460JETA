#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include <stdint.h>
#include "main.h"

/* 类型别名 */
#ifndef u8
#define u8  uint8_t
#endif
#ifndef u16
#define u16 uint16_t
#endif
#ifndef u32
#define u32 uint32_t
#endif

/* ===== 屏幕方向：保持你原来的定义方式 =====
   0/1/2/3 具体意义按你原工程来（这里不改逻辑）
*/
#ifndef USE_HORIZONTAL
#define USE_HORIZONTAL 0
#endif

#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 2)
#define LCD_W 240
#define LCD_H 320
#else
#define LCD_W 320
#define LCD_H 240
#endif

/* ====== 引脚定义：与你 main.c 保持一致 ====== */
/* CS = PB6 (建议软件控制，不要设置成 SPI NSS 复用功能) */
#define LCD_CS_PORT     (GPIO_PORT_B)
#define LCD_CS_PIN      (GPIO_PIN_06)

/* SCK = PB4, MOSI = PB5 由 SPI3 复用输出（你 main.c 已经做了 GPIO_SetFunc） */
#define LCD_SCK_PORT    (GPIO_PORT_B)
#define LCD_SCK_PIN     (GPIO_PIN_04)
#define LCD_MOSI_PORT   (GPIO_PORT_B)
#define LCD_MOSI_PIN    (GPIO_PIN_05)

/* DC/RS = PB3 */
#define LCD_DC_PORT     (GPIO_PORT_B)
#define LCD_DC_PIN      (GPIO_PIN_03)

/* RST = PA15 */
#define LCD_RST_PORT    (GPIO_PORT_A)
#define LCD_RST_PIN     (GPIO_PIN_15)

/* BL = PB7 */
#define LCD_BL_PORT     (GPIO_PORT_B)
#define LCD_BL_PIN      (GPIO_PIN_07)

/* ====== SPI 外设：与你 main.c 保持一致 ====== */
#ifndef SPI_UNIT
#define SPI_UNIT        (CM_SPI3)
#endif

/* ====== GPIO 操作宏（DDL）====== */
#define LCD_CS_Clr()    GPIO_ResetPins(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_CS_Set()    GPIO_SetPins(LCD_CS_PORT, LCD_CS_PIN)

#define LCD_DC_Clr()    GPIO_ResetPins(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_DC_Set()    GPIO_SetPins(LCD_DC_PORT, LCD_DC_PIN)

#define LCD_RES_Clr()   GPIO_ResetPins(LCD_RST_PORT, LCD_RST_PIN)
#define LCD_RES_Set()   GPIO_SetPins(LCD_RST_PORT, LCD_RST_PIN)

#define LCD_BLK_Clr()   GPIO_ResetPins(LCD_BL_PORT, LCD_BL_PIN)
#define LCD_BLK_Set()   GPIO_SetPins(LCD_BL_PORT, LCD_BL_PIN)

/* 延时：用 DDL */
#define delay_ms(x)     DDL_DelayMS(x)

/* API */
void LCD_DMA_Init(void);
void LCD_GPIO_Init(void);
void LCD_Writ_Bus(u8 dat);
void LCD_WR_DATA8(u8 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u8 dat);
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);
void LCD_Init(void);

void LCD_Set_BL_Light(int light_intensity);
int LCD_Get_BL_Light(void);
void LCD_Writ_Bus(u8 dat);
#endif
