#include "lcd_init.h"

#define TMRA_UNIT                       (CM_TMRA_4)
#define TMRA_PERIPH_CLK                 (FCG2_PERIPH_TMRA_4)
#define TMRA_PWM_CH                     (TMRA_CH2)

#define TMRA_PWM_PORT                   (GPIO_PORT_B)
#define TMRA_PWM_PIN                    (GPIO_PIN_07)
#define TMRA_PWM_PIN_FUNC               (GPIO_FUNC_4)

#define TMRA_MD                         (TMRA_MD_SAWTOOTH)
#define TMRA_DIR                        (TMRA_DIR_UP)
#define TMRA_PERIOD_VAL                 (5000U - 1U)
#define TMRA_PWM_CMP_VAL                (2500U - 1U)

#define TMRA_PWM_CMP_MAX   ((uint16_t)(TMRA_PERIOD_VAL - 1U))

static void LCD_BL_Init()
{
    stc_tmra_init_t stcTmraInit;
    stc_tmra_pwm_init_t stcPwmInit;

    /* 1. Enable TimerA peripheral clock. */
    FCG_Fcg2PeriphClockCmd(TMRA_PERIPH_CLK, ENABLE);

    /* 2. Set a default initialization value for stcTmraInit. */
    (void)TMRA_StructInit(&stcTmraInit);

    /* 3. Modifies the initialization values depends on the application. */
    stcTmraInit.sw_count.u8CountMode = TMRA_MD;
    stcTmraInit.sw_count.u8CountDir  = TMRA_DIR;
    stcTmraInit.u32PeriodValue = TMRA_PERIOD_VAL;
    (void)TMRA_Init(TMRA_UNIT, &stcTmraInit);

    /* 4. Set the comparison reference value. */
    (void)TMRA_PWM_StructInit(&stcPwmInit);
//		stcPwmInit.u16StartPolarity = TMRA_PWM_HIGH;
//		stcPwmInit.u16StopPolarity = TMRA_PWM_LOW;
		stcPwmInit.u16CompareMatchPolarity = TMRA_PWM_LOW;
		stcPwmInit.u16PeriodMatchPolarity = TMRA_PWM_HIGH;
		
    stcPwmInit.u32CompareValue = TMRA_PWM_CMP_VAL;
    GPIO_SetFunc(TMRA_PWM_PORT, TMRA_PWM_PIN, TMRA_PWM_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWM_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWM_CH, ENABLE);
		
		//TMRA_CompareBufCmd(TMRA_UNIT, TMRA_PWM_CH, ENABLE);
		//TMRA_SetCompareBufCond(TMRA_UNIT, TMRA_PWM_CH, TMRA_BUF_TRANS_COND_OVF_UDF_CLR);
		
		
		//TMRA_SetCompareValue(TMRA_UNIT, TMRA_PWM_CH, 0);
}
void LCD_Set_BL_Light(int light_intensity)
{
    uint16_t cmp;

    if (light_intensity <= 0)
    {
        cmp = 0;
    }
    else if (light_intensity >= 100)
    {
        cmp = TMRA_PWM_CMP_MAX;
    }
    else
    {
        cmp = (uint16_t)((uint32_t)light_intensity * TMRA_PWM_CMP_MAX / 100U);
    }

    TMRA_SetCompareValue(TMRA_UNIT, TMRA_PWM_CH, cmp);
}

int LCD_Get_BL_Light(void)
{
    uint16_t cmp;
    int light;

    /* CMP 实际是 16 位寄存器 */
    cmp = (uint16_t)TMRA_GetCompareValue(TMRA_UNIT, TMRA_PWM_CH);

    /* 裁剪到硬件合法范围 */
    if (cmp >= (TMRA_PERIOD_VAL - 1U))
    {
        cmp = (uint16_t)(TMRA_PERIOD_VAL - 1U);
    }

    /* 0-(PERIOD-1) 映射回 0-100 */
    light = (int)((uint32_t)cmp * 100U / (TMRA_PERIOD_VAL - 1U));

    return light;
}

/* 如果你后面要做 DMA 加速填充，可以在这里扩展
   目前先给一个空实现，不影响功能 */
void LCD_DMA_Init(void)
{
    /* 可选：把 DMA 配置放这里，或继续沿用你 main.c 的 LCD_SPI_Config() */
}

void LCD_GPIO_Init(void)
{
    stc_gpio_init_t stcGpioInit;
    GPIO_StructInit(&stcGpioInit);

    stcGpioInit.u16PinDir   = PIN_DIR_OUT;
    stcGpioInit.u16PinState = PIN_STAT_SET;
    stcGpioInit.u16PinDrv   = PIN_HIGH_DRV;
		stcGpioInit.u16PullUp = PIN_PU_ON;
    /* CS / DC / RST / BL 做普通输出 */
    (void)GPIO_Init(LCD_CS_PORT,  LCD_CS_PIN,  &stcGpioInit);
    (void)GPIO_Init(LCD_DC_PORT,  LCD_DC_PIN,  &stcGpioInit);
    (void)GPIO_Init(LCD_RST_PORT, LCD_RST_PIN, &stcGpioInit);
    //(void)GPIO_Init(LCD_BL_PORT,  LCD_BL_PIN,  &stcGpioInit);
		LCD_BL_Init();
	
    /* 默认电平 */
    LCD_CS_Set();
    LCD_DC_Set();
    LCD_RES_Set();
		//LCD_Set_BL_Light(2);
	
		TMRA_Start(TMRA_UNIT);
    //LCD_BLK_Set();
}

/* 通过 SPI3 发送 1 字节（阻塞） */
void LCD_Writ_Bus(u8 dat)
{
    /* 你的 DDL 里 main.c 已经在用 SPI_Trans，所以这里直接复用 */
    (void)SPI_Trans(SPI_UNIT, &dat, 1U, 10U);
}

void LCD_WR_DATA8(u8 dat)
{
    LCD_CS_Clr();
    LCD_DC_Set();      /* Data */
    LCD_Writ_Bus(dat);
    LCD_CS_Set();
}

void LCD_WR_DATA(u16 dat)
{
    u8 hi = (u8)(dat >> 8);
    u8 lo = (u8)(dat & 0xFF);

    LCD_CS_Clr();
    LCD_DC_Set();      /* Data */
    LCD_Writ_Bus(hi);
    LCD_Writ_Bus(lo);
    LCD_CS_Set();
}

void LCD_WR_REG(u8 dat)
{
    LCD_CS_Clr();
    LCD_DC_Clr();      /* Command */
    LCD_Writ_Bus(dat);
    LCD_CS_Set();
}

void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
    LCD_WR_REG(0x2A);
    LCD_WR_DATA(x1);
    LCD_WR_DATA(x2);

    LCD_WR_REG(0x2B);
    LCD_WR_DATA(y1);
    LCD_WR_DATA(y2);

    LCD_WR_REG(0x2C);
}

void LCD_Init(void)
{
    LCD_GPIO_Init();

    LCD_RES_Clr();
    delay_ms(30);
    LCD_RES_Set();
    delay_ms(100);

    LCD_BLK_Set();
    delay_ms(100);

    /* ====== 下面保持你的 ST7789V 初始化序列（按你原文件） ====== */
    LCD_WR_REG(0x11);      /* Sleep Out */

    LCD_WR_REG(0x36);
    if (USE_HORIZONTAL == 0)      LCD_WR_DATA8(0x00);
    else if (USE_HORIZONTAL == 1) LCD_WR_DATA8(0xC0);
    else if (USE_HORIZONTAL == 2) LCD_WR_DATA8(0x70);
    else                          LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);    /* 16bit/pixel */

    /* 你原 lcd_init.c 里的其它寄存器配置（E0/E1/...）请继续粘贴在这里
       我这里只展示你截出来那部分风格，避免漏你工程里其它参数 */
		LCD_WR_REG(0xB2);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x0C);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x33); 

		LCD_WR_REG(0xB7); 
		LCD_WR_DATA8(0x35);  

		LCD_WR_REG(0xBB);
		LCD_WR_DATA8(0x35);

		LCD_WR_REG(0xC0);
		LCD_WR_DATA8(0x2C);

		LCD_WR_REG(0xC2);
		LCD_WR_DATA8(0x01);

		LCD_WR_REG(0xC3);
		LCD_WR_DATA8(0x13);   

		LCD_WR_REG(0xC4);
		LCD_WR_DATA8(0x20);  

		LCD_WR_REG(0xC6); 
		LCD_WR_DATA8(0x0F);    

		LCD_WR_REG(0xD0); 
		LCD_WR_DATA8(0xA4);
		LCD_WR_DATA8(0xA1);

		LCD_WR_REG(0xD6); 
		LCD_WR_DATA8(0xA1);

		LCD_WR_REG(0xE0);
		LCD_WR_DATA8(0xF0);
		LCD_WR_DATA8(0x00);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x04);
		LCD_WR_DATA8(0x05);
		LCD_WR_DATA8(0x29);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x3E);
		LCD_WR_DATA8(0x38);
		LCD_WR_DATA8(0x12);
		LCD_WR_DATA8(0x12);
		LCD_WR_DATA8(0x28);
		LCD_WR_DATA8(0x30);

		LCD_WR_REG(0xE1);
		LCD_WR_DATA8(0xF0);
		LCD_WR_DATA8(0x07);
		LCD_WR_DATA8(0x0A);
		LCD_WR_DATA8(0x0D);
		LCD_WR_DATA8(0x0B);
		LCD_WR_DATA8(0x07);
		LCD_WR_DATA8(0x28);
		LCD_WR_DATA8(0x33);
		LCD_WR_DATA8(0x3E);
		LCD_WR_DATA8(0x36);
		LCD_WR_DATA8(0x14);
		LCD_WR_DATA8(0x14);
		LCD_WR_DATA8(0x29);
		LCD_WR_DATA8(0x32);
	
    //LCD_WR_REG(0x21);      /* Display inversion ON（按你原工程） */
		LCD_WR_REG(0x21);
		

    LCD_WR_REG(0x11);
    delay_ms(120);

    LCD_WR_REG(0x29);      /* Display ON */
}
