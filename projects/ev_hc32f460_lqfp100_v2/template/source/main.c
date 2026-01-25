/**
 *******************************************************************************
 * @file  template/source/main.c
 * @brief Main program template for the Device Driver Library.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "main.h"
#include "lcd_init.h"
#include "lcd.h"
#include "DA213.h"
#include "LED.h"
#include "Motor.h"

#include "stdio.h"
#include "string.h"

#include "MultiTimer.h"
#include "multi_button.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_demo_ball.h"
#include "lv_demo_music.h"
/**
 * @addtogroup HC32F460_DDL_Examples
 * @{
 */

/**
 * @addtogroup LL_Templates
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static void BSP_CLK_Init(void);
static void LCD_SPI_Config(void);

volatile uint8_t g_dma_transfer_complete = 0;

static void DMA_TransCompleteCallback(void)
{
    /* 检查是否是正确的通道触发 (防止误触发) */
    if(DMA_GetTransCompleteStatus(DMA_UNIT, DMA_TX_INT_CH))
    {
        DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_TX_INT_CH);
        g_dma_transfer_complete = 1; /* 告知主循环：这一块发完了 */
    }
}

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


static Button left_bnt1, left_btn2, right_btn1;

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
uint8_t chip_id;
lv_obj_t * label_chip;
lv_obj_t * label_accx;
lv_obj_t * label_accy;
lv_obj_t * label_accz;

void On_Borad_Peripheral_Init()
{
		stc_gpio_init_t stcGpioInit;
		GPIO_StructInit(&stcGpioInit);
	
		/* LCD背光、板载LED、震动马达初始化*/
		stcGpioInit.u16PinState = PIN_STAT_RST;
		stcGpioInit.u16PinDir = PIN_DIR_OUT;
		
		LED_Init();
		MOTOR_Init();
		GPIO_Init(LCD_BL_PORT, LCD_BL_PIN, &stcGpioInit);
		GPIO_Init(ON_BOARD_MOTOR_PORT, ON_BOARD_MOTOR_PIN, &stcGpioInit);
	
		/* 按键初始化 */
		stcGpioInit.u16PullUp = PIN_PU_ON;
		stcGpioInit.u16PinDir =  PIN_DIR_IN;
	
		GPIO_Init(LEFT_KEY1_PORT, LEFT_KEY1_PIN, &stcGpioInit);
		GPIO_Init(LEFT_KEY2_PORT, LEFT_KEY2_PIN, &stcGpioInit);
		GPIO_Init(RIGHT_KEY1_PORT, RIGHT_KEY1_PIN, &stcGpioInit);
	
		/* ADC电量检测初始化 */
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    GPIO_Init(ADC_CH_PORT, ADC_CH_PIN, &stcGpioInit);
		
}

uint8_t read_button_gpio(uint8_t button_id)
{
    switch (button_id) {
        case 1:
            return !GPIO_ReadInputPins(LEFT_KEY1_PORT, LEFT_KEY1_PIN);
        case 2:
            return !GPIO_ReadInputPins(LEFT_KEY2_PORT, LEFT_KEY2_PIN);
				case 3:
						return !GPIO_ReadInputPins(RIGHT_KEY1_PORT, RIGHT_KEY1_PIN);
        default:
            return 0;
    }
}

void left1_btn_single_click_callback(Button* btn_handle)
{
		GPIO_TogglePins(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN);
}

void left1_btn_double_click_callback(Button* btn_handle)
{
		GPIO_TogglePins(ON_BOARD_MOTOR_PORT, ON_BOARD_MOTOR_PIN);
}

/**
 * @brief  简单的 UI 测试函数：在屏幕中间显示一行字
 */
void lv_example_hello_world(void)
{
    /* 获取当前活动屏幕 */
    lv_obj_t * scrub = lv_scr_act();

    /* 1. 创建一个按钮对象 */
    lv_obj_t * btn = lv_btn_create(scrub);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0); // 居中显示
    lv_obj_set_size(btn, 120, 50);            // 设置大小

    /* 2. 在按钮上创建一个标签 */
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Hello HC32!");  // 设置文字
    lv_obj_center(label);                     // 文字居中
}
/**
 * @brief  Main function of template project
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    /* Register write enable for some required peripherals. */
    LL_PERIPH_WE(LL_PERIPH_ALL);
		//只要SWD,不要JTAG
		GPIO_SetDebugPort(GPIO_PIN_DEBUG, DISABLE);
	
		GPIO_SetDebugPort(GPIO_PIN_SWDIO | GPIO_PIN_SWCLK, ENABLE);
		//GPIO_SetDebugPort(GPIO_PIN_SWO | GPIO_PIN_TRST, DISABLE) ;

		BSP_CLK_Init();
	
		SysTick_Init(1000);
	
		On_Borad_Peripheral_Init();
	
		LCD_SPI_Config();
	
		//LCD_Init();
		lv_init();                                 
		lv_port_disp_init();
		//lv_example_hello_world();
		DA213_Init();

		//lv_demo_music();
		lv_example_bouncing_ball();
		LL_PERIPH_WP(LL_PERIPH_ALL);

		//multiTimerInstall();
		button_init(&left_bnt1, read_button_gpio, 1, 1);
		button_init(&left_btn2, read_button_gpio, 1, 2);
		button_init(&right_btn1, read_button_gpio, 1, 3);
		
		button_attach(&left_bnt1, BTN_SINGLE_CLICK , left1_btn_single_click_callback);
		button_attach(&left_bnt1, BTN_DOUBLE_CLICK , left1_btn_double_click_callback);
		
		button_start(&left_bnt1);
		button_start(&left_btn2);
		button_start(&right_btn1);
		chip_id = DA213_ReadID();
		
		//if(chip_id == 0x13) LCD_Fill(0, 0, 240, 320, BLUE);
		lv_obj_t * scr = lv_scr_act();

		// Chip ID
		label_chip = lv_label_create(scr);
		lv_obj_align(label_chip, LV_ALIGN_TOP_MID, 0, 10);

		// acc X
		label_accx = lv_label_create(scr);
		lv_obj_align(label_accx, LV_ALIGN_TOP_LEFT, 10, 40);

		// acc Y
		label_accy = lv_label_create(scr);
		lv_obj_align(label_accy, LV_ALIGN_TOP_LEFT, 10, 70);

		// acc Z
		label_accz = lv_label_create(scr);
		lv_obj_align(label_accz, LV_ALIGN_TOP_LEFT, 10, 100);

		// Set default texts
		lv_label_set_text(label_chip, "chip_id: --");
		lv_label_set_text(label_accx, "X: --");
		lv_label_set_text(label_accy, "Y: --");
		lv_label_set_text(label_accz, "Z: --");
		short acc_x, acc_y, acc_z;
		char temp_buff[21];
		uint32_t last_update_ms = 0;
    /* Add your code here */
    for (;;) {
    uint32_t now = SysTick_GetTick();  // 获取当前系统毫秒

    // 每 1000ms 更新一次显示
    if (now - last_update_ms >= 1000) {
        last_update_ms = now;

        // 读取传感器
        DA213_Read_XYZ(&acc_x, &acc_y, &acc_z);

        // 更新 LVGL 的标签文本
        lv_label_set_text_fmt(label_chip, "chip: 0x%02X", chip_id);
        lv_label_set_text_fmt(label_accx, "acc X: %d", acc_x);
        lv_label_set_text_fmt(label_accy, "acc Y: %d", acc_y);
        lv_label_set_text_fmt(label_accz, "acc Z: %d", acc_z);
    }
				lv_task_handler();
    }
}

/**
 * @}
 */

/**
 * @}
 */
/**
 * @brief  BSP clock initialize.
 *         Set board system clock to MPLL@200MHz
 * @param  None
 * @retval None
 */
static void BSP_CLK_Init(void)
{
    stc_clock_xtal_init_t     stcXtalInit;
    stc_clock_pll_init_t      stcMpllInit;

    GPIO_AnalogCmd(BSP_XTAL_PORT, BSP_XTAL_PIN, ENABLE);
    (void)CLK_XtalStructInit(&stcXtalInit);
    (void)CLK_PLLStructInit(&stcMpllInit);

    /* Set bus clk div. */
    CLK_SetClockDiv(CLK_BUS_CLK_ALL, (CLK_HCLK_DIV1 | CLK_EXCLK_DIV2 | CLK_PCLK0_DIV1 | CLK_PCLK1_DIV2 | \
                                      CLK_PCLK2_DIV4 | CLK_PCLK3_DIV4 | CLK_PCLK4_DIV2));

    /* Config Xtal and enable Xtal */
    stcXtalInit.u8Mode = CLK_XTAL_MD_OSC;
    stcXtalInit.u8Drv = CLK_XTAL_DRV_ULOW;
    stcXtalInit.u8State = CLK_XTAL_ON;
    stcXtalInit.u8StableTime = CLK_XTAL_STB_2MS;
    (void)CLK_XtalInit(&stcXtalInit);

		/* MPLL config for 24MHz crystal to get 200MHz */
		stcMpllInit.PLLCFGR = 0UL;
		stcMpllInit.PLLCFGR_f.PLLM = 5UL;    // M = 6 (24MHz/6=4MHz)
		stcMpllInit.PLLCFGR_f.PLLN = 99UL;   // N = 100 (4MHz×100=400MHz)
		stcMpllInit.PLLCFGR_f.PLLP = 1UL;    // P = 2 (400MHz/2=200MHz)
		stcMpllInit.PLLCFGR_f.PLLQ = 1UL;    // Q = 2
		stcMpllInit.PLLCFGR_f.PLLR = 1UL;    // R = 2
		stcMpllInit.u8PLLState = CLK_PLL_ON;
		stcMpllInit.PLLCFGR_f.PLLSRC = CLK_PLL_SRC_XTAL;
    (void)CLK_PLLInit(&stcMpllInit);
    /* Wait MPLL ready. */
    while (SET != CLK_GetStableStatus(CLK_STB_FLAG_PLL)) {
        ;
    }

    /* sram init include read/write wait cycle setting */
    SRAM_SetWaitCycle(SRAM_SRAMH, SRAM_WAIT_CYCLE0, SRAM_WAIT_CYCLE0);
    SRAM_SetWaitCycle((SRAM_SRAM12 | SRAM_SRAM3 | SRAM_SRAMR), SRAM_WAIT_CYCLE1, SRAM_WAIT_CYCLE1);

    /* flash read wait cycle setting */
    (void)EFM_SetWaitCycle(EFM_WAIT_CYCLE5);
    /* 3 cycles for 126MHz ~ 200MHz */
    GPIO_SetReadWaitCycle(GPIO_RD_WAIT3);
    /* Switch driver ability */
    (void)PWC_HighSpeedToHighPerformance();
    /* Switch system clock source to MPLL. */
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_PLL);
    /* Reset cache ram */
    EFM_CacheRamReset(ENABLE);
    EFM_CacheRamReset(DISABLE);
    /* Enable cache */
    EFM_CacheCmd(ENABLE);
}

static void LCD_SPI_Config(void)
{
		stc_spi_init_t stcSpiInit;
		stc_spi_delay_t  stcSpiDelayConfig;
		stc_dma_init_t stcDmaInit;
    stc_gpio_init_t stcGpioInit;
		stc_irq_signin_config_t stcIrqSignConfig;
		
	
    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv       = PIN_HIGH_DRV;
		stcGpioInit.u16PullUp       = PIN_PU_ON;
    //(void)GPIO_Init(LCD_SS_PORT,   LCD_SS_PIN,   &stcGpioInit);
    (void)GPIO_Init(LCD_SCK_PORT,  LCD_SCK_PIN,  &stcGpioInit);
    (void)GPIO_Init(LCD_MOSI_PORT, LCD_MOSI_PIN, &stcGpioInit);
	
		/* Configure Port */
		//GPIO_SetFunc(LCD_SS_PORT,   LCD_SS_PIN,   LCD_SS_FUNC);
    GPIO_SetFunc(LCD_SCK_PORT,  LCD_SCK_PIN,  LCD_SCK_FUNC);
    GPIO_SetFunc(LCD_MOSI_PORT, LCD_MOSI_PIN, LCD_MOSI_FUNC);

    /* Configuration SPI */
    FCG_Fcg1PeriphClockCmd(SPI_CLK, ENABLE);
    SPI_StructInit(&stcSpiInit);
    stcSpiInit.u32WireMode          = SPI_4_WIRE;
    stcSpiInit.u32TransMode         = SPI_SEND_ONLY;  //只发不收,驱动LCD
    stcSpiInit.u32MasterSlave       = SPI_MASTER;
    stcSpiInit.u32Parity            = SPI_PARITY_INVD;
    stcSpiInit.u32SpiMode           = SPI_MD_3;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_CLK_DIV2;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    stcSpiInit.u32FrameLevel        = SPI_1_FRAME;
    (void)SPI_Init(SPI_UNIT, &stcSpiInit);
		
		 stcSpiDelayConfig.u32IntervalDelay = SPI_INTERVAL_TIME_1SCK;
		 stcSpiDelayConfig.u32ReleaseDelay  = SPI_RELEASE_TIME_1SCK;
		 stcSpiDelayConfig.u32SetupDelay = SPI_SETUP_TIME_1SCK;
		(void)SPI_DelayStructInit(&stcSpiDelayConfig);
    /* DMA configuration */
    /* DMA configuration */
    FCG_Fcg0PeriphClockCmd(DMA_CLK, ENABLE);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32BlockSize  = 1;
    stcDmaInit.u32TransCount = 1;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
		
    /* Configure TX */
    stcDmaInit.u32SrcAddrInc  = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    stcDmaInit.u32DestAddr    = (uint32_t)(&SPI_UNIT->DR);
		stcDmaInit.u32SrcAddr     = 0;
		stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    if (LL_OK != DMA_Init(DMA_UNIT, DMA_TX_CH, &stcDmaInit)) {
        for (;;) {
        }
    }
    AOS_SetTriggerEventSrc(DMA_TX_TRIG_CH,  SPI_TX_EVT_SRC);
		
		AOS_CommonTriggerCmd(DMA_TX_TRIG_CH, AOS_COMM_TRIG1, ENABLE);   //允许公共事件1触发 DMA1通道0的触发
		AOS_SetTriggerEventSrc(AOS_COMM_1, EVT_SRC_AOS_STRG) ;          //把公共事件1
		
    /* DMA receive NVIC configure */
    stcIrqSignConfig.enIntSrc    = DMA_TX_INT_SRC;
    stcIrqSignConfig.enIRQn      = DMA_TX_IRQ_NUM;
    stcIrqSignConfig.pfnCallback = &DMA_TransCompleteCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
		
		DMA_Cmd(DMA_UNIT, ENABLE);
    
		
		
		//DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
		
		SPI_Cmd(SPI_UNIT, ENABLE);
		//AOS_SW_Trigger();
}

static uint8_t button_ticks_5cnt;
void SysTick_Handler(void)
{
    SysTick_IncTick();
		lv_tick_inc(1);
		if(++button_ticks_5cnt > 3 ){
			button_ticks();
			button_ticks_5cnt = 0;
		}
    __DSB();  /* Arm Errata 838869 */
		
}

void SystemInit_QspiMem(void)
{
    /* QSPI configure */
    CM_GPIO->PWPR = 0xA501U;
    /* High driver */
    CM_GPIO->PCRB1  = 0x0020U;
    CM_GPIO->PCRB14  = 0x0020U;
    CM_GPIO->PCRB13  = 0x0020U;
    CM_GPIO->PCRB12  = 0x0020U;
    CM_GPIO->PCRB10 = 0x0020U;
    CM_GPIO->PCRB2 = 0x0020U;
    /* Set function */
    CM_GPIO->PFSRB1  = 0x07U;
    CM_GPIO->PFSRB14  = 0x07U;
    CM_GPIO->PFSRB13  = 0x07U;
    CM_GPIO->PFSRB12  = 0x07U;
    CM_GPIO->PFSRB10 = 0x07U;
    CM_GPIO->PFSRB2 = 0x07U;
    /* qspi configure */
    CM_PWC->FCG1 &= ~0x00000008UL;
    //CM_QSPI->CR   = 0x0002000D;
		CM_QSPI->CR   = 0x0001000D;
    CM_QSPI->CSCR = 0x00000011;
    CM_QSPI->FCR  = 0x00008332;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
