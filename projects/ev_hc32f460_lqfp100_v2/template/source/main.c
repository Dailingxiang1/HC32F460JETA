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
#include "stdio.h"
#include "string.h"
#include "picture_def.h"
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

static void DMA_TransCompleteCallback(void)
{
    /* 清除 DMA 通道传输完成标志 */
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_TX_INT_CH);
}

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
const uint32_t u32TestBuf[] __attribute__((section(".ex_rom"))) = 
{ 
    0x11223344UL, 
    0xA1A2A3A4UL, 
    0x55667788UL, 
    0xE5E69900UL, 
}; 

const uint8_t* picture_ptr[] = {
	test_pic,
	micu_board_map,
	pic_bear,
	pic_cute_girl,
	pic_dog,
	pic_dragon,
	pic_monster,
	pic_mouse,
	pic_tree
};
const uint8_t picture_num = sizeof(picture_ptr) / sizeof(picture_ptr[0]);

void Picture_Loop_Task(void) {
    static uint8_t current_index = 0;

    while (1) {
        // 1. 获取当前图片的指针
        const uint8_t* current_pic_data = picture_ptr[current_index];

        // 2. 刷屏 (240x320)
        // 注意：因为数据在 .ex_rom，确保您的单片机开启了 Memory Mapped Mode (QSPI/SPI 映射模式)
        // 否则直接指针访问会导致 HardFault
        LCD_ShowImage(0, 0, 240, 320, current_pic_data);

        // 3. 索引递增与循环
        current_index++;
        if (current_index >= picture_num) {
            current_index = 0; // 回到第一张
        }
				GPIO_TogglePins(GPIO_PORT_B, GPIO_PIN_08 | GPIO_PIN_15);
        // 4. 延时 2 秒
        SysTick_Delay(2000);
    }
}

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
uint8_t chip_id;

void On_Borad_Peripheral_Init()
{
		stc_gpio_init_t stcGpioInit;
		GPIO_StructInit(&stcGpioInit);
	
		/* LCD背光、板载LED、震动马达初始化*/
		stcGpioInit.u16PinState = PIN_STAT_RST;
		stcGpioInit.u16PinDir = PIN_DIR_OUT;
		
		GPIO_Init(LCD_BL_PORT, LCD_BL_PIN, &stcGpioInit);
		GPIO_Init(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN, &stcGpioInit);
		GPIO_Init(ON_BOARD_MOTOR_PORT, ON_BOARD_MOTOR_PIN, &stcGpioInit);
	
		/* 按键初始化 */
		stcGpioInit.u16PullUp = PIN_PU_ON;
		stcGpioInit.u16PinDir =  PIN_DIR_IN;
	
		GPIO_Init(LEFT_KEY1_PORT, LEFT_KEY1_PIN, &stcGpioInit);
		GPIO_Init(LEFT_KEY2_PORT, LEFT_KEY2_PIN, &stcGpioInit);
		GPIO_Init(RIGHT_KEY1_PORT, RIGHT_KEY1_PIN, &stcGpioInit);
		
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

		BSP_CLK_Init();
	
		SysTick_Init(1000);
	
		On_Borad_Peripheral_Init();
	
		LCD_SPI_Config();
	
		LCD_Init();

		DA213_Init();
		
		LL_PERIPH_WP(LL_PERIPH_ALL);
		LCD_Fill(0, 0, 240, 320, BLUE);
		LCD_ShowString(0, 0, "Hello World", RED, BLUE, 32, 0);
		LCD_ShowString(0, 32, "Demo:", RED, BLUE, 32, 0);
		LCD_ShowString(0, 64, "HC32F460JETA", RED, BLUE, 32, 0);
		
		chip_id = DA213_ReadID();
		
		if(chip_id == 0x13) LCD_Fill(0, 0, 240, 320, BLUE);
		
		short acc_x, acc_y, acc_z;
		char temp_buff[21];
		
		
		//Picture_Loop_Task();

		
    /* Add your code here */
    for (;;) {
			if(GPIO_ReadInputPins(RIGHT_KEY1_PORT, RIGHT_KEY1_PIN) == PIN_RESET )
				GPIO_SetPins(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN);
			else
				GPIO_ResetPins(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN);
			
			if(GPIO_ReadInputPins(LEFT_KEY1_PORT, LEFT_KEY1_PIN) == PIN_RESET)
				GPIO_SetPins(ON_BOARD_MOTOR_PORT, ON_BOARD_MOTOR_PIN);
			else
				GPIO_ResetPins(ON_BOARD_MOTOR_PORT, ON_BOARD_MOTOR_PIN);
//				DA213_Read_XYZ(&acc_x, &acc_y, &acc_z);
//        
//        
//        sprintf(temp_buff, "SensorID: 0x%02X", chip_id);
//				LCD_ShowString(0, 0, temp_buff, RED, BLUE, 32, 0) ;
//				sprintf(temp_buff, "Acc_x: %d", acc_x);
//				LCD_ShowString(0, 32, temp_buff, RED, BLUE, 32, 0) ;
//				sprintf(temp_buff, "Acc_y: %d", acc_x);
//				LCD_ShowString(0, 64, temp_buff, RED, BLUE, 32, 0) ;
//				sprintf(temp_buff, "Acc_z: %d", acc_x);
//				LCD_ShowString(0, 96, temp_buff, RED, BLUE, 32, 0) ;
//        DDL_DelayMS(100);
//				if(u32TestBuf[0] == 0x11223344UL)
//					LCD_ShowString(0, 128, "EXFLASH READY", RED, BLUE, 32, 0) ;
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
    stcDmaInit.u32BlockSize  = 1UL;
    stcDmaInit.u32TransCount = 10;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_8BIT;
		
    /* Configure TX */
    stcDmaInit.u32SrcAddrInc  = DMA_DEST_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    stcDmaInit.u32DestAddr    = (uint32_t)(&SPI_UNIT->DR);
		stcDmaInit.u32IntEn      = DMA_INT_ENABLE;
    if (LL_OK != DMA_Init(DMA_UNIT, DMA_TX_CH, &stcDmaInit)) {
        for (;;) {
        }
    }
    AOS_SetTriggerEventSrc(DMA_TX_TRIG_CH, SPI_TX_EVT_SRC);
		
    /* DMA receive NVIC configure */
    stcIrqSignConfig.enIntSrc    = DMA_TX_INT_SRC;
    stcIrqSignConfig.enIRQn      = DMA_TX_IRQ_NUM;
    stcIrqSignConfig.pfnCallback = &DMA_TransCompleteCallback;
    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    NVIC_ClearPendingIRQ(stcIrqSignConfig.enIRQn);
    NVIC_SetPriority(stcIrqSignConfig.enIRQn, DDL_IRQ_PRIO_DEFAULT);
    NVIC_EnableIRQ(stcIrqSignConfig.enIRQn);
		
//    DMA_Cmd(DMA_UNIT, ENABLE);
//    DMA_ChCmd(DMA_UNIT, DMA_TX_CH, ENABLE);
		
		SPI_Cmd(SPI_UNIT, ENABLE);
}

void SysTick_Handler(void)
{
    SysTick_IncTick();

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
