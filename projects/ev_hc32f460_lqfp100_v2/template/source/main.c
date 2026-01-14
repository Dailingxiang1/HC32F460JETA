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
static int32_t I2C_Master_Initialize(void);
static int32_t I2C_Master_Transmit(uint16_t u16DevAddr, const uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout);
static int32_t I2C_Master_Receive(uint16_t u16DevAddr, uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout);

static void DMA_TransCompleteCallback(void)
{
    /* 清除 DMA 通道传输完成标志 */
    DMA_ClearTransCompleteStatus(DMA_UNIT, DMA_TX_INT_CH);
}

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
 uint8_t spi_tx_data = 0x77;
uint8_t chip_id;
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
	
		stc_gpio_init_t stcGpioInit;

    GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinState = PIN_STAT_RST;
    stcGpioInit.u16PinDir = PIN_DIR_OUT;
    //GPIO_Init(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN, &stcGpioInit);
		GPIO_Init(LCD_BL_PORT, LCD_BL_PIN, &stcGpioInit);
	
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
    /* Add your code here */
    for (;;) {
				DA213_Read_XYZ(&acc_x, &acc_y, &acc_z);
        
        
        sprintf(temp_buff, "SensorID: 0x%02X", chip_id);
				LCD_ShowString(0, 0, temp_buff, RED, BLUE, 32, 0) ;
				sprintf(temp_buff, "Acc_x: %d", acc_x);
				LCD_ShowString(0, 32, temp_buff, RED, BLUE, 32, 0) ;
				sprintf(temp_buff, "Acc_y: %d", acc_x);
				LCD_ShowString(0, 64, temp_buff, RED, BLUE, 32, 0) ;
				sprintf(temp_buff, "Acc_z: %d", acc_x);
				LCD_ShowString(0, 96, temp_buff, RED, BLUE, 32, 0) ;
        DDL_DelayMS(100);
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

/**
 * @brief   Initialize the I2C peripheral for master
 * @param   None
 * @retval int32_t:
 *            - LL_OK:              Success
 **           - LL_ERR_INVD_PARAM:  Invalid parameter
 */
static int32_t I2C_Master_Initialize(void)
{
    int32_t i32Ret;
    stc_i2c_init_t stcI2cInit;
    float32_t fErr;

		stc_gpio_init_t stcGpioInit;
		(void)GPIO_StructInit(&stcGpioInit);
		stcGpioInit.u16PinState = PIN_DIR_OUT;
//		stcGpioInit.u16PullUp = PIN_PU_ON;
//		stcGpioInit.u16PinAttr  = PIN_ATTR_DIGITAL;
//		stcGpioInit.u16PinOutputType = PIN_OUT_TYPE_NMOS;
//		(void)GPIO_Init(I2C_SCL_PORT, I2C_SCL_PIN, &stcGpioInit);
//		(void)GPIO_Init(I2C_SDA_PORT, I2C_SDA_PIN, &stcGpioInit);
		
		GPIO_SetFunc(I2C_SCL_PORT, I2C_SCL_PIN, I2C_GPIO_SCL_FUNC);
    GPIO_SetFunc(I2C_SDA_PORT, I2C_SDA_PIN, I2C_GPIO_SDA_FUNC);
		FCG_Fcg1PeriphClockCmd(I2C_FCG_USE, ENABLE);
    (void)I2C_DeInit(I2C_UNIT);

    (void)I2C_StructInit(&stcI2cInit);
    stcI2cInit.u32ClockDiv = I2C_CLK_DIV2;
    stcI2cInit.u32Baudrate = I2C_BAUDRATE;
    stcI2cInit.u32SclTime = 3UL;
    i32Ret = I2C_Init(I2C_UNIT, &stcI2cInit, &fErr);

    I2C_BusWaitCmd(I2C_UNIT, ENABLE);

    return i32Ret;
}

/**
 * @brief  Master transmit data
 *
 * @param  [in] u16DevAddr          The slave address
 * @param  [in] au8Data             The data array
 * @param  [in] u32Size             Data size
 * @param  [in] u32Timeout          Time out count
 * @retval int32_t:
 *            - LL_OK:              Success
 *            - LL_ERR_TIMEOUT:     Time out
 */
static int32_t I2C_Master_Transmit(uint16_t u16DevAddr, const uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;
    I2C_Cmd(I2C_UNIT, ENABLE);

    I2C_SWResetCmd(I2C_UNIT, ENABLE);
    I2C_SWResetCmd(I2C_UNIT, DISABLE);
    i32Ret = I2C_Start(I2C_UNIT, u32Timeout);
    if (LL_OK == i32Ret) {
#if (I2C_ADDR_MD == I2C_ADDR_MD_10BIT)
        i32Ret = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2C_DIR_TX, u32Timeout);
#else
        i32Ret = I2C_TransAddr(I2C_UNIT, u16DevAddr, I2C_DIR_TX, u32Timeout);
#endif

        if (LL_OK == i32Ret) {
            i32Ret = I2C_TransData(I2C_UNIT, au8Data, u32Size, u32Timeout);
        }
    }

    (void)I2C_Stop(I2C_UNIT, u32Timeout);
    I2C_Cmd(I2C_UNIT, DISABLE);

    return i32Ret;
}

static int32_t I2C_Master_Receive(uint16_t u16DevAddr, uint8_t au8Data[], uint32_t u32Size, uint32_t u32Timeout)
{
    int32_t i32Ret;

    I2C_Cmd(I2C_UNIT, ENABLE);
    I2C_SWResetCmd(I2C_UNIT, ENABLE);
    I2C_SWResetCmd(I2C_UNIT, DISABLE);
    i32Ret = I2C_Start(I2C_UNIT, u32Timeout);
    if (LL_OK == i32Ret) {
        if (1UL == u32Size) {
            I2C_AckConfig(I2C_UNIT, I2C_NACK);
        }

#if (I2C_ADDR_MD == I2C_ADDR_MD_10BIT)
        i32Ret = I2C_Trans10BitAddr(I2C_UNIT, u16DevAddr, I2C_DIR_RX, u32Timeout);
#else
        i32Ret = I2C_TransAddr(I2C_UNIT, u16DevAddr, I2C_DIR_RX, u32Timeout);
#endif

        if (LL_OK == i32Ret) {
            i32Ret = I2C_MasterReceiveDataAndStop(I2C_UNIT, au8Data, u32Size, u32Timeout);
        }

        I2C_AckConfig(I2C_UNIT, I2C_ACK);
    }

    if (LL_OK != i32Ret) {
        (void)I2C_Stop(I2C_UNIT, u32Timeout);
    }
    I2C_Cmd(I2C_UNIT, DISABLE);
    return i32Ret;
}

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
