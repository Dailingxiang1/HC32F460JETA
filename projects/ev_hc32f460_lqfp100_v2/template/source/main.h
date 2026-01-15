/**
 *******************************************************************************
 * @file  template/source/main.h
 * @brief This file contains the including files of main routine.
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
#ifndef __MAIN_H__
#define __MAIN_H__

#include "hc32_ll.h"

#define BSP_XTAL_PORT                   (GPIO_PORT_H)
#define BSP_XTAL_PIN                    (GPIO_PIN_00 | GPIO_PIN_01)

#define SPI_UNIT                        (CM_SPI3)
#define SPI_CLK                         (FCG1_PERIPH_SPI3)
#define SPI_TX_EVT_SRC                  (EVT_SRC_SPI3_SPTI)
#define SPI_RX_EVT_SRC                  (EVT_SRC_SPI3_SPRI)

/* DMA definition */
#define DMA_UNIT                        (CM_DMA1)
#define DMA_CLK                         (FCG0_PERIPH_DMA1 | FCG0_PERIPH_AOS)
#define DMA_TX_CH                       (DMA_CH0)
#define DMA_TX_TRIG_CH                  (AOS_DMA1_0)
#define EXAMPLE_SPI_BUF_LEN             (128UL)

#define DMA_TX_INT_CH                   (DMA_INT_TC_CH0)
#define DMA_TX_INT_SRC 									(INT_SRC_DMA1_TC0)
#define DMA_TX_IRQ_NUM 									(INT006_IRQn)

/* SPI definition */

/* SS = PB6 */
#define LCD_SS_PORT                     (GPIO_PORT_B)
#define LCD_SS_PIN                      (GPIO_PIN_06)
#define LCD_SS_FUNC                     (GPIO_FUNC_42)
/* SCK = PB4 */
#define LCD_SCK_PORT                    (GPIO_PORT_B)
#define LCD_SCK_PIN                     (GPIO_PIN_04)
#define LCD_SCK_FUNC                    (GPIO_FUNC_43)
/* MOSI = PB5 */
#define LCD_MOSI_PORT                   (GPIO_PORT_B)
#define LCD_MOSI_PIN                    (GPIO_PIN_05)
#define LCD_MOSI_FUNC                   (GPIO_FUNC_40)
/* RS = PB3 */
#define LCD_RS_PORT 										(GPIO_PORT_B)
#define LCD_RS_PIN                   		(GPIO_PIN_03)
/* RST = PA15 */
#define LCD_RST_PORT 										(GPIO_PORT_A)
#define LCD_RST_PIN                   	(GPIO_PIN_15)
/* BL = PB7 */
#define LCD_BL_PORT 										(GPIO_PORT_B)
#define LCD_BL_PIN                   		(GPIO_PIN_07)

/* I2C definition */
#define I2C_UNIT                        (CM_I2C1)
#define I2C_FCG_USE                     (FCG1_PERIPH_I2C1)

#define I2C_SCL_PORT                    (GPIO_PORT_B)
#define I2C_SCL_PIN                     (GPIO_PIN_00)
#define I2C_SDA_PORT                    (GPIO_PORT_A)
#define I2C_SDA_PIN                     (GPIO_PIN_07)

#define I2C_GPIO_SCL_FUNC               (GPIO_FUNC_49)
#define I2C_GPIO_SDA_FUNC               (GPIO_FUNC_48)

#define I2C_BAUDRATE 										(400000)
#define I2C_ADDR_MD_7BIT                (0U)
#define I2C_ADDR_MD_10BIT               (1U)
#define I2C_ADDR_MD                     (I2C_ADDR_MD_7BIT)

/* ADC definition*/
#define ADC_UNIT                        (CM_ADC1)
#define ADC_PERIPH_CLK                  (FCG3_PERIPH_ADC1)

#define ADC_CH_POTENTIOMETER            (ADC_CH0)
#define ADC_CH                          (ADC_CH_POTENTIOMETER)
#define ADC_CH_PORT                     (GPIO_PORT_A)
#define ADC_CH_PIN                      (GPIO_PIN_00)

#define ADC_SEQ                         (ADC_SEQ_A)
#define ADC_EOC_FLAG                    (ADC_FLAG_EOCA)

#define ADC_VREF                        (3.3F)

/* On Board LED */
#define ON_BOARD_LED_PORT               (GPIO_PORT_B)
#define ON_BOARD_LED_PIN                (GPIO_PIN_15)

/* On Board Motor */
#define ON_BOARD_MOTOR_PORT             (GPIO_PORT_B)
#define ON_BOARD_MOTOR_PIN              (GPIO_PIN_08) 

/* On Board KEYS*/
#define LEFT_KEY1_PORT                  (GPIO_PORT_A)
#define LEFT_KEY1_PIN                   (GPIO_PIN_05)

#define LEFT_KEY2_PORT                  (GPIO_PORT_A)
#define LEFT_KEY2_PIN                   (GPIO_PIN_06)

#define RIGHT_KEY1_PORT                 (GPIO_PORT_A)
#define RIGHT_KEY1_PIN                  (GPIO_PIN_04)

/* Battery Detect PIN */
#define BATTERY_DETECT_PORT             (GPIO_PORT_A)
#define BATTERY_DETECT_PIN              (GPIO_PIN_00)


#endif /* __MAIN_H__ */
/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
