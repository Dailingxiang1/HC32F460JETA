#include "BATTERY.h"

#define ADC_TIMEOUT_VAL                 (1000U)

/**
 * @brief  Configures ADC clock.
 * @param  None
 * @retval None
 */
static void AdcClockConfig(void)
{
		CLK_SetClockDiv(CLK_BUS_PCLK4, CLK_PCLK4_DIV4);
		CLK_SetPeriClockSrc(ADC_CLK);
}

/**
 * @brief  Set specified ADC pin to analog mode.
 * @param  None
 * @retval None
 */
static void AdcSetPinAnalogMode(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinAttr = PIN_ATTR_ANALOG;
    (void)GPIO_Init(ADC_CH_PORT, ADC_CH_PIN, &stcGpioInit);
}

/**
 * @brief  Initializes ADC.
 * @param  None
 * @retval None
 */
static void AdcInitConfig(void)
{
    stc_adc_init_t stcAdcInit;

    /* 1. Enable ADC peripheral clock. */
    FCG_Fcg3PeriphClockCmd(ADC_PERIPH_CLK, ENABLE);

    /* 2. Modify the default value depends on the application. Not needed here. */
    (void)ADC_StructInit(&stcAdcInit);

    /* 3. Initializes ADC. */
    (void)ADC_Init(ADC_UNIT, &stcAdcInit);

    /* 4. ADC channel configuration. */
    /* 4.1 Set the ADC pin to analog input mode. */
    AdcSetPinAnalogMode();
    /* 4.2 Enable ADC channels. Call ADC_ChCmd() again to enable more channels if needed. */
    ADC_ChCmd(ADC_UNIT, ADC_SEQ, ADC_CH, ENABLE);

    /* 5. Set sampling time if needed. */
    ADC_SetSampleTime(ADC_UNIT, ADC_CH, 0x40U);

    /* 6. Conversion data average calculation function, if needed.
          Call ADC_ConvDataAverageChCmd() again to enable more average channels if needed. */
    ADC_ConvDataAverageConfig(ADC_UNIT, ADC_AVG_CNT8);
    ADC_ConvDataAverageChCmd(ADC_UNIT, ADC_CH, ENABLE);
}

static uint16_t u16AdcValue;
/**
 * @brief  Use ADC in polling mode.
 * @param  None
 * @retval None
 * 需要在主循环中轮询调用
 */
void AdcPolling(void)
{
    
    int32_t iRet = LL_ERR;
    __IO uint32_t u32TimeCount = 0UL;

    /* Can ONLY start sequence A conversion.
       Sequence B needs hardware trigger to start conversion. */
    ADC_Start(ADC_UNIT);
    do {
        if (ADC_GetStatus(ADC_UNIT, ADC_EOC_FLAG) == SET) {
            ADC_ClearStatus(ADC_UNIT, ADC_EOC_FLAG);
            iRet = LL_OK;
            break;
        }
    } while (u32TimeCount++ < ADC_TIMEOUT_VAL);

    if (iRet == LL_OK) {
        /* Get any ADC value of sequence A channel that needed. */
        u16AdcValue = ADC_GetValue(ADC_UNIT, ADC_CH);
    } else {
        ADC_Stop(ADC_UNIT);
    }
}


void Battery_Init()
{
		AdcClockConfig();
		AdcInitConfig();
}