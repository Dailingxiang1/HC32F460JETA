#include "LED.h"

void LED_Init()
{
    stc_tmr6_init_t stcTmr6Init;
    stc_tmr6_pwm_init_t stcPwmInit;
	
    (void)TMR6_StructInit(&stcTmr6Init);
    (void)TMR6_PWM_StructInit(&stcPwmInit);
	
		FCG_Fcg2PeriphClockCmd(FCG2_PERIPH_TMR6_3, ENABLE);
	
		GPIO_SetFunc(ON_BOARD_LED_PORT, ON_BOARD_LED_PIN, GPIO_FUNC_3);
	
    stcTmr6Init.sw_count.u32ClockDiv = TMR6_CLK_DIV2;
    stcTmr6Init.u32PeriodValue = 5000U;  
    (void)TMR6_Init(CM_TMR6_3, &stcTmr6Init);
		
		TMR6_SetGeneralBufNum(CM_TMR6_3, TMR6_CH_B, TMR6_BUF_SINGLE);
		TMR6_GeneralBufCmd(CM_TMR6_3, TMR6_CH_B, ENABLE);
		TMR6_SetCompareValue(CM_TMR6_3, TMR6_CMP_REG_D, 0UL);
	
    //stcPwmInit.u32CompareValue = 1000UL;
    stcPwmInit.u32PeriodMatchPolarity = TMR6_PWM_HIGH;
    stcPwmInit.u32CompareMatchPolarity = TMR6_PWM_INVT;
    stcPwmInit.u32StopPolarity = TMR6_PWM_LOW;
    stcPwmInit.u32StartPolarity = TMR6_PWM_LOW;
    stcPwmInit.u32StartStopHold = TMR6_PWM_START_STOP_HOLD;
		
		(void)TMR6_PWM_Init(CM_TMR6_3, TMR6_CH_B, &stcPwmInit);
		
		TMR6_SetFunc(CM_TMR6_3, TMR6_CH_B, TMR6_PIN_CMP_OUTPUT);
		
		TMR6_PWM_OutputCmd(CM_TMR6_3, TMR6_CH_B, ENABLE);
		
		TMR6_Start(CM_TMR6_3);
}

void LED_Set_Light(uint8_t light_intensity)
{
    uint32_t compare_value;
    
    // 限制亮度值在0-100范围内
    if(light_intensity > 100)
    {
        light_intensity = 100;
    }
    
    // 将0-100的亮度值映射到0-5000的比较值
    // 使用浮点计算：compare_value = (light_intensity / 100.0) * 5000
    // 为了避免浮点运算，使用整数计算：compare_value = light_intensity * 5000 / 100
    compare_value = (uint32_t)light_intensity * 5000UL / 100UL;
    
    // 设置比较值，控制PWM占空比
    TMR6_SetCompareValue(CM_TMR6_3, TMR6_CMP_REG_D, compare_value);
}