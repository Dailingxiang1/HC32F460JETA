#include "MOTOR.h"

#define TMRA_UNIT                       (CM_TMRA_4)
#define TMRA_PERIPH_CLK                 (FCG2_PERIPH_TMRA_4)
#define TMRA_PWM_CH                     (TMRA_CH3)

#define TMRA_PWM_PORT                   (GPIO_PORT_B)
#define TMRA_PWM_PIN                    (GPIO_PIN_08)
#define TMRA_PWM_PIN_FUNC               (GPIO_FUNC_4)

#define TMRA_MD                         (TMRA_MD_SAWTOOTH)
#define TMRA_DIR                        (TMRA_DIR_UP)
#define TMRA_PERIOD_VAL                 (5000U - 1U)
#define TMRA_PWM_CMP_VAL                (2500U - 1U)

/**
 * @brief  TimerA configuration.
 * @param  None
 * @retval None
 */
static void TmrAConfig(void)
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

    (void)TMRA_PWM_StructInit(&stcPwmInit);
    stcPwmInit.u32CompareValue = TMRA_PWM_CMP_VAL;
    GPIO_SetFunc(TMRA_PWM_PORT, TMRA_PWM_PIN, TMRA_PWM_PIN_FUNC);
    (void)TMRA_PWM_Init(TMRA_UNIT, TMRA_PWM_CH, &stcPwmInit);
    TMRA_PWM_OutputCmd(TMRA_UNIT, TMRA_PWM_CH, ENABLE);
}

void MOTOR_Init()
{
		TmrAConfig();
	
		TMRA_SetCompareValue(TMRA_UNIT, TMRA_PWM_CH, 0);
	
		//TMRA_Start(TMRA_UNIT);
}

/**
 * @brief  设置马达震动强度
 * @param  strength: 0 - 100
 *         0   = 关闭
 *         100 = 最大震动
 * @retval None
 */
void MOTOR_SetStrength(uint8_t strength)
{
    uint32_t cmp;

    /* 限幅，防止非法参数 */
    if (strength > 100)
    {
        strength = 100;
    }

    /* 0-100 映射到 0-4999 */
    cmp = (uint32_t)strength * TMRA_PERIOD_VAL / 100U;

    TMRA_SetCompareValue(TMRA_UNIT, TMRA_PWM_CH, cmp);
}
