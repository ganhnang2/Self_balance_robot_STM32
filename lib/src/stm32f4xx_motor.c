#include "stm32f4xx_motor.h"
#include "stm32f4xx_gpio.h"
#include "pin_config.h"
#include <string.h>   /* memset */

/* =========================================================================
 * Global Encoder State (file-scope only — accessed via getter functions)
 * ========================================================================= */
static Encoder_t g_EncLeft;
static Encoder_t g_EncRight;

/* =========================================================================
 * TB6612_Init
 * Configures: Direction pins (PB12-PB15), PWM pins (PB4/PB5 AF2→TIM3)
 *             and TIM3 in PWM Mode 1 at 12.5 kHz.
 * ========================================================================= */
void TB6612_Init(void)
{
    /***********************PWM (TIM3) *****************************/
    /* Enable TIM3 clock (APB1) */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* fTIM3 = SystemCoreClock / (PSC + 1)
     * fPWM  = fTIM3 / (ARR + 1)  => ARR = fTIM3 / fPWM - 1 
	 * Clock source: fSystem = 25MHz, PRES = 0 and fPWM = 12.5kHz -> ARR = 2000 */
    TIM3->PSC  = PWM_PRES_VAL;
    TIM3->ARR  = (SystemCoreClock / (PWM_PRES_VAL + 1) / PWM_FREQ) - 1;

    /* PWM Mode 1 on CH1 and CH2, preload enabled */
    TIM3->CCMR1 &= ~((0x07 << TIM_CCMR1_OC1M_Pos) | (0x07 << TIM_CCMR1_OC2M_Pos));
    TIM3->CCMR1 |=  ((0x06 << TIM_CCMR1_OC1M_Pos) | (0x06 << TIM_CCMR1_OC2M_Pos));
    TIM3->CCMR1 |=  TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;

    /* Clear CCxNP (output channel: must be 0) then enable CC1 and CC2 outputs */
    TIM3->CCER &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP);
    TIM3->CCER |=  (TIM_CCER_CC1E  | TIM_CCER_CC2E); 

    /* Initial duty cycle = 0 */
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;

    /* Enable auto-reload preload and start counter */
    TIM3->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN;
}

/* =========================================================================
 * TB6612_Control_Motors
 * ========================================================================= */
void TB6612_Control_Motors(int32_t leftSpeed, int32_t rightSpeed)
{
    /* Right motor (AIN1=PB13, AIN2=PB12, PWM=TIM3_CH1) */
    if (rightSpeed > 0)
    {
        GPIO_Write_Pin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   	/* AIN1 HIGH   */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);     /* AIN2 LOW   */
        TIM3->CCR1 = (uint32_t)rightSpeed;
    }
    else if (rightSpeed < 0)
    {
        GPIO_Write_Pin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 	/* AIN1 LOW  */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);  		/* AIN2 HIGH */
        TIM3->CCR1 = (uint32_t)(-rightSpeed);
    }
    else
    {
		TIM3->CCR1 = 0;
        GPIO_Write_Pin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 	/* AIN1 LOW  */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 	/* AIN2 LOW  */
    }

    /* Left motor (BIN1=PB14, BIN2=PB15, PWM=TIM3_CH2) */
    if (leftSpeed > 0)
    {
        GPIO_Write_Pin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);   	/* BIN1 HIGH   */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 	/* BIN2 LOW    */
        TIM3->CCR2 = (uint32_t)leftSpeed;
    }
    else if (leftSpeed < 0)
    {
        GPIO_Write_Pin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 	/* BIN1 LOW  */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);   	/* BIN2 HIGH */
        TIM3->CCR2 = (uint32_t)(-leftSpeed);
    }
    else
    {
		TIM3->CCR2 = 0;
        GPIO_Write_Pin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); /* BIN1 LOW  */
        GPIO_Write_Pin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); /* BIN2 LOW  */
    }
}

/* =========================================================================
 * TB6612_Short_Brake
 * ========================================================================= */
void TB6612_Short_Brake(void)
{
    /* AIN1=H, AIN2=H → short brake right */
	GPIO_Write_Pin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);		
	GPIO_Write_Pin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);	
    /* BIN1=H, BIN2=H → short brake left */
	GPIO_Write_Pin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);		
	GPIO_Write_Pin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);		

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

/* =========================================================================
 * TB6612_Coast
 * ========================================================================= */
void TB6612_Coast(void)
{
    /* AIN1=L, AIN2=L → coast right */
    GPIO_Write_Pin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);		
	GPIO_Write_Pin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	
    /* BIN1=L, BIN2=L → coast left */
	GPIO_Write_Pin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);		
	GPIO_Write_Pin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
}

/* =========================================================================
 * Motor_Encoder_Init
 * Configures: PA8/PA9 (AF1 → TIM1_CH1/CH2) for Left encoder
 *             PB6/PB7 (AF2 → TIM4_CH1/CH2) for Right encoder
 *             TIM1 and TIM4 in Encoder Interface mode 3 (x4 quadrature)
 * ========================================================================= */
void Motor_Encoder_Init(void)
{
    /***********************ENCODER TIMERS (TIM1, TIM4)**************************/
    /* Enable TIM1 (APB2) and TIM4 (APB1) clocks */
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    /* TIM1 — Left Motor Encoder (PA8=CH1, PA9=CH2)
     * Encoder mode 3: count on both TI1 and TI2 edges (x4 quadrature) */
    TIM1->SMCR  &= ~TIM_SMCR_SMS;
    TIM1->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM1->CCMR1 |=  (0x01 << TIM_CCMR1_CC1S_Pos)   		/* IC1 mapped on TI1 */
                 |  (0x01 << TIM_CCMR1_CC2S_Pos);   	/* IC2 mapped on TI2 */
    TIM1->CCER  &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP);  /* CC1NP=0, CC2NP=0 */
	TIM1->CCER  &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P); 	/* CC1P=0, CC2P=0 */
    TIM1->SMCR  |=  (0x03 << TIM_SMCR_SMS_Pos);    		/* Encoder mode 3 */
    TIM1->ARR    = 0xFFFF;                            	/* 16-bit full range */
    TIM1->CNT    = 0;
	TIM1->BDTR |= (1 << TIM_BDTR_MOE_Pos);				/* Main output enable (Only if TIM1, TIM8) */
    TIM1->CR1   |= TIM_CR1_CEN;

    /* TIM4 — Right Motor Encoder (PB6=CH1, PB7=CH2) */
    TIM4->SMCR  &= ~TIM_SMCR_SMS;
    TIM4->CCMR1 &= ~(TIM_CCMR1_CC1S | TIM_CCMR1_CC2S);
    TIM4->CCMR1 |=  (0x01 << TIM_CCMR1_CC1S_Pos)          /* IC1 mapped on TI1 (PB6) */
                 |  (0x01 << TIM_CCMR1_CC2S_Pos);         /* IC2 mapped on TI2 (PB7) */
    TIM4->CCER  &= ~(TIM_CCER_CC1NP | TIM_CCER_CC2NP);    /* CC1NP=0, CC2NP=0 */
    TIM4->CCER  &= ~(TIM_CCER_CC1P  | TIM_CCER_CC2P);     /* CC1P=0,  CC2P=0  */
    TIM4->SMCR  |=  (0x03 << TIM_SMCR_SMS_Pos);           /* Encoder mode 3 */
    TIM4->ARR    = 0xFFFF;                                /* 16-bit full range */
    TIM4->CNT    = 0;
    TIM4->CR1   |= TIM_CR1_CEN;
	
	/* Reset motor encoder	*/
	Motor_Encoder_Reset();
}

/* =========================================================================
 * Motor_Encoder_Update
 * Read TIM1 (Left) and TIM4 (Right) CNT registers and compute signed delta
 * using wrap-around subtraction — handles overflow/underflow with no if-else.
 * ========================================================================= */
void Motor_Encoder_Update(void)
{
    /* --- Left wheel: TIM1 (16-bit) --- */
    uint16_t leftCnt = (uint16_t)TIM1->CNT;
    /* (uint16_t)(cur - prev) wraps correctly in [0, 65535].
     * Casting to int16_t reinterprets values > 32767 as negative → backward. */
    int16_t  leftDelta = (int16_t)(uint16_t)(leftCnt - g_EncLeft.prevCnt);
    g_EncLeft.prevCnt     = leftCnt;
    g_EncLeft.currCnt     = leftCnt;
    g_EncLeft.deltaCounts = (int32_t)leftDelta;
	
    /* --- Right wheel: TIM4 (16-bit) --- */
    uint16_t rightCnt = (uint16_t)TIM4->CNT;
    int16_t  rightDelta = (int16_t)(uint16_t)(rightCnt - g_EncRight.prevCnt);
    g_EncRight.prevCnt     = rightCnt;
    g_EncRight.currCnt     = rightCnt;
    g_EncRight.deltaCounts = (int32_t)rightDelta;
}

/* =========================================================================
 * Motor_Encoder_Reset
 * ========================================================================= */
void Motor_Encoder_Reset(void)
{
    /* Re-sync prevCnt so the next Update() gives delta = 0 */
	TIM1->CNT = 0;			
	TIM4->CNT = 0;
    g_EncLeft.prevCnt  = 0;
    g_EncRight.prevCnt = 0;
	g_EncLeft.currCnt = 0;
	g_EncRight.currCnt = 0;

    g_EncLeft.deltaCounts  = 0;
    g_EncRight.deltaCounts = 0;
}

/* =========================================================================
 * Motor_Get_Encoder_Left / Right
 * Return the signed delta pulse count from the last Motor_Encoder_Update().
 * ========================================================================= */
int32_t Motor_Get_Encoder_Left(void)
{
    return g_EncLeft.deltaCounts;
}

int32_t Motor_Get_Encoder_Right(void)
{
    return g_EncRight.deltaCounts;
}

uint32_t Motor_Get_Encoder_Counter_Left(void)
{
	return (uint32_t)g_EncLeft.currCnt;
}

uint32_t Motor_Get_Encoder_Counter_Right(void)
{
	return (uint32_t)g_EncRight.currCnt;
}