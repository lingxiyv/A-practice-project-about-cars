
#include "stm32f1xx.h"
#include "delay.h"


static uint32_t g_fac_us = 0;       /* us延时倍乘数 */

/**
 * @brief     初始化延迟函数
 * @retval    无
 */  
void delay_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 使能 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            // 启动周期计数
}


/**
 * @brief     延时nus
 * @param     nus: 要延时的us数
 * @retval    无
 */
void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

/**
 * @brief     延时nms
 * @param     nms: 要延时的ms数 (0< nms <= (2^32 / fac_us / 1000))(fac_us一般等于系统主频, 自行套入计算)
 * @retval    无
 */
void delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
}









