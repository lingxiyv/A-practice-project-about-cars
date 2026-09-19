
#ifndef DELAY_H
#define DELAY_H

#include "stm32f1xx.h"


void delay_init(void);          /* 初始化延迟函数 */
void delay_ms(uint16_t nms);    /* 延时nms */
void delay_us(uint32_t nus);    /* 延时nus */

#endif



























