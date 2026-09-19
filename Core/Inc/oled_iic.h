#ifndef MYIIC_H
#define MYIIC_H

#include "stm32f1xx.h"

/* 引脚配置结构体 */
typedef struct {
    GPIO_TypeDef *Port;
    uint16_t      Pin;
} IIC_PinCfg_t;

/* IIC句柄结构体 */
typedef struct {
    IIC_PinCfg_t  scl;
    IIC_PinCfg_t  sda;
} IIC_HandleTypeDef;

/* 外部实例声明 */
extern IIC_HandleTypeDef IIC1;
//定义基础计算宏
#define SDA_PIN_NUM         (uint8_t)((IIC1.sda.Pin & 0x0F00) >> 8)
#define SDA_REG_ADDR        ((SDA_PIN_NUM < 8) ? (&IIC1.sda.Port->CRL) : (&IIC1.sda.Port->CRH))
#define SDA_PIN_SHIFT       ((SDA_PIN_NUM % 8) * 4)

/* IO操作宏 */
#define IIC_SCL(x)   HAL_GPIO_WritePin(IIC1.scl.Port, IIC1.scl.Pin, \
                                       (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_SDA(x)   HAL_GPIO_WritePin(IIC1.sda.Port, IIC1.sda.Pin, \
                                       (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define IIC_READ_SDA HAL_GPIO_ReadPin(IIC1.sda.Port, IIC1.sda.Pin)

#define SDA_IN()  do { \
    uint32_t tmp = *SDA_REG_ADDR; \
    tmp &= ~(0x0F << SDA_PIN_SHIFT); \
    tmp |= (0x08 << SDA_PIN_SHIFT); \
    *SDA_REG_ADDR = tmp; \
} while(0)

#define SDA_OUT() do { \
    uint32_t tmp = *SDA_REG_ADDR; \
    tmp &= ~(0x0F << SDA_PIN_SHIFT); \
    tmp |= (0x05 << SDA_PIN_SHIFT); \
    *SDA_REG_ADDR = tmp; \
} while(0)

/* IIC所有操作函数 */
void iic_start(void);
void iic_stop(void);
void iic_ack(void);
void iic_nack(void);
uint8_t iic_wait_ack(void);
void iic_send_byte(uint8_t txd);
uint8_t iic_read_byte(unsigned char ack);

#endif