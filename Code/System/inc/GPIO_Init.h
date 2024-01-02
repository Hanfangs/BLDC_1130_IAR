#ifndef  _GPIO_Init_H
#define  _GPIO_Init_H

#ifndef   _GPIO_FILE
#define   GLOBAL_GPIO_   extern
#else
#define   GLOBAL_GPIO_  
#endif

#include "SysConfig.h"

/*设备地址管脚*/
#define LED_PORT                        GPIOB//GPIOB //LED
#define LED_PIN                         GPIO_Pin_10//GPIO_Pin_10
#define ADDR_BIT_LEN                    6//6位
//拨码开关1~6 1为低位6为高位
#define DEVICE_ADDR1_PORT               GPIOB //地址最低位，第0位
#define DEVICE_ADDR1_PIN                GPIO_Pin_9//原理图中D1  //GPIO_Pin_5
#define DEVICE_ADDR2_PORT               GPIOB
#define DEVICE_ADDR2_PIN                GPIO_Pin_3//原理图中D7 //GPIO_Pin_8
#define DEVICE_ADDR4_PORT               GPIOB 
#define DEVICE_ADDR4_PIN                GPIO_Pin_4//原理图中D6 //GPIO_Pin_9
#define DEVICE_ADDR8_PORT               GPIOB
#define DEVICE_ADDR8_PIN                GPIO_Pin_5//原理图中D5//GPIO_Pin_10
#define DEVICE_ADDR16_PORT              GPIOB 
#define DEVICE_ADDR16_PIN               GPIO_Pin_6//原理图中D4//GPIO_Pin_11
#define DEVICE_ADDR32_PORT              GPIOB  //地址第5位
#define DEVICE_ADDR32_PIN               GPIO_Pin_7//原理图中D3//GPIO_Pin_12

#define DEVICE_ADDR64_PORT              GPIOB  //地址第6位，预留测试用
#define DEVICE_ADDR64_PIN               GPIO_Pin_8//原理图中D2//GPIO_Pin_12

 #define   POWER_ON     GPIO_SetBits(GPIOB, GPIO_Pin_3)
 #define   POWER_OFF    GPIO_ResetBits(GPIOB, GPIO_Pin_3)
 #define   POWER_DELAY   2000

#define LED_LIGHTUP     0  //亮灯
#define LED_FLASH       1  //闪灯
#define LED_LIGHTDOWM   2  //灭灯

void BLDC_GPIO_Config(void);
void Exti_Config(void);
#endif


