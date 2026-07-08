/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/

#ifndef __OLED_H
#define __OLED_H			  	 

#include "main.h"
#include <stdint.h>

typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define delay_ms HAL_Delay

//-----------------OLED端口定义---------------- 
#define OLED_RST_Clr()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET)   //RST
#define OLED_RST_Set()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET)     //RST

#define OLED_RS_Clr()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)   //DC
#define OLED_RS_Set()    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)     //DC

#define OLED_SCLK_Clr()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET)  //SCL
#define OLED_SCLK_Set()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)    //SCL

#define OLED_SDIN_Clr()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)   //SDA
#define OLED_SDIN_Set()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)     //SDA
#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据





extern u8 OLED_GRAM[128][8];	 

//OLED控制用函数
void OLED_WR_Byte(u8 dat,u8 cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Refresh_Gram(void);		   				   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(u8 x,u8 y,u8 t);
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode);
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size);
void OLED_ShowString(u8 x,u8 y,const u8 *p);	
void OLED_Refresh_GRAM_Line(u8 line);

#endif
// by codex
