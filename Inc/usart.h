/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
extern char *pBuf;

#define NUM_SIZE                    15u
#define USART_BUF_CAPACITY          2000u
#define USART_BUF_SEND_THESHOLD     (USART_BUF_CAPACITY - NUM_SIZE - 30)
#define DEBUG_UART_HANDLER huart1

#define UART_Transmit_DMA(__RESTRICT_STR, ...)                     \
  do                                                       \
  {                                                        \
    static uint32_t __i_divSendTimes = 0;                  \
    if (__i_divSendTimes++ < 1)                            \
    {                                                      \
      __i_divSendTimes = 0;                                \
      pBuf += _sprintf(pBuf, __RESTRICT_STR, __VA_ARGS__); \
    }                                                      \
    SendBufWhenFull();                                 \
  } while (0)

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
  
void SendBuf(void);
void SendBufWaitWhenSending(void);
uint32_t SendBufWhenFull(void);
void USART_BufPush(char ch);
static inline int IntToAscii(int value, char *str);
void PutNum(int data, char endSign);
uint32_t PutStr(const char *str);
void UART_Enable(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
