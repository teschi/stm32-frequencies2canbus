/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define RES_CAN_CTRL_Pin GPIO_PIN_13
#define RES_CAN_CTRL_GPIO_Port GPIOC
#define OUT_DEBUG_SIGNAL0_Pin GPIO_PIN_0
#define OUT_DEBUG_SIGNAL0_GPIO_Port GPIOC
#define OUT_DEBUG_SIGNAL1_Pin GPIO_PIN_1
#define OUT_DEBUG_SIGNAL1_GPIO_Port GPIOC
#define OUT_DEBUG_SIGNAL2_Pin GPIO_PIN_2
#define OUT_DEBUG_SIGNAL2_GPIO_Port GPIOC
#define OUT_DEBUG_SIGNAL3_Pin GPIO_PIN_3
#define OUT_DEBUG_SIGNAL3_GPIO_Port GPIOC
#define OUT_LED_YELLOW_Pin GPIO_PIN_1
#define OUT_LED_YELLOW_GPIO_Port GPIOA
#define OUT_LED_GREEN_Pin GPIO_PIN_5
#define OUT_LED_GREEN_GPIO_Port GPIOA
#define IN_DEBUG_ENABLED_Pin GPIO_PIN_6
#define IN_DEBUG_ENABLED_GPIO_Port GPIOA
#define OUT_DEBUG_ALLWAYS_LOW_Pin GPIO_PIN_7
#define OUT_DEBUG_ALLWAYS_LOW_GPIO_Port GPIOA
#define OUT_DEBUG_SIGNAL4_Pin GPIO_PIN_4
#define OUT_DEBUG_SIGNAL4_GPIO_Port GPIOC
#define IN_BOOT_BTN_Pin GPIO_PIN_9
#define IN_BOOT_BTN_GPIO_Port GPIOC
#define PWM_DEBUG_TIM1_CH1_Pin GPIO_PIN_8
#define PWM_DEBUG_TIM1_CH1_GPIO_Port GPIOA
#define UNUSED_EXTI10_Pin GPIO_PIN_10
#define UNUSED_EXTI10_GPIO_Port GPIOC
#define UNUSED_EXTI10_EXTI_IRQn EXTI15_10_IRQn
#define IN_USB_P_Pin GPIO_PIN_11
#define IN_USB_P_GPIO_Port GPIOC
#define OUT_USB_DISC_Pin GPIO_PIN_12
#define OUT_USB_DISC_GPIO_Port GPIOC
#define RES_TRST_Pin GPIO_PIN_4
#define RES_TRST_GPIO_Port GPIOB
#define PWM_DEBUG_TIM4_CH1_Pin GPIO_PIN_6
#define PWM_DEBUG_TIM4_CH1_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
