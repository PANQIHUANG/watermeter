/**
  ******************************************************************************
  * @file    Project/STM8L15x_StdPeriph_Template/stm8l15x_it.c
  * @author  MCD Application Team
  * @version V1.6.1
  * @date    30-September-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm8l15x_it.h"
#include "main.h"
#include <string.h>

/** @addtogroup STM8L15x_StdPeriph_Template
  * @{
  */
	
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

#ifdef _COSMIC_
/**
  * @brief Dummy interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER(NonHandledInterrupt,0)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
#endif

/**
  * @brief TRAP interrupt routine
  * @par Parameters:
  * None
  * @retval 
  * None
*/
INTERRUPT_HANDLER_TRAP(TRAP_IRQHandler)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief FLASH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(FLASH_IRQHandler,1)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel0 and channel1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL0_1_IRQHandler,2)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief DMA1 channel2 and channel3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(DMA1_CHANNEL2_3_IRQHandler,3)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief RTC / CSS_LSE Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(RTC_CSSLSE_IRQHandler,4)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief External IT PORTE/F and PVD Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIE_F_PVD_IRQHandler,5)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTB / PORTG Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTIB_G_IRQHandler,6)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PORTD /PORTH Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTID_H_IRQHandler,7)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN0 Interrupt routine.
  * @param  None
  * @retval None
  */

uint8_t PB_DR           = 0xFF;
uint8_t FLAG0_UP0        = 0;
uint8_t FLAG0_DOWN0      = 0;
uint8_t FLAG0_UP1        = 0;
uint8_t FLAG0_DOWN1      = 0;

INTERRUPT_HANDLER(EXTI0_IRQHandler,8)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  PB_DR = GPIOB->IDR;
  
  if(PB_DR == 0x01)
  {
    FLAG0_UP0 = 1;
  }
  else if(PB_DR == 0x02)
  {
    FLAG0_DOWN0 = 1;
  }
  else if(PB_DR == 0x03)
  {
    FLAG0_UP1 = 1;
  }
  else if(PB_DR == 0x00)
  {
    FLAG0_DOWN1 = 1;
  }
 
  EXTI_ClearITPendingBit(EXTI_IT_Pin0);
}

/**
  * @brief External IT PIN1 Interrupt routine.
  * @param  None
  * @retval None
  */

uint8_t FLAG1_UP0        = 0;
uint8_t FLAG1_DOWN0      = 0;
uint8_t FLAG1_UP1        = 0;
uint8_t FLAG1_DOWN1      = 0;

INTERRUPT_HANDLER(EXTI1_IRQHandler,9)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  
  PB_DR = GPIOB->IDR;
  
  if(PB_DR == 0x03)
  {
    FLAG1_UP0 = 1;
  }
  else if(PB_DR == 0x00)
  {
    FLAG1_DOWN0 = 1;
  }
  else if(PB_DR == 0x02)
  {
    FLAG1_UP1 = 1;
  }
  else if(PB_DR == 0x01)
  {
    FLAG1_DOWN1 = 1;
  }
 
  EXTI_ClearITPendingBit(EXTI_IT_Pin1);
}

/**
  * @brief External IT PIN2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI2_IRQHandler,10)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN3 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI3_IRQHandler,11)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN4 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI4_IRQHandler,12)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN5 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI5_IRQHandler,13)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN6 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI6_IRQHandler,14)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief External IT PIN7 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(EXTI7_IRQHandler,15)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief LCD /AES Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(LCD_AES_IRQHandler,16)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief CLK switch/CSS/TIM1 break Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SWITCH_CSS_BREAK_DAC_IRQHandler,17)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief ADC1/Comparator Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(ADC1_COMP_IRQHandler,18)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM2 Update/Overflow/Trigger/Break /USART2 TX Interrupt routine.
  * @param  None
  * @retval None
  */

uint8_t capture_status_pb0 = 0;
uint32_t update_times_pb0 = 0;

INTERRUPT_HANDLER(TIM2_UPD_OVF_TRG_BRK_USART2_TX_IRQHandler,19)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  if(TIM2_GetITStatus(TIM2_IT_Update) != RESET)
  {
    TIM2_ClearITPendingBit(TIM2_IT_Update);
    if(capture_status_pb0 == 1)
    {
      update_times_pb0 ++;
    }
    
  }
  
}

/**
  * @brief Timer2 Capture/Compare / USART2 RX Interrupt routine.
  * @param  None
  * @retval None
  */

uint32_t high_time_pb0 = 0;

INTERRUPT_HANDLER(TIM2_CC_USART2_RX_IRQHandler,20)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
    uint16_t capture_data1_pb0, capture_data2_pb0;
  if(TIM2_GetITStatus(TIM2_IT_CC1) != RESET)
  {
    TIM2_ClearITPendingBit(TIM2_IT_CC1);
    switch(capture_status_pb0)
    {
      case 0:
        capture_data1_pb0 = TIM2_GetCounter();
        TIM2->CCER1 |= TIM_CCER1_CC1P;
        capture_status_pb0 = 1;
      
        break;
      case 1:
        capture_data2_pb0 = TIM2_GetCounter();
        TIM2->CCER1 &= (uint8_t)(~TIM_CCER1_CC1P);
        high_time_pb0 = capture_data1_pb0 + update_times_pb0*50000 + capture_data2_pb0;
        capture_status_pb0 = 0;
        update_times_pb0 = 0;
        
        break;
      default:
        break;
    }
  }
}


/**
  * @brief Timer3 Update/Overflow/Trigger/Break Interrupt routine.
  * @param  None
  * @retval None
  */

uint8_t capture_status_pb1 = 0;
uint32_t update_times_pb1 = 0;

INTERRUPT_HANDLER(TIM3_UPD_OVF_TRG_BRK_USART3_TX_IRQHandler,21)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  if(TIM3_GetITStatus(TIM3_IT_Update) != RESET)
  {
    TIM3_ClearITPendingBit(TIM3_IT_Update);
    if(capture_status_pb1 == 1)
    {
      update_times_pb1 ++;
    }
    
  }
  
}
/**
  * @brief Timer3 Capture/Compare /USART3 RX Interrupt routine.
  * @param  None
  * @retval None
  */

uint32_t high_time_pb1 = 0;

INTERRUPT_HANDLER(TIM3_CC_USART3_RX_IRQHandler,22)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  uint16_t capture_data1_pb1, capture_data2_pb1;
  if(TIM3_GetITStatus(TIM3_IT_CC1) != RESET)
  {
    TIM3_ClearITPendingBit(TIM3_IT_CC1);
    switch(capture_status_pb1)
    {
      case 0:
        capture_data1_pb1 = TIM3_GetCounter();
        TIM3->CCER1 |= TIM_CCER1_CC1P;
        capture_status_pb1 = 1;
      
        break;
      case 1:
        capture_data2_pb1 = TIM3_GetCounter();
        TIM3->CCER1 &= (uint8_t)(~TIM_CCER1_CC1P);
        high_time_pb1 = capture_data1_pb1 + update_times_pb1*50000 + capture_data2_pb1;
        capture_status_pb1 = 0;
        update_times_pb1 = 0;
        
        break;
      default:
        break;
    }
  }
  
}
/**
  * @brief TIM1 Update/Overflow/Trigger/Commutation Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_UPD_OVF_TRG_COM_IRQHandler,23)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief TIM1 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM1_CC_IRQHandler,24)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief TIM4 Update/Overflow/Trigger Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(TIM4_UPD_OVF_TRG_IRQHandler,25)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @brief SPI1 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(SPI1_IRQHandler,26)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */		
}

/**
  * @brief USART1 TX / TIM5 Update/Overflow/Trigger/Break Interrupt  routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(USART1_TX_TIM5_UPD_OVF_TRG_BRK_IRQHandler,27)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}

/**
  * @brief USART1 RX / Timer5 Capture/Compare Interrupt routine.
  * @param  None
  * @retval None
  */

uint8_t buf_size = 0;
uint8_t receive_buf[16] = {0};
uint8_t received_cmd = 0;
uint8_t normal_read[5] = {0x10, 0x5B, 0xFE, 0x59, 0x16};
uint8_t change_bandwidth_head[6] = {0x68, 0x09, 0x09, 0x68, 0x53, 0xFE};//{0x68, 0x09, 0x09, 0x68, 0x53, 0xFE, 0x50, 0x00, 0xA1, 0x16};
uint32_t high_time_set = 0;

INTERRUPT_HANDLER(USART1_RX_TIM5_CC_IRQHandler,28)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
  if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
  {
    USART_ClearITPendingBit (USART1,USART_IT_RXNE);//清中断标志
    receive_buf[buf_size] = USART_ReceiveData8(USART1);
    buf_size++;
  }
  else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
  {
    USART_ClearITPendingBit(USART1, USART_IT_IDLE);
    
    USART1->SR;
    USART1->DR;

    if(memcmp(receive_buf, normal_read, buf_size) == 0)
    {
      received_cmd = NORMAL_READ_CMD;
      memset(receive_buf, 0, sizeof(receive_buf));
      buf_size = 0;
    }
    else if((memcmp(receive_buf, change_bandwidth_head, sizeof(change_bandwidth_head)) == 0) && buf_size == 12)
    {
      received_cmd = CHANGE_BANDWIDTH_CMD;
      uint8_t tmp[4];
      tmp[0] = receive_buf[9];
      tmp[1] = receive_buf[8];
      tmp[2] = receive_buf[7];
      tmp[3] = receive_buf[6];
      
      high_time_set = *(uint32_t *)tmp;
      memset(receive_buf, 0, sizeof(receive_buf));
      buf_size = 0;
    }

  }
  
  if(buf_size > 12)
  {
    memset(receive_buf, 0, sizeof(receive_buf));
    buf_size = 0;
  }

}

/**
  * @brief I2C1 / SPI2 Interrupt routine.
  * @param  None
  * @retval None
  */
INTERRUPT_HANDLER(I2C1_SPI2_IRQHandler,29)
{
    /* In order to detect unexpected events during development,
       it is recommended to set a breakpoint on the following instruction.
    */
}
/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/