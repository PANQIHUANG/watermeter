#include "stm8l15x.h"//STM8L051/151公用库函数
#include "main.h"
#include <stdlib.h>
#include <string.h>


extern uint8_t received_cmd;

extern uint8_t FLAG_UP0;
extern uint8_t FLAG_DOWN0;
extern uint8_t FLAG_UP1;
extern uint8_t FLAG_DOWN1;
extern uint32_t high_time_pb0;
extern uint32_t high_time_pb1;
extern uint32_t high_time_set;

uint32_t positive_round = 0;
uint32_t negative_round = 0;
uint8_t serial_num = 0;

uint8_t frame_head[19]   = {0x68, 0x38, 0x38, 0x68, 0x08, 0x90, 0x72, 0x78, 0x56, 0x34, 0x12, 0x33, 0x78, 0x01, 0x07, 0x01, 0x00, 0x00, 0x00};
uint8_t positive_flow[6] = {0x0C, 0x13, 0x00, 0x00, 0x00, 0x00};
uint8_t negative_flow[7] = {0x8C, 0x10, 0x13, 0x00, 0x00, 0x00, 0x00};
uint8_t other_bytes[28]  = {0x0C, 0x3B, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x26, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x59, 0x00, 0x00, 0x00, 0x04, 0x6D, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFD, 0x17, 0x00, 0x00};
uint8_t total_bytes[62]  = {0};


void Delay(__IO uint16_t nCount);
static void USART_SYS_Init(uint32_t baudrate);
static void GPIO_SYS_Init(void);
static void TIMER2_SYS_Init(void);
static void TIMER3_SYS_Init(void);
static void CLOCK_SYS_Init(void);
void data_convert(uint32_t inputData, uint8_t *target);
uint8_t sum_check(uint8_t *content, int len);
void usart_send_bytes(uint8_t *data, int len);


void main(void)
{
  CLOCK_SYS_Init();
  GPIO_SYS_Init();
  USART_SYS_Init(4800);
  TIMER2_SYS_Init();
  TIMER3_SYS_Init();
  enableInterrupts(); 

  if(high_time_set == 0)
  {
    high_time_set = 100000;
  }

  while (1)
  {
    if(FLAG_UP0 == 1 && FLAG_DOWN0 == 1)
    {
      if(high_time_pb1 > high_time_set)
      {
        positive_round++;
        high_time_pb1 = 0;
      }
      FLAG_UP0 = 0;
      FLAG_DOWN0 = 0;   
      
    }
    else if(FLAG_UP1 == 1 && FLAG_DOWN1 == 1)
    {
      if(high_time_pb1 > high_time_set)
      {
        negative_round++;
        high_time_pb1 = 0;
      }   
      FLAG_UP1 = 0;
      FLAG_DOWN1 = 0; 
      
    }
    switch(received_cmd)
    {
      case NORMAL_READ_CMD:
        frame_head[15] = serial_num % 256 + 1;
        serial_num ++;
        data_convert(positive_round, (positive_flow + 2));
        data_convert(negative_round, (negative_flow + 3));
        memcpy(total_bytes, frame_head, 19);
        memcpy(total_bytes + 19, positive_flow, 6);
        memcpy(total_bytes + 25, negative_flow, 7);
        memcpy(total_bytes + 32, other_bytes, 28);
        total_bytes[60] = sum_check((total_bytes + 4), 56);
        total_bytes[61] = 0x16;
        usart_send_bytes(total_bytes, 62);   
        received_cmd = 0;
        break;
        
      case CHANGE_BANDWIDTH_CMD:
        
        received_cmd = 0;
        break;
      default:
        break;
    }
       
  }
}

void Delay(__IO uint16_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

static void USART_SYS_Init(uint32_t baudrate)
{
  USART_DeInit(USART1);
  
  CLK_PeripheralClockConfig (CLK_Peripheral_USART1, ENABLE);//开启USART时钟
  USART_Init(USART1, baudrate, USART_WordLength_8b, USART_StopBits_1, USART_Parity_No, (USART_Mode_TypeDef)(USART_Mode_Tx|USART_Mode_Rx));//设置USART参数9600，8N1，接收/发送
  USART_ITConfig (USART1,USART_IT_RXNE, ENABLE);//使能接收中断，中断向量号28
  USART_ITConfig (USART1,USART_IT_IDLE, ENABLE);
  USART_Cmd (USART1, ENABLE);//使能USART
  USART_ClearFlag(USART1, USART_FLAG_RXNE);
  USART_ClearITPendingBit(USART1, USART_IT_RXNE);
  USART_ClearFlag(USART1, USART_FLAG_IDLE);
  USART_ClearITPendingBit(USART1,USART_IT_IDLE);
}

static void CLOCK_SYS_Init(void)
{
  CLK_DeInit();
  CLK_HSEConfig(CLK_HSE_ON);
  while(!CLK_GetFlagStatus(CLK_FLAG_HSERDY));
  CLK_SYSCLKSourceConfig(CLK_SYSCLKSource_HSE);
  CLK_SYSCLKDivConfig(CLK_SYSCLKDiv_1); 
  CLK_SYSCLKSourceSwitchCmd(ENABLE);  
  CLK_ClockSecuritySystemEnable();
  
}

static void GPIO_SYS_Init(void)
{
  GPIO_DeInit(GPIOB);
  GPIO_DeInit(GPIOD);

  GPIO_Init(GPIOB, GPIO_Pin_0, GPIO_Mode_In_FL_IT);
  GPIO_Init(GPIOB, GPIO_Pin_1, GPIO_Mode_In_FL_No_IT);
  
  GPIO_Init(GPIOD, GPIO_Pin_0, GPIO_Mode_Out_OD_Low_Slow);
  
  EXTI_DeInit();
  EXTI_SetPinSensitivity(EXTI_Pin_0, EXTI_Trigger_Rising_Falling);
  
}

static void TIMER2_SYS_Init(void)
{
  TIM2_DeInit();
  
  CLK_PeripheralClockConfig (CLK_Peripheral_TIM2,ENABLE);
  TIM2_TimeBaseInit(TIM2_Prescaler_16, TIM2_CounterMode_Up, 0xC350);
  TIM2_ICInit(TIM2_Channel_1, TIM2_ICPolarity_Rising, TIM2_ICSelection_DirectTI, TIM2_ICPSC_DIV1, 0x00);
  TIM2_ITConfig(TIM2_IT_Update, ENABLE);
  TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
  TIM2_Cmd(ENABLE);
  TIM2_ClearITPendingBit(TIM2_IT_Update);
  TIM2_ClearFlag(TIM2_FLAG_Update);
  TIM2_ClearITPendingBit(TIM2_IT_CC1);
  TIM2_ClearFlag(TIM2_FLAG_CC1);
}

static void TIMER3_SYS_Init(void)
{
  TIM3_DeInit();
  
  CLK_PeripheralClockConfig (CLK_Peripheral_TIM3,ENABLE);
  TIM3_TimeBaseInit(TIM3_Prescaler_16, TIM3_CounterMode_Up, 0xC350);
  TIM3_ICInit(TIM3_Channel_1, TIM3_ICPolarity_Rising, TIM3_ICSelection_DirectTI, TIM3_ICPSC_DIV1, 0x00);
  TIM3_ITConfig(TIM3_IT_Update, ENABLE);
  TIM3_ITConfig(TIM3_IT_CC1, ENABLE);
  TIM3_Cmd(ENABLE);
  TIM3_ClearITPendingBit(TIM3_IT_Update);
  TIM3_ClearFlag(TIM3_FLAG_Update);
  TIM3_ClearITPendingBit(TIM3_IT_CC1);
  TIM3_ClearFlag(TIM3_FLAG_CC1);
}

void data_convert(uint32_t inputData, uint8_t *target)
{
  if(inputData <= 99999999)
  {
    for(int i=0; i<4; i++)
    {
      *(target + i) = inputData % 100;
      inputData = inputData / 100;
    }
  }
  else
  {
    for(int i=0; i<4; i++)
    {
      *(target + i) = inputData & 0xFF;
      inputData = inputData >> 8;
    }
  }
}

uint8_t sum_check(uint8_t *content, int len)
{
  uint8_t sum = 0;
  for(int i = 0; i < len; i++)
  {
    sum += content[i];
  }
  sum = sum & 0xFF;
  
  return sum;  
}

void usart_send_bytes(uint8_t *data, int len)
{
  for(int i = 0; i < len; i++)
  {
    USART_SendData8(USART1, data[i]);
    while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
  }
}
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
