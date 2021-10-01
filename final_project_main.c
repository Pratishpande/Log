
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include <cmsis_os2.h>
#include "queue.h"
#include "task.h"

void GPIO_Config(void);
void ADC_Config(void);
void TIM_Config(void);
void Uart_Config(void);

void SysTick_Handler(void);

void ADC_IRQHandler(void);

ADC_HandleTypeDef myAdcHandle;
TIM_HandleTypeDef myTimHandle;
UART_HandleTypeDef myUartHandle;
QueueHandle_t xQueue;


static void vProducer(void *pvParameters);
static void vConsumer(void *pvParameters);

uint16_t adcval;


int main(void)
{
	
	BaseType_t ret=0;
	
	xQueue = xQueueCreate(10,4);

	HAL_Init();
	GPIO_Config();
	ADC_Config();
	TIM_Config();
	Uart_Config();
	
	
	
	HAL_TIM_Base_Start(&myTimHandle);
	HAL_ADC_Start_IT(&myAdcHandle);	

        ret = xTaskCreate( vProducer,"Task 1",240,NULL,1,NULL);
	configASSERT(ret);
   	
        ret = xTaskCreate( vConsumer,"Task 2",240,NULL,1,NULL);
	configASSERT(ret);
	
	vTaskStartScheduler();
	while(1)
	{
		
	}
		
		
	
}



void GPIO_Config(void)
{
	
       __HAL_RCC_GPIOA_CLK_ENABLE();	
        __HAL_RCC_GPIOD_CLK_ENABLE();
	
	GPIO_InitTypeDef myPinInit;
	
	myPinInit.Pin = GPIO_PIN_0;
	myPinInit.Mode = GPIO_MODE_ANALOG;
	
	HAL_GPIO_Init(GPIOA,&myPinInit);
	
	myPinInit.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        myPinInit.Mode = GPIO_MODE_OUTPUT_PP;
	
	HAL_GPIO_Init(GPIOD,&myPinInit);
	
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
	
         HAL_NVIC_SetPriority(ADC_IRQn,0,0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);
	
	
}

void ADC_Config(void)
{
	
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	myAdcHandle.Instance = ADC1;
	myAdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
	myAdcHandle.Init.ContinuousConvMode = DISABLE;
	myAdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	myAdcHandle.Init.DiscontinuousConvMode = DISABLE;
	myAdcHandle.Init.DMAContinuousRequests = DISABLE;
	myAdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	myAdcHandle.Init.ExternalTrigConv = ADC_EXTERNALTRIG2_T2_TRGO;
	myAdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	myAdcHandle.Init.NbrOfConversion = 1;
	myAdcHandle.Init.Resolution = ADC_RESOLUTION12b;
	myAdcHandle.Init.ScanConvMode = DISABLE;
	HAL_ADC_Init(&myAdcHandle);
	
	ADC_ChannelConfTypeDef myChannelConf;
	
	myChannelConf.Channel = ADC_CHANNEL_0;
	myChannelConf.Offset = 0;
	myChannelConf.Rank =1;
	myChannelConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&myAdcHandle,&myChannelConf);
	
	
	
}

void TIM_Config(void)
{
	
        __HAL_RCC_TIM2_CLK_ENABLE();
   	
	myTimHandle.Instance = TIM2;
	myTimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	myTimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	myTimHandle.Init.Period = 1000-1;
	myTimHandle.Init.Prescaler = 16000-1;
	HAL_TIM_Base_Init(&myTimHandle);
	
	
        TIM_ClockConfigTypeDef myClkSrcConfig;
	myClkSrcConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&myTimHandle,&myClkSrcConfig);
	
	TIM_MasterConfigTypeDef myMasterConfig;
	
	myMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	myMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&myTimHandle,&myMasterConfig);
	
	
	
}



void ADC_IRQHandler(void)
{
	HAL_ADC_IRQHandler(&myAdcHandle);
	
}

void HAL_MspInit(void)
{
 
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

 
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
        adcval = HAL_ADC_GetValue(&myAdcHandle);
	HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_13);
}

void Uart_Config(void)
{
	
 __HAL_RCC_USART2_CLK_ENABLE();

 GPIO_InitTypeDef myUartDef;
 myUartDef.Pin = GPIO_PIN_2 | GPIO_PIN_3;
 myUartDef.Mode = GPIO_MODE_AF_PP;
 myUartDef.Pull = GPIO_PULLUP;
 myUartDef.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
 myUartDef.Alternate = GPIO_AF7_USART2;
 HAL_GPIO_Init(GPIOA,&myUartDef);

 myUartHandle.Instance = USART2;
 myUartHandle.Init.BaudRate = 115200;
 myUartHandle.Init.WordLength = UART_WORDLENGTH_8B;
 myUartHandle.Init.Parity = UART_PARITY_NONE;	
 myUartHandle.Init.Mode = UART_MODE_TX_RX;
 myUartHandle.Init.StopBits = UART_STOPBITS_1;
 myUartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
 HAL_UART_Init(&myUartHandle);
	
	
	
}

static void vProducer(void *pvParameters)
{
	
	 TickType_t xLastWakeTime;
	 uint16_t adcvalue1;
	 
	 xLastWakeTime = xTaskGetTickCount();
	
        while(1)	
	{
		adcvalue1 = adcval;

		xQueueSend(xQueue,&adcvalue1,0);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,SET);			
                vTaskDelay(1000);
 	        HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14,RESET);			
		vTaskDelayUntil(&xLastWakeTime,10000);   
 				 
	}
		
		
}


static void vConsumer(void *pvParameters)
{
   uint16_t adcvalue2; 
   char tx_data[17] = "";
	
       while(1)
       {
				 
	 xQueueReceive(xQueue,&adcvalue2,portMAX_DELAY);
	 sprintf(tx_data,"%hu",adcvalue2);
	 tx_data[13]='\0';
	 tx_data[14]='\r';
         tx_data[15]='\n';
         HAL_UART_Transmit(&myUartHandle,(uint8_t *)tx_data,sizeof(tx_data),10);
      	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,SET);			
         vTaskDelay(1000);
 	 HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,RESET);			
         vTaskDelay(1000);   
   		
         
 				 
				 
      }
}

