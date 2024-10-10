/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include "event_groups.h"
#include "ws2812Frame.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
const char *model1 = "颜色控制";  //  Chinese
const char *model2 = "呼吸灯 ";  //  Chinese
const char *model3 = "流水灯 ";  //  Chinese
const char *model4 = "跑马灯 ";  //  Chinese
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
TaskHandle_t myUARTtaskhandler;
TaskHandle_t mySPItaskhandler;
TaskHandle_t myledcontroltaskhandler;
TaskHandle_t myledbreathtaskhandler;
TaskHandle_t myledstreamtaskhandler;
TaskHandle_t myledpumataskhandler;

uint8_t breath_whileflag = 0;
uint8_t puma_whileflag = 0;
uint8_t stream_whileflag = 0;

QueueHandle_t myqueueforled;
//QueueHandle_t myqueueforcontrol;
//QueueHandle_t myqueueforbreath;
//QueueHandle_t myqueueforstream;
//QueueHandle_t myqueueforpuma;

SemaphoreHandle_t my_UART_mutex;
SemaphoreHandle_t my_SPI_mutex;
//SemaphoreHandle_t my_LED_mutex;
//SemaphoreHandle_t myledcountSemaphore;
EventGroupHandle_t myledeventgroup;

uint8_t rxData[18]; //发送18位数据包含2个0x的crc校验码
size_t  rxSize = sizeof(rxData);
uint8_t txData[18];//只要前16位
size_t  txSize = sizeof(txData);
static uint8_t txData_copy[18]; //全局变量保护
uint16_t crc_temple;
uint8_t crc[10];
uint32_t sum;
uint32_t sum_last;
//uint8_t reset[24];


//WS2812_INIT(my2_ws2812,1,reset,WS2812SendMassge);
#if 0
struct DATA
{
	uint8_t data[18];
};
struct DATA rxData;
size_t  rxSize = sizeof(rxData);
struct DATA txData;
size_t  txSize = sizeof(txData);
uint16_t crc_temple;
uint8_t crc[10];
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#if 0
void mytask1(void *arg)
{
	while(1)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_Delay(500);
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_Delay(500);
	}
}
#endif

uint32_t multiplyLastTwoHexPairs(uint8_t *data)
{
    // 获取最后4个字节，将它们组合成两个 16 位整数
    uint16_t num1 = (data[12] << 8) | data[13]; // 组合高低位，0xFF01
    uint16_t num2 = (data[14] << 8) | data[15]; // 组合高低位，0x3221

    // 计算乘积
    uint32_t product = (uint32_t)num1 * (uint32_t)num2;

    return product;
}

void InvertUint8(unsigned char* DesBuf, unsigned char* SrcBuf)
{
    int i;
    unsigned char temp = 0;

    for (i = 0; i < 8; i++)
    {
        if (SrcBuf[0] & (1 << i))
        {
            temp |= 1 << (7 - i);
        }
    }
    DesBuf[0] = temp;
}

void InvertUint16(unsigned short* DesBuf, unsigned short* SrcBuf)
{
    int i;
    unsigned short temp = 0;

    for (i = 0; i < 16; i++)
    {
        if (SrcBuf[0] & (1 << i))
        {
            temp |= 1 << (15 - i);
        }
    }
    DesBuf[0] = temp;
}

unsigned short CRC16_CCITT(unsigned char* puchMsg, unsigned int usDataLen)
{
    unsigned short wCRCin = 0x0000;
    unsigned short wCPoly = 0x1021;
    unsigned char wChar = 0;

    while (usDataLen--)
    {
        wChar = *(puchMsg++);
        InvertUint8(&wChar, &wChar);
        wCRCin ^= (wChar << 8);

        for (int i = 0; i < 8; i++)
        {
            if (wCRCin & 0x8000)
            {
							wCRCin = (wCRCin << 1) ^ wCPoly;//CRC的第一位舍去
            }
            else
            {
                wCRCin = wCRCin << 1;
            }
        }
    }
    InvertUint16(&wCRCin, &wCRCin);
    return (wCRCin);
}


void	waituart(void)
{
	xSemaphoreTake(my_UART_mutex,portMAX_DELAY);
}

void releaseuart(void)
{
	xSemaphoreGive(my_UART_mutex);
}

void	waitspi(void)
{
	xSemaphoreTake(my_SPI_mutex,portMAX_DELAY);
}

void releasespi(void)
{
	xSemaphoreGive(my_SPI_mutex);
}

#if 0
void	waitled(void)
{
	xSemaphoreTake(my_LED_mutex,portMAX_DELAY);
}

void releaseled(void)
{
	xSemaphoreGive(my_LED_mutex);
}
#endif

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // 处理接收到的数据 
				 breath_whileflag = 1;
				 puma_whileflag = 1;
				 stream_whileflag = 1;
				HAL_UART_Receive_IT(&huart1,rxData,rxSize);
				xQueueSendToBackFromISR(myqueueforled,&rxData,NULL);   
    }
}

#if 0
void uarttask(void *arg)
{
	waituart();
	UART_SendString(&huart1, "uarttask1");
	releaseuart();
	HAL_UART_Receive_IT(&huart1,rxData,rxSize);
	while(1)
	{
		vTaskDelay(10);
	}
}
#endif

void spitask(void *arg)
{
	waituart();
	UART_SendString(&huart1, "spitask");
	releaseuart();
	HAL_UART_Receive_IT(&huart1,rxData,rxSize);
	while(1)
	{
		//memcpy(txData_copy, rxData, 16);
		xQueueReceive(myqueueforled,txData,portMAX_DELAY);
		//在其他灯带任务阻塞时候，先计算crc，通过运行效率
		crc_temple = CRC16_CCITT(txData,16);
		crc[0] = (crc_temple >>8 );   //高8位
		crc[1] = (crc_temple & 0xFF);	//低8位
		
		//新数据前先把之前灯带设置的rgb设置为0，sum为上次的灯带数目
		waitspi();
		if(sum != 0)
				{
								uint8_t color_zero[(sum/3)*24];
								WS2812_INIT(my2_ws2812,1,color_zero,WS2812SendMassge);
								for(uint32_t i=0;i<(sum/3);i++)
								{
									SetWSColor(&my2_ws2812, i, 0, 0,  0);  // RGB
								//SetWSColor(&my_ws2812, 1, 255, 0, 0);  // 将第二个灯珠设为绿色
								}

					HAL_SPI_Transmit(&hspi1, my2_ws2812.sendBuff, sum/3*24,HAL_MAX_DELAY);
				}
		releasespi();
				
		memcpy(txData_copy, txData, 16);
		if(txData[0]==0x48)
		{		
				waituart();
				UART_SendString(&huart1, "CRC_RESULT:");
				if(crc[0] == txData[16] && crc[1] == txData[17])
				{
					//waituart();
					UART_SendString(&huart1, "TRUE");
					//releaseuart();
				}
				else
				{
					UART_SendString(&huart1, "FALSE");
				}
				UART_SendString(&huart1, "\t");
				UART_SendString(&huart1, "Model_Set:");			
				
				switch(txData[4])
				{
					case 0:xEventGroupSetBits(myledeventgroup,(1<<0)),UART_SendUTF8String(&huart1, model1);break;
					case 1:xEventGroupSetBits(myledeventgroup,(1<<1)),UART_SendUTF8String(&huart1, model2);break;
					case 2:xEventGroupSetBits(myledeventgroup,(1<<2)),UART_SendUTF8String(&huart1, model3);break;
					case 3:xEventGroupSetBits(myledeventgroup,(1<<3)),UART_SendUTF8String(&huart1, model4);break;
				}
#if 0
				if(txData[4]==0)
					xEventGroupSetBits(myledeventgroup,(1<<0)),UART_SendString(&huart1, "sended0");
				else if(txData[4]==1)
					xEventGroupSetBits(myledeventgroup,(1<<1)),UART_SendString(&huart1, "sended1");
				else if(txData[4]==2)
					xEventGroupSetBits(myledeventgroup,(1<<2)),UART_SendString(&huart1, "sended2");
				else if(txData[4]==3)
					xEventGroupSetBits(myledeventgroup,(1<<3)),UART_SendString(&huart1, "sended3");
#endif
				//txData[0] = 0x00;
				releaseuart();
		}
			
				//releaseuart();
		vTaskDelay(100);
	}
}

void led_control_task(void *arg)
{
	while(1)
	{	
	xEventGroupWaitBits(myledeventgroup,(1<<0),pdTRUE,pdFALSE,portMAX_DELAY);
	//waitled();
	sum = multiplyLastTwoHexPairs(txData_copy);
	waituart();
	UART_SendString(&huart1, "\t");
	UART_SendString(&huart1, "LIGHT_Number:");
	UART_SendIntString(&huart1,sum);
	releaseuart();
	if(sum != 0)
	{
		uint8_t color[(sum/3)*24];
		WS2812_INIT(my_ws2812,1,color,WS2812SendMassge);
		for(uint32_t i=0;i<(sum/3);i++)
		{
				SetWSColor(&my_ws2812, i, txData_copy[9],  txData_copy[10],  txData_copy[11]);  // RGB
			//SetWSColor(&my_ws2812, 1, 255, 0, 0);  // 将第二个灯珠设为绿色
		}
			// Send "ok" after receiving the complete packet
		waitspi();
		HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, sum/3*24,HAL_MAX_DELAY);
		releasespi();
	}
	vTaskDelay(100);
	}

}

void led_breath_task(void *arg)
{
	while(1)
	{	
	xEventGroupWaitBits(myledeventgroup,(1<<1),pdTRUE,pdFALSE,portMAX_DELAY);
	sum = multiplyLastTwoHexPairs(txData_copy);
	waituart();
	breath_whileflag=0;
	UART_SendString(&huart1, "\t");
	UART_SendString(&huart1, "LIGHT_Number:");
	UART_SendIntString(&huart1,sum);
	releaseuart();
	if(sum != 0)
		{ 		
			uint8_t color[(sum/3)*24];
			WS2812_INIT(my_ws2812,1,color,WS2812SendMassge);
			while(1)
			{		
				if (breath_whileflag)
					{
							// 清除标志
									breath_whileflag = 0;
							// 退出循环或执行其他操作
							break;
					}
					//waitled();
					waitspi();
							//pwm 上升
						for(uint8_t j=10;j>0;j--)
							{
									for(uint32_t i=0;i<(sum/3);i++)
									{
										SetWSColor(&my_ws2812, i, rxData[9]/j,  rxData[10]/j,  rxData[11]/j);  // RGB
									}
									
									HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, sum/3*24,HAL_MAX_DELAY);											
									vTaskDelay(100);
									if (breath_whileflag)
														{		
																break;
														}
							}
					releasespi();
					if (breath_whileflag)
												{
														// 清除标志
																breath_whileflag = 0;
														// 退出循环或执行其他操作
														break;
												}
						waitspi();				
						//pwm下降
						for(uint8_t j=1;j<11;j++)
						{
								for(uint32_t i=0;i<(sum/3);i++)
								{
									SetWSColor(&my_ws2812, i, rxData[9]/j,  rxData[10]/j,  rxData[11]/j);  // RGB
								}
								
								HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, sum/3*24,HAL_MAX_DELAY);
								
								vTaskDelay(100);
								
								if (breath_whileflag)
														{
																break;
														}
						}

					releasespi();
			}

		}
		vTaskDelay(100);
	}
}

void led_stream_task(void *arg)
{
	while(1)
	{	
		xEventGroupWaitBits(myledeventgroup,(1<<2),pdTRUE,pdFALSE,portMAX_DELAY);
		sum = multiplyLastTwoHexPairs(txData_copy);
		waituart();
		stream_whileflag=0;
		UART_SendString(&huart1, "\t");
		UART_SendString(&huart1, "LIGHT_Number:");
		UART_SendIntString(&huart1,sum);
		releaseuart();

			if(sum != 0)
			{ 
				uint8_t color[(sum/3)*24];
				WS2812_INIT(my_ws2812,1,color,WS2812SendMassge);
				while(1)
				{
					if (stream_whileflag)
								{
										// 清除标志
												stream_whileflag = 0;

										// 退出循环或执行其他操作
										break;
								}
					
					waitspi();
					for(uint32_t i=0;i<(sum/3);i++)
					{
							SetWSColor(&my_ws2812, i, txData_copy[9],  txData_copy[10],  txData_copy[11]);  // RGB
							HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, (i+1)*24,HAL_MAX_DELAY);
							vTaskDelay(200);
						
							if (stream_whileflag)
									{								
											break;
									}
					}
					releasespi();
					
					if (stream_whileflag)
								{
										// 清除标志
												stream_whileflag = 0;

										// 退出循环或执行其他操作
										break;
								}
						
					waitspi();
					for(int32_t j=(sum/3)-1;j>=0;j--)
					{
							SetWSColor(&my_ws2812, j, 0,  0,  0);  // RGB
							HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, (sum/3)*24,HAL_MAX_DELAY);
							vTaskDelay(200);
							if (stream_whileflag)
									{								
											break;
									}
					}
					releasespi();
				}
				
			}
						 
		vTaskDelay(100);
	}

}

void led_puma_task(void *arg)
{
	while(1)
	{	
		xEventGroupWaitBits(myledeventgroup,(1<<3),pdTRUE,pdFALSE,portMAX_DELAY);
		sum = multiplyLastTwoHexPairs(txData_copy);
		waituart();
		puma_whileflag=0;
		UART_SendString(&huart1, "\t");
		UART_SendString(&huart1, "LIGHT_Number:");
		UART_SendIntString(&huart1,sum);
		releaseuart();	
		if(sum != 0)
							{ 
								uint8_t color[(sum/3)*24];
								WS2812_INIT(my_ws2812,1,color,WS2812SendMassge);
								while(1)
								{
									if (puma_whileflag)
												{
														// 清除标志
																puma_whileflag = 0;

														// 退出循环或执行其他操作
														break;
												}
									
									for(uint32_t i=0;i<(sum/3);i++)
									{
											for(uint32_t j=0;j<(sum/3);j++)
											{
													SetWSColor(&my_ws2812, j, 0,  0,  0);  // RGB
											}
											SetWSColor(&my_ws2812, i, txData_copy[9],  txData_copy[10],  txData_copy[11]);  // RGB
											HAL_SPI_Transmit(&hspi1, my_ws2812.sendBuff, (sum/3)*24,HAL_MAX_DELAY);
											vTaskDelay(200);
											if (puma_whileflag)
												{								
														break;
												}
									}		
								}							
							}
		
		vTaskDelay(100);
	}

}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
#if 0
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 32,
  .priority = (osPriority_t) osPriorityLow,
};
#endif
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	my_UART_mutex = xSemaphoreCreateMutex();
	my_SPI_mutex = xSemaphoreCreateMutex();
	//my_LED_mutex = xSemaphoreCreateMutex();
	myqueueforled = xQueueCreate(5,rxSize);
	//UART_SendIntString(&huart1,rxSize);
	myledeventgroup = xEventGroupCreate();
	//myledcountSemaphore = xSemaphoreCreateCounting(4,1); /* USER CODE END Init */
	//设置rgb为0的数据帧
	//SetWSColor(&my2_ws2812, 0, 0, 0, 0);
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  //defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
	xTaskCreate(spitask,"spitask",64,NULL,osPriorityNormal+1,&mySPItaskhandler);
	//xTaskCreate(uarttask,"uarttask",64,NULL,osPriorityNormal,&myUARTtaskhandler);
	xTaskCreate(led_control_task,"led_control_task",64,NULL,osPriorityNormal,&myledcontroltaskhandler);
	xTaskCreate(led_breath_task,"led_breath_task",64,NULL,osPriorityNormal,&myledbreathtaskhandler);
	xTaskCreate(led_stream_task,"led_stream_task",64,NULL,osPriorityNormal,&myledstreamtaskhandler);
	xTaskCreate(led_puma_task,"led_puma_task",64,NULL,osPriorityNormal,&myledpumataskhandler);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for(;;)
  {

    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

