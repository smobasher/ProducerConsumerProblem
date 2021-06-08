/* USER CODE BEGIN Header */
/**
  ******************************************************************************
    Salma Mobasher
    8120214
    March 15 2021
  *
  ******************************************************************************
  */




#include "../inc/main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <stdlib.h>
#include <task.h>
#include <queue.h>
#include  <semphr.h>
#include <FreeRTOS.h> //not sure about this
int x=1;
int y = 1;
#define     Np x //producers can randomize value
#define     Mc y  //consumers
# define    items 20 //buffer size
# define    STACK_SIZE         ( configMINIMAL_STACK_SIZE+512)
struct task_args_s{
    int n;
};

osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);

// for SWV printf
int write(int file, char*ptr, int len) //this is to be able to print the things
{
	int i=0;
    for(i=0 ; i<len; i++)
	ITM_SendChar((*ptr++));
    return len;
}

int buf[items];
int in = 0, out = 0;
xSemaphoreHandle  empty,  full;
xSemaphoreHandle  mutex ;
void  producer_code( void * argv)
{
	struct  task_args_s * args = ( struct task_args_s *) argv ;
    int c = 0;
    float t =(rand() % 100 + 1)/100; //generate value from 0.01 to 1 second
    uint16_t item ; //this is the format asked for
    while (1)
    {
      vTaskDelay(t);
      item = args ->n *1000 + c;
      c ++;
      printf (" Producer #%d is running producing item %6d\n",    args ->n, item);

      /*for message passing:
       *  if( xQueueSendToBack(args ->q, &item,  portMAX_DELAY)   != pdPASS)
      printf ("*   Producer %d unable to send\n",      args ->n);
      */

      if( xSemaphoreTake(empty,  portMAX_DELAY) !=  pdTRUE )
             printf ("*   Producer %d unable to take ’empty’\n", args ->n);

            else if( xSemaphoreTake( mutex,  portMAX_DELAY) !=  pdTRUE )
             printf ("*   Producer %d unable to take ’mutex’\n", args ->n);
            else
            {
             buf[ in] = item;
             in = (in + 1) % items;
             osDelay(1);//1 second delay
             /*  Release mutex  */
             if( xSemaphoreGive( mutex ) != pdTRUE )
               printf ("*   Producer %d unable to give ’mutex’\n", args ->n);
             /*  Synchronize with consumers  */
             if( xSemaphoreGive( full) != pdTRUE)
               printf ("* Producer %d unable to give ’full’\n", args ->n);
            }
         }
}
uint16_t  consumer_code( void * argv)
{
	struct  task_args_s * args = (struct task_args_s *) argv ;
	float t =(rand() % 100 + 1)/100; //generate value from 0.01 to 1 second
	uint16_t item ;
	    while (1)
	    {
	       /*if message passing:
	        if( xQueueReceive(args ->q,  &item, portMAX_DELAY)   !=  pdPASS )
         printf ("* Consumer #%d unable to      receive \n",  args ->n);
         */

	    	printf (" Consumer #%d is running, consuming item %6d\n", args ->n, item );
	      /*  Synchronize with producers  */
	       if( xSemaphoreTake(full,  portMAX_DELAY) !=  pdTRUE)
	        printf ("*   Consumer %d unable to take ’full’\n", args ->n);
	      /*  Mutual exclusion for buffer access  */
	       else if( xSemaphoreTake( mutex,  portMAX_DELAY) !=  pdTRUE)
	        printf ("*   Consumer %d unable to take ’mutex’\n", args ->n);
	       else
	       {

	        item = buf[out ];
	        out = (out + 1) % items;
	        osDelay(1); //1 sec delay

	        if( xSemaphoreGive( mutex ) != pdTRUE )
	          printf ("*   Consumer %d unable to give ’mutex’\n", args ->n);

	        if( xSemaphoreGive( empty ) != pdTRUE )
	          printf ("*   Consumer %d unable to give ’full’\n", args ->n);

	        printf (" Consumer #%d is running, consuming item %6d\n", args ->n, item );
	        vTaskDelay(t);
	       }


	    }
	    return out;
}
int main(int  argc,  char * argv [])
{

 HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();

  osKernelInitialize();
// xQueueHandle q; for message passing
  x= rand() % 10 + 1; //randomizes producers from 1 to 10
  y= rand() % 10 + 1; //randomizes consumers from 1 to 10
     struct  task_args_s prod_args[Np];    /*  Task arguments for producers  */
     struct task_args_s cons_args[Mc];    /*  Task arguments for consumers  */
     xTaskHandle  dummy ;
     int i;
     if (( empty =  xSemaphoreCreateCounting(items, items)) == NULL
       || ( full =  xSemaphoreCreateCounting(items, 0)) ==  NULL )
        printf ("* Cannot create counting  semaphores\n" );
    /* Create the mutual exclusion semaphore  */
     else if (( mutex =  xSemaphoreCreateMutex()) ==  NULL )
        printf ("Cannot create mutex \n");
     else
     {
       /*  Create Np producer tasks  */
        for(i =0; i<Np;   i ++)
        {
       prod_args[i].n = i;  /*  Prepare the argument  */
       /*  The task handles are not used in the following ,
           so a dummy variable is used for them
       */


       /*for message passing:
        * if ((q =      xQueueCreate(N,  sizeof (int ))) == NULL)
     printf ("* Cannot create message queue of %d elements \n", N);

     /*  Create NP producer tasks  */
     for(i =0; i<NP;     i ++)
     {
       prod_args[i].n = i;  /*  Prepare the arguments  */
 //      prod_args[i].q = q;*/
       if( xTaskCreate( producer_code, " PROD", STACK_SIZE ,
                       &( prod_args[i]),  tskIDLE_PRIORITY, &dummy ) != pdPASS )
         printf ("* Cannot create producer #%d\n", i );
        }
       /*  Create Mc consumer tasks  */
        for(i =0; i<Mc; i ++)
        {
       cons_args[i].n = i;
       if( xTaskCreate( consumer_code, " CONS", STACK_SIZE ,
                       &( cons_args[i]),  tskIDLE_PRIORITY, &dummy ) != pdPASS )
         printf ("* Cannot create consumer #%d\n", i );
        }
        vTaskStartScheduler();
        printf ("*  vTaskStartScheduler()  failed \n");
     }

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  osKernelStart();

  return EXIT_SUCCESS;
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE5 MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CLK_IN_Pin PB12 */
  GPIO_InitStruct.Pin = CLK_IN_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}


void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
