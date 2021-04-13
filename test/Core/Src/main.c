/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uprintf.h"
#include "stm32l0538_discovery_epd.h"
#include "picture.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Rx_Data U1_RD;
uint8_t U1DMA_RXBUF[200];
uint16_t Value_ADC[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Enable_USART1_IT(void)
{
    LL_USART_ClearFlag_IDLE(USART1);          //清除空闲标志
    LL_USART_EnableIT_RXNE(USART1); 					//使能串口1接收中断
    LL_USART_EnableIT_IDLE(USART1); 					//使能串口1空闲中断
}

void Start_U1DMA(void)
{
    LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_3,(uint32_t)&USART1->RDR,(uint32_t)&U1DMA_RXBUF,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);   //设置源地址，目的地址，传输方式
    LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_3,200);	                                                                               //设置传输字节数
    LL_USART_EnableDMAReq_RX(USART1);                                                                                                //使能串口1接收DMA
    LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_3);                                                                                     //启动DMA传输通道
}

void Start_ADC1DMA(void)
{
    LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_1,(uint32_t)&ADC1->DR,(uint32_t)&Value_ADC,LL_DMA_DIRECTION_PERIPH_TO_MEMORY);   //设置源地址，目的地址，传输方式
    LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,2);	                                                                              //设置传输字节数
    LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);                                                                                  //启动DMA传输通道
}


void Activate_ADC(void)
{
    __IO uint32_t wait_loop_index = 0;
    __IO uint32_t backup_setting_adc_dma_transfer = 0;

    if (LL_ADC_IsEnabled(ADC1) == 0)
    {
        /* 校准期间必须关闭DMA */
        backup_setting_adc_dma_transfer = LL_ADC_REG_GetDMATransfer(ADC1);
        LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

        /* 开始自校准 */
        LL_ADC_StartCalibration(ADC1);

        /*轮询等待自校准完成*/
        while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0)
        {
        }

        /* 自校准结束延时2个ADC时钟周期才可以开始使能ADC，本例APB2时钟=ADC时钟=16M*/
        wait_loop_index = (2>> 1);
        while(wait_loop_index != 0)
        {
            wait_loop_index--;
        }

        /* 校准完成后恢复DMA */
        LL_ADC_REG_SetDMATransfer(ADC1, backup_setting_adc_dma_transfer);

        /* 使能 ADC */
        LL_ADC_Enable(ADC1);

        /* 轮询ADC准备状态 */

        while (LL_ADC_IsActiveFlag_ADRDY(ADC1) == 0)
        {
        }

        /* Note: ADC flag ADRDY is not cleared here to be able to check ADC       */
        /*       status afterwards.                                               */
        /*       This flag should be cleared at ADC Deactivation, before a new    */
        /*       ADC activation, using function "LL_ADC_ClearFlag_ADRDY()".       */
    }

}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */
    LL_RCC_ClocksTypeDef sysfeq;
    char Value_Str[10]= {0};
    static uint16_t Avg_Volt;
    static uint8_t ADC_display=1;
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

    /* System interrupt init*/

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI1_Init();
    MX_USART1_UART_Init();
    MX_ADC_Init();
    /* USER CODE BEGIN 2 */

    /* 使能SPI1 */
    LL_SPI_Enable(SPI1);

    /* 获取系统时钟 */
    LL_RCC_GetSystemClocksFreq(&sysfeq);

    /* 打印开机信息*/
    my_printf(U1,"stm32l0538-disco开发板测试程序\n作者：王亮\n");
    my_printf(U1,"stm32l0538-disco开发板硬件资源说明\n");
    my_printf(U1,"按键：B1--PA0\n");
    my_printf(U1,"LED绿：LD3--PB4\n");
    my_printf(U1,"LED红：LD4--PA5\n");
    my_printf(U1,"调试打印串口：deBug(USART1)--PA9（TX） PA10（RX）\n");
    my_printf(U1,"墨水屏：ePD1(SPI1)--PA8（BUSY） PB2（RESET） PB11(D/C) PA15(CS) PB3(SCK) PB5(MOSI) PB10(PWREN)\n");
    my_printf(U1,"当前系统时钟：%dHz\n\n",sysfeq.SYSCLK_Frequency);
    my_printf(U1,"请将要ePD显示的ASIIC字符串通过串口发送给我..\n");

    /*启动DMA*/
    Start_ADC1DMA();

    /*激活ADC*/
    Activate_ADC();

    /* 使能串口接收空闲中断+DMA*/
    Enable_USART1_IT();
    Start_U1DMA();

    /* 初始化墨水屏*/
    BSP_EPD_Init();
    BSP_EPD_DrawImage(0,14,16,16,(uint8_t*) picture_1);
    BSP_EPD_DrawImage(0,0,16,16,(uint8_t*) picture_1);
    BSP_EPD_DrawImage(154,0,16,16,(uint8_t*) picture_1);
    BSP_EPD_DrawImage(154,14,16,16,(uint8_t*) picture_1);
    BSP_EPD_DrawHLine(0,12,172);
    BSP_EPD_DisplayStringAt(0,7,(uint8_t*)"** Welcom, Alice! **",CENTER_MODE);
    BSP_EPD_DrawHLine(0,6,172);
    BSP_EPD_RefreshDisplay();
    LL_mDelay(5000);

    /*设置LED初始状态*/
    LL_GPIO_ResetOutputPin(LD_R_GPIO_Port,LD_R_Pin);
    LL_GPIO_SetOutputPin(LD_G_GPIO_Port,LD_G_Pin);


    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        //延时5秒
        LL_mDelay(5000);

        //ADC1开始转换
        if ((LL_ADC_IsEnabled(ADC1) == 1) &&
                (LL_ADC_IsDisableOngoing(ADC1) == 0)&&
                (LL_ADC_REG_IsConversionOngoing(ADC1) == 0))
        {
            LL_ADC_REG_StartConversion(ADC1);
        }

        //翻转红绿LED
        LL_GPIO_TogglePin(LD_R_GPIO_Port,LD_R_Pin);
        LL_GPIO_TogglePin(LD_G_GPIO_Port,LD_G_Pin);

        //计算ADC平均电压，单位mv
        Avg_Volt=__LL_ADC_CALC_DATA_TO_VOLTAGE(3300,(Value_ADC[0]+Value_ADC[1])/2,LL_ADC_RESOLUTION_12B);



        //如果串口接收指令传输完成则打印收到的串口数据，同时显示到墨水屏上
        if(U1_RD.TC_Flag==1)
        {
            ADC_display=0;
            U1_RD.TC_Flag=0;
            my_printf(U1,"接收到串口数据共%d字节\n",U1_RD.Data_Lenth);
            my_printf(U1,"接收内容如下：%s\n",U1_RD.BUF_DMA);

            //墨水屏清屏，显示接收数据
            BSP_EPD_Clear(EPD_COLOR_BLACK);
            BSP_EPD_Clear(EPD_COLOR_WHITE);
            sprintf(Value_Str,"ADC:%dmv",Avg_Volt);
            BSP_EPD_DisplayStringAtLine(4,(uint8_t*)&Value_Str);
            BSP_EPD_DisplayStringAtLine(3,(uint8_t*)"My Recive Data:");
            BSP_EPD_DrawHLine(0,8,172);
            BSP_EPD_DrawHLine(0,7,172);
            BSP_EPD_DisplayStringAtLine(0,(uint8_t*)&U1_RD.BUF_DMA);
            BSP_EPD_RefreshDisplay();
            //延时0.5秒
            LL_mDelay(5000);
            ADC_display=1;
        }

        //如果串口接收无数据，则只显示ADC转换电压
        if(ADC_display) {
            BSP_EPD_Clear(EPD_COLOR_BLACK);
            BSP_EPD_Clear(EPD_COLOR_WHITE);
            sprintf(Value_Str,"ADC:%dmv",Avg_Volt);
            BSP_EPD_DisplayStringAtLine(4,(uint8_t*)&Value_Str);
            BSP_EPD_RefreshDisplay();
        }
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
    {
    }
    LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE1);
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while(LL_RCC_HSI_IsReady() != 1)
    {

    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_4, LL_RCC_PLL_DIV_4);
    LL_RCC_PLL_Enable();

    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1)
    {

    }
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {

    }

    LL_Init1msTick(16000000);

    LL_SetSystemCoreClock(16000000);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
