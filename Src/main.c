/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ct_exposure.h"
#include "app_uart.h"
#include "comm_string.h"
#include "app_spi.h"
#include "xray.h"
#include "app_fun.h"
#include "protect.h"
#include "stdio.h"
#include "calibrate.h"
#include "debug_mode.h"
#include "delay.h"
#include "updata.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC2_Init();
    MX_ADC3_Init();
    MX_DAC1_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART3_UART_Init();
    MX_SPI1_Init();
    MX_TIM5_Init();
    MX_TIM2_Init();
    /* USER CODE BEGIN 2 */
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    debug_tx3("123\r\n");

    uint8_t rely_state = 0;
    uint32_t rely_time = 0;
    HAL_TIM_Base_Start_IT(&htim2);
    bsp_read_info();

    registerFunc_init();
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer2, ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buffer3, ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
    //PWM输出强制为低
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0); //占空比的设置直接决定了输出电压的高低（经过滤波器后作为类DA使用）
    //关于占空比 需定义 duty_max / duty_min  Vout ≈ Duty * VDD 需测试做类比线性关系
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_buffer2, ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_buffer3, ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM);
    load_from_flash(parm_table);

    ctrl_data.interlock = 1;//整机不用
    cali_data.para_save_flag = 0;

    xray_data.isCheckAvailable = 0;
    xray_on_led(0);

    if (ctrl_data.hv_vol_fault || ctrl_data.hv_curr_fault)
    {
        /*灯丝复位*/
        config_reset_signal(1);
        fault_value_clear();
			  config_reset_signal(0);
    }

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);

    //cali_data.para_save_flag =1;
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
        if (rely_state == 0)
        {
            if ((get_tick_ms() - rely_time) > 2000)
            {
                HAL_GPIO_WritePin(HV_RE_GPIO_Port, HV_RE_PIN, Calc_Gpio_State_P(1));
                rely_state = 1;
            }
        }
				if (rely_state == 2)
				{
				config_HVEn_signal(1,1);
				config_HVEn_signal(0,1);
				}
								if (rely_state == 3)
				{
								config_HVEn_signal(1,0);
				config_HVEn_signal(0,0);
				}
        if ((get_hv_state(0) == HVPS_SM_ID_UPDATE_RUN) && (get_hv_state(1) == HVPS_SM_ID_UPDATE_RUN))
        {
            upgrade_proc();
            continue;
        }

        cmd_parser();
        cmd_parser_string();

        Protect_Check_Slow();      //慢速故障检查

        if (cali_data.para_save_flag == 1)
        {
            HAL_TIM_Base_Stop_IT(&htim2);
            save_to_flash(parm_table);
            HAL_TIM_Base_Start_IT(&htim2);
            cali_data.para_save_flag = 0;
        }
        heartBeat_led();

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 25;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    /* USER CODE BEGIN Callback 0 */
    /* 1,TIM6 80kHz*/
    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM2)
    {
        ctrl_data.hv_vol_fault  = get_tube_vol_fault_pin();
        ctrl_data.hv_curr_fault = get_tube_curr_fault_pin();
       // ctrl_data.interlock     = get_interLock_pin();
        transform_adc_values();

        if (Is_CTMode())
        {
            //整机烧录
            ctrl_data.expo[0]   = get_expo_pin(0);

            ctrl_data.expo[1]   = get_expo_pin(1);
//                  //其余模式
//            ctrl_data.expo[1]   = get_expo_pin(0);

//            ctrl_data.expo[0]   = get_expo_pin(1);

            ct_task();
        }
        else if (Is_CalibrateMode())
        {
            calibrate_task();
        }
        else if (Is_DebugMode())
        {
            debug_task();
        }

        //故障快速检测
        xray_fast_protect();
        xray_system_fault_check();
    }

    /* USER CODE END Callback 1 */
}
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
