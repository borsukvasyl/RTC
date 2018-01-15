/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "lcd5110.h"

#define RTC_ADDRESS                 0x68*2
#define NUMBER_OF_DAYS_IN_WEEK      7

#define LCD_CS_Pin                  GPIO_PIN_14
#define LCD_CS_GPIO_Port            GPIOB
#define LCD_RST_Pin                 GPIO_PIN_11
#define LCD_RST_GPIO_Port           GPIOB
#define LCD_DC_Pin                  GPIO_PIN_12
#define LCD_DC_GPIO_Port            GPIOB

#define KEYPAD_PIN_COL0             GPIO_PIN_7
#define KEYPAD_PIN_COL1             GPIO_PIN_5
#define KEYPAD_PIN_COL2             GPIO_PIN_1
#define KEYPAD_PIN_COL3             GPIO_PIN_7
#define KEYPAD_GPIO_COL0            GPIOA
#define KEYPAD_GPIO_COL1            GPIOC
#define KEYPAD_GPIO_COL2            GPIOB
#define KEYPAD_GPIO_COL3            GPIOE

#define KEYPAD_PIN_ROW0             GPIO_PIN_1
#define KEYPAD_PIN_ROW1             GPIO_PIN_3
#define KEYPAD_PIN_ROW2             GPIO_PIN_4
#define KEYPAD_PIN_ROW3             GPIO_PIN_5
#define KEYPAD_GPIO_ROW0            GPIOA
#define KEYPAD_GPIO_ROW1            GPIOA
#define KEYPAD_GPIO_ROW2            GPIOF
#define KEYPAD_GPIO_ROW3            GPIOA

//extern void initialise_monitor_handles(void);

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

typedef struct {
    uint8_t seconds; // Seconds parameter, from 00 to 59
    uint8_t minutes; // Minutes parameter, from 00 to 59
    uint8_t hours;   // Hours parameter, 24Hour mode, 00 to 23
    uint8_t weekday; // Day in a week, from 1 to 7
    uint8_t date;    // Date in a month, 1 to 31
    uint8_t month;   // Month in a year, 1 to 12
    uint8_t year;    // Year parameter
} DS3231_Time;

inline int bcd_to_decimal(uint8_t bcd_value) {
    return ((bcd_value & 0xF0) >> 4) * 10 + (bcd_value & 0x0F);
}

inline int decimal_to_bcd(uint8_t decimal) {
    return ((decimal / 10) << 4) + decimal % 10;
}

int RTC_write_data(DS3231_Time* time) {
    uint8_t send_data[7];
    send_data[0] = decimal_to_bcd(time->seconds);
    send_data[1] = decimal_to_bcd(time->minutes);
    send_data[2] = decimal_to_bcd(time->hours);
    send_data[3] = decimal_to_bcd(time->weekday);
    send_data[4] = decimal_to_bcd(time->date);
    send_data[5] = decimal_to_bcd(time->month);
    send_data[6] = decimal_to_bcd(time->year);
    HAL_I2C_Mem_Write(&hi2c1, RTC_ADDRESS, 0, 1, send_data, 7, 500);
    return 0;
}

int RTC_read_data(DS3231_Time* time) {
    uint8_t received_data[7];
    HAL_I2C_Mem_Read(&hi2c1, RTC_ADDRESS, 0, 1, received_data, 7, 500);
    time->seconds = bcd_to_decimal(received_data[0]);
    time->minutes = bcd_to_decimal(received_data[1]);
    time->hours = bcd_to_decimal(received_data[2]);
    time->weekday = bcd_to_decimal(received_data[3]);
    time->date = bcd_to_decimal(received_data[4]);
    time->month = bcd_to_decimal(received_data[5]);
    time->year = bcd_to_decimal(received_data[6]);
    return 0;
}

LCD5110_display lcd1;

const char *weekday_names[NUMBER_OF_DAYS_IN_WEEK] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

inline char* convert_weekday(int weekday_number){ // weekday in range [1, 7]
    return weekday_names[weekday_number - 1];
}

void LCD5110_display_time(DS3231_Time* time){
    LCD5110_clear_scr(&lcd1.hw_conf);
    LCD5110_set_cursor(0,0, &lcd1.hw_conf);
    LCD5110_printf(&lcd1, BLACK, "   %02d:%02d:%02d\n %s\n %02d.%02d.20%d\n",
            time->hours, time->minutes, time->seconds, convert_weekday(time->weekday), time->date, time->month, time->year);
}

void LCD5110_display_timearray(int* timearray, int current_symbol) {
    char to_display[13];
    for (int i = 0; i < 13; i++) {
        to_display[i] = (i == current_symbol)?'_':timearray[i] + '0';
	}

    LCD5110_clear_scr(&lcd1.hw_conf);
    LCD5110_set_cursor(0,0, &lcd1.hw_conf);
    LCD5110_printf(&lcd1, BLACK, "   %c%c:%c%c:%c%c\n %c\n %c%c.%c%c.20%c%c\n",
            to_display[0], to_display[1], to_display[2], to_display[3], to_display[4], to_display[5],
            to_display[6],
            to_display[7], to_display[8], to_display[9], to_display[10], to_display[11], to_display[12]);
}

GPIO_TypeDef* column_gpios[] = {KEYPAD_GPIO_COL0, KEYPAD_GPIO_COL1, KEYPAD_GPIO_COL2};
uint16_t column_pins[] = {KEYPAD_PIN_COL0, KEYPAD_PIN_COL1, KEYPAD_PIN_COL2};
GPIO_TypeDef* row_gpios[] = {KEYPAD_GPIO_ROW0, KEYPAD_GPIO_ROW1, KEYPAD_GPIO_ROW2, KEYPAD_GPIO_ROW3};
uint16_t row_pins[] = {KEYPAD_PIN_ROW0, KEYPAD_PIN_ROW1, KEYPAD_PIN_ROW2, KEYPAD_PIN_ROW3};


#define TO_PREVIOUS_SYMBOL               10
#define TO_NEXT_SYMBOL                   11
int symbs[3][4] = {{1, 4, 7, TO_PREVIOUS_SYMBOL}, {2, 5, 8, 0}, {3, 6, 9, TO_NEXT_SYMBOL}};

int read_button() {
    for (int col_ind = 0; col_ind < 3; col_ind++) {
        HAL_GPIO_WritePin(column_gpios[col_ind], column_pins[col_ind], GPIO_PIN_SET);
        for (int row_ind = 0; row_ind < 4; row_ind++) {
            if (HAL_GPIO_ReadPin(row_gpios[row_ind], row_pins[row_ind])) {
                HAL_GPIO_WritePin(column_gpios[col_ind], column_pins[col_ind], GPIO_PIN_RESET);
                return symbs[col_ind][row_ind];
            }
        }
        HAL_GPIO_WritePin(column_gpios[col_ind], column_pins[col_ind], GPIO_PIN_RESET);
    }
    return -1;
}

int read_user_data(DS3231_Time* time) {
    //                  h     m     s   day   d     m     y
    int my_time[13] = {0,0,  0,0,  0,0,  0,  0,0,  0,0,  0,0};
    int value;

    LCD5110_display_timearray(&my_time, 0);

    int my_time_ind = 0;
    while (my_time_ind < 13) {
        while (1) {
            value = read_button();
            if (value != -1) {
            	if (value < 10) {
                    my_time[my_time_ind] = value;
                    my_time_ind++;
                }
                else {
                    switch(value) {
                    case TO_PREVIOUS_SYMBOL:
                        my_time_ind--;
                        break;
                    case TO_NEXT_SYMBOL:
                        my_time_ind++;
                        break;
                    }
                }
                LCD5110_display_timearray(&my_time, my_time_ind);
                HAL_Delay(500);
                break;
            }
        }
	}
    time->seconds = my_time[4] * 10 + my_time[5];
    time->minutes = my_time[2] * 10 + my_time[3];
    time->hours = my_time[0] * 10 + my_time[1];
    time->weekday = my_time[6];
    time->date = my_time[7] * 10 + my_time[8];
    time->month = my_time[9] * 10 + my_time[10];
    time->year = my_time[11] * 10 + my_time[12];
    return 0;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
    //initialise_monitor_handles();

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2C1_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

    lcd1.hw_conf.spi_handle = &hspi2;
    lcd1.hw_conf.spi_cs_pin = LCD_CS_Pin;
    lcd1.hw_conf.spi_cs_port = LCD_CS_GPIO_Port;
    lcd1.hw_conf.rst_pin =  LCD_RST_Pin;
    lcd1.hw_conf.rst_port = LCD_RST_GPIO_Port;
    lcd1.hw_conf.dc_pin =  LCD_DC_Pin;
    lcd1.hw_conf.dc_port = LCD_DC_GPIO_Port;
    lcd1.def_scr = lcd5110_def_scr;
    LCD5110_init(&lcd1.hw_conf, LCD5110_NORMAL_MODE, 0x40, 2, 3);

    DS3231_Time time;
    read_user_data(&time);
    RTC_write_data(&time);

    HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1){
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    	if (HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW3, KEYPAD_PIN_ROW3)) {
    		HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
    		read_user_data(&time);
    		RTC_write_data(&time);
    		HAL_GPIO_WritePin(KEYPAD_GPIO_COL3, KEYPAD_PIN_COL3, GPIO_PIN_SET);
    	}

        RTC_read_data(&time);

        LCD5110_display_time(&time);
        HAL_Delay(33);
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1){}
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
