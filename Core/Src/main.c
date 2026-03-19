/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modbus_client.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Буферы для хранения полученных данных
#define MAX_HOLDING_REGISTERS    100
#define MAX_INPUT_REGISTERS      100
#define MAX_COILS                200
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
ModbusClient modbus_client;

// Структуры для хранения данных
typedef struct {
    uint16_t address;
    uint16_t values[MAX_HOLDING_REGISTERS];
    uint16_t count;
    uint32_t timestamp;
} HoldingRegisters_t;

typedef struct {
    uint16_t address;
    uint16_t values[MAX_INPUT_REGISTERS];
    uint16_t count;
    uint32_t timestamp;
} InputRegisters_t;

typedef struct {
    uint16_t address;
    uint8_t coils[MAX_COILS / 8];
    uint16_t count;
    uint32_t timestamp;
} Coils_t;

// Глобальные переменные для хранения данных
static HoldingRegisters_t holding_regs = {0};
static InputRegisters_t input_regs = {0};
static Coils_t coils = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ModbusCallback(uint8_t slave_addr, uint8_t function, uint8_t *data, uint16_t len, bool success)
{
  if (!success)
  {
    // Обработка ошибки
    if(len > 0)
    {
      uint8_t exception_code = data[0]; // Код исключения находится в первом байте данных

      switch (exception_code) {
        case MODBUS_EXCEPTION_ILLEGAL_FUNCTION:
          // Неверный код функции
          break;
        case MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR:
          // Неверный адрес данных
          break;
        case MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE:
          // Неверное значение данных
          break;
        case MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE:
          // Ошибка устройства-слейва
          break;
        case MODBUS_EXCEPTION_ACKNOWLEDGE:
          // Подтверждение
          break;
        case MODBUS_EXCEPTION_SLAVE_DEVICE_BUSY:
          // Устройство-слейв занято
          break;
        case MODBUS_EXCEPTION_MEMORY_PARITY_ERROR:
          // Ошибка четности памяти
          break;
        case MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAIL:
          // Путь шлюза недоступен
          break;
        case MODBUS_EXCEPTION_GATEWAY_TARGET_FAILED:
          // Цель шлюза не отвечает
          break;
        default:
          // Неизвестная ошибка
          break;
      
      }
    }
    
  }
  else
  {
    // Обработка успешного ответа   

    switch (function) {
      case MODBUS_FUNC_READ_COILS:
        // Обработка данных катушек
        break;
      case MODBUS_FUNC_READ_DISCRETE_INPUTS:
        // Обработка данных дискретных входов
        break;
      case MODBUS_FUNC_READ_HOLDING_REGISTERS:
        // Обработка данных holding регистров
        uint8_t byte_count = data[0]; // Количество байт данных
        uint16_t reg_count = byte_count / 2; // Количество регистров (для функций чтения регистров)

        // Сохраняем в глобальную структуру
        holding_regs.address = slave_addr; // Адрес устройства-слейва
        holding_regs.count = reg_count; // Количество регистров
        holding_regs.timestamp = HAL_GetTick(); // Время получения данных

        for (uint16_t i = 0; i < reg_count; i++) {
            holding_regs.values[i] = (data[1 + i*2] << 8) | data[2 + i*2]; // Сохранение регистра
        }
        break;
      case MODBUS_FUNC_READ_INPUT_REGISTERS:
        // Обработка данных input регистров
        break;
      case MODBUS_FUNC_WRITE_SINGLE_COIL:
        // Подтверждение записи катушки
        break;
      case MODBUS_FUNC_WRITE_SINGLE_REGISTER:
        // Подтверждение записи регистра
        break;
      case MODBUS_FUNC_WRITE_MULTIPLE_COILS:
        // Подтверждение записи нескольких катушек
        break;
      case MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS:
        // Подтверждение записи нескольких регистров
        break;
      default:
        // Неизвестная функция
        break;
    }
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  // Инициализация Modbus клиента  
  MODBUS_Init(&modbus_client, &huart1, 0x01);
  MODBUS_SetTimeout(&modbus_client, 1000);
  MODBUS_SetRetries(&modbus_client, 3); 
  MODBUS_SetCallback(&modbus_client, ModbusCallback);

  // Проверка, не устарели ли данные (например, старше 10 секунд)
  bool MODBUS_IsDataFresh(uint32_t timestamp, uint32_t max_age_ms)
  {
    return (HAL_GetTick() - timestamp) < max_age_ms;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    MODBUS_Process(&modbus_client);
    static uint32_t last_read = 0; // Время последнего чтения
    // Каждые 2 секунды читаем данные
    if ((HAL_GetTick() - last_read) > 2000)
    {
      MODBUS_ReadHoldingRegisters(&modbus_client, 0x01, 0x0001, 1);
      last_read = HAL_GetTick();
    }

    // Проверка свежести данных
    if (MODBUS_IsDataFresh(holding_regs.timestamp, 10000)) {
        // Данные свежие, можно использовать
        int value = holding_regs.values[0]; // Получаем значение первого регистра
        
    } else {
        // Данные устарели, нужно обновить
        MODBUS_ReadHoldingRegisters(&modbus_client, 0x01, 0x0000, 1);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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
// Обработчики прерываний UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        MODBUS_UART_RxCpltCallback(&modbus_client);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        MODBUS_UART_TxCpltCallback(&modbus_client);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        MODBUS_UART_ErrorCallback(&modbus_client);
    }
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
#ifdef USE_FULL_ASSERT
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
