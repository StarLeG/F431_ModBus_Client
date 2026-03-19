#ifndef MODBUS_CLIENT_H
#define MODBUS_CLIENT_H


#include <stdint.h>
#include <stdbool.h>
#include "stm32g4xx_hal.h"
#include "crc_mb.h"

// Адреса Modbus устройств
#define MODBUS_BROADCAST_ADDR    0x00
#define MODBUS_MIN_ADDR          0x01
#define MODBUS_MAX_ADDR          0xF7

// Коды функций Modbus
#define MODBUS_FUNC_READ_COILS                0x01 // Чтение дискретных выходов
#define MODBUS_FUNC_READ_DISCRETE_INPUTS      0x02 // Чтение дискретных входов
#define MODBUS_FUNC_READ_HOLDING_REGISTERS    0x03 // Чтениеholding регистров
#define MODBUS_FUNC_READ_INPUT_REGISTERS      0x04 // Чтение input регистров
#define MODBUS_FUNC_WRITE_SINGLE_COIL         0x05 // Запись одного дискретного выхода
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER     0x06 // Запись одного holding регистра
#define MODBUS_FUNC_WRITE_MULTIPLE_COILS      0x0F // Запись нескольких дискретных выходов
#define MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS  0x10 // Запись нескольких holding регистров

// Коды ошибок Modbus
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION      0x01 // Неверный код функции
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDR     0x02 // Неверный адрес данных
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE    0x03 // Неверное значение данных
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE  0x04 // Ошибка устройства-слейва
#define MODBUS_EXCEPTION_ACKNOWLEDGE           0x05 // Подтверждение
#define MODBUS_EXCEPTION_SLAVE_DEVICE_BUSY     0x06 // Устройство-слейв занято
#define MODBUS_EXCEPTION_MEMORY_PARITY_ERROR   0x08 // Ошибка четности памяти
#define MODBUS_EXCEPTION_GATEWAY_PATH_UNAVAIL  0x0A // Путь шлюза недоступен
#define MODBUS_EXCEPTION_GATEWAY_TARGET_FAILED 0x0B // Цель шлюза не отвечает

// Состояния Modbus клиента
typedef enum {
    MODBUS_STATE_IDLE,                  // Ожидание команды
    MODBUS_STATE_SENDING_REQUEST,       // Отправка запроса
    MODBUS_STATE_WAITING_RESPONSE,      // Ожидание ответа
    MODBUS_STATE_RESPONSE_RECEIVED,     // Ответ получен
    MODBUS_STATE_TIMEOUT,               // Время ожидания истекло
    MODBUS_STATE_ERROR                  // Произошла ошибка
} ModbusState;

// Структура для хранения состояния Modbus клиента
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t slave_addr;
    uint8_t function_code;
    uint16_t timeout_ms;
    ModbusState state;
    uint8_t tx_buffer[256];
    uint8_t rx_buffer[256];
    uint16_t tx_length;
    uint16_t rx_length;
    uint32_t last_tick;
    uint8_t retry_count;
    uint8_t max_retries;
    void (*callback)(uint8_t slave_addr, uint8_t function, uint8_t *data, uint16_t len, bool success);
} ModbusClient;

// Инициализация Modbus клиента
void MODBUS_Init(ModbusClient *client, UART_HandleTypeDef *huart, uint8_t slave_addr); 

// Установка времени ожидания ответа
void MODBUS_SetTimeout(ModbusClient *client, uint16_t timeout_ms); 

// Установка количества попыток повторной отправки
void MODBUS_SetRetries(ModbusClient *client, uint8_t max_retries);


void MODBUS_SetCallback(ModbusClient *client, void (*callback)(uint8_t, uint8_t, uint8_t*, uint16_t, bool));

void MODBUS_Process(ModbusClient *client);

// Чтение катушек (Coils) - функция 0x01
bool MODBUS_ReadCoils(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity);

// Чтение дискретных входов - функция 0x02
bool MODBUS_ReadDiscreteInputs(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity);

// Чтение holding регистров - функция 0x03
bool MODBUS_ReadHoldingRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity);

// Чтение input регистров - функция 0x04
bool MODBUS_ReadInputRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity);

// Запись одной катушки - функция 0x05
bool MODBUS_WriteSingleCoil(ModbusClient *client, uint8_t slave_addr, uint16_t coil_addr, bool value);

// Запись одного регистра - функция 0x06
bool MODBUS_WriteSingleRegister(ModbusClient *client, uint8_t slave_addr, uint16_t reg_addr, uint16_t value);

// Запись нескольких катушек - функция 0x0F
bool MODBUS_WriteMultipleCoils(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, 
                               uint16_t quantity, uint8_t *data);

// Запись нескольких регистров - функция 0x10
bool MODBUS_WriteMultipleRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, 
                                   uint16_t quantity, uint16_t *data);

// Обработчик прерывания UART 
void MODBUS_UART_RxCpltCallback(ModbusClient *client);
void MODBUS_UART_TxCpltCallback(ModbusClient *client);
void MODBUS_UART_ErrorCallback(ModbusClient *client);                                   

#endif // MODBUS_CLIENT_H