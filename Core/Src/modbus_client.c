#include "modbus_client.h"
#include "crc_mb.h"
#include <string.h>

// Инициализация Modbus клиента
void MODBUS_Init(ModbusClient *client, UART_HandleTypeDef *huart, uint8_t slave_addr)
{
    client->huart = huart;
    client->slave_addr = slave_addr;
    client->timeout_ms = 1000; // Таймаут по умолчанию 1 секунда
    client->state = MODBUS_STATE_IDLE;
    client->tx_length = 0;
    client->rx_length = 0;
    client->last_tick = 0;
    client->retry_count = 0;
    client->max_retries = 3; // Максимум 3 попытки по умолчанию
    client->callback = NULL;
    
    memset(client->tx_buffer, 0, sizeof(client->tx_buffer));
    memset(client->rx_buffer, 0, sizeof(client->rx_buffer));
}

// Установка таймаута
void MODBUS_SetTimeout(ModbusClient *client, uint16_t timeout_ms)
{
    client->timeout_ms = timeout_ms;
}

// Установка количества повторных попыток
void MODBUS_SetRetries(ModbusClient *client, uint8_t max_retries)
{
    client->max_retries = max_retries;
}

// Установка callback функции
void MODBUS_SetCallback(ModbusClient *client, void (*callback)(uint8_t, uint8_t, uint8_t*, uint16_t, bool))
{
    client->callback = callback;
}

// Отправка запроса
static bool MODBUS_SendRequest(ModbusClient *client)
{
    if (HAL_UART_Transmit_IT(client->huart, client->tx_buffer, client->tx_length) == HAL_OK) {
        client->state = MODBUS_STATE_WAITING_RESPONSE;
        client->last_tick = HAL_GetTick();
        client->rx_length = 0;
        return true;
    }
    
    return false;
}

// Проверка CRC ответа
static bool MODBUS_CheckCRC(ModbusClient *client)
{
    if (client->rx_length < 4) return false;
    
    uint16_t received_crc = client->rx_buffer[client->rx_length - 2] | 
                            (client->rx_buffer[client->rx_length - 1] << 8);
    uint16_t calculated_crc = crc_mb(client->rx_buffer, client->rx_length - 2);
    
    return (received_crc == calculated_crc);
}

// Обработка ответа
static void MODBUS_ProcessResponse(ModbusClient *client)
{      
    bool success = false;
    // Проверка минимальной длины и CRC
    if (client->rx_length >= 4 && MODBUS_CheckCRC(client)) {
        // Проверка на ошибку (если установлен старший бит функции)
        if (client->rx_buffer[1] & 0x80) {
            // Получен код ошибки
            if (client->callback) {
                client->callback(client->rx_buffer[0], client->rx_buffer[1] & 0x7F, 
                                &client->rx_buffer[2], client->rx_length - 4, false);
            }
        } else {
            // Успешный ответ
            success = true;
            if (client->callback) {
                client->callback(client->rx_buffer[0], client->rx_buffer[1], 
                                &client->rx_buffer[2], client->rx_length - 4, true);
            }
        }
    }
    
    client->state = MODBUS_STATE_IDLE;
    client->retry_count = 0;
}

// Основная функция обработки
void MODBUS_Process(ModbusClient *client)
{
    if (client->state == MODBUS_STATE_WAITING_RESPONSE) {
        // Проверка таймаута
        if ((HAL_GetTick() - client->last_tick) >= client->timeout_ms) {
            // Таймаут - пробуем повторить
            if (client->retry_count < client->max_retries) {
                client->retry_count++;
                MODBUS_SendRequest(client);
            } else {
                // Превышено количество попыток
                client->state = MODBUS_STATE_TIMEOUT;
                client->retry_count = 0;
                
                if (client->callback) {
                    client->callback(client->tx_buffer[0], client->tx_buffer[1], NULL, 0, false);
                }
                
                client->state = MODBUS_STATE_IDLE;
            }
        }
    } else if (client->state == MODBUS_STATE_RESPONSE_RECEIVED) {
        MODBUS_ProcessResponse(client);
    }
}

// Чтение катушек (Coils)
bool MODBUS_ReadCoils(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_READ_COILS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Чтение дискретных входов
bool MODBUS_ReadDiscreteInputs(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_READ_DISCRETE_INPUTS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Чтение holding регистров
bool MODBUS_ReadHoldingRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_READ_HOLDING_REGISTERS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Чтение input регистров
bool MODBUS_ReadInputRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, uint16_t quantity)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_READ_INPUT_REGISTERS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Запись одной катушки
bool MODBUS_WriteSingleCoil(ModbusClient *client, uint8_t slave_addr, uint16_t coil_addr, bool value)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_WRITE_SINGLE_COIL;
    client->tx_buffer[2] = (coil_addr >> 8) & 0xFF;
    client->tx_buffer[3] = coil_addr & 0xFF;
    
    // Modbus спецификация: 0xFF00 = ON, 0x0000 = OFF
    if (value) {
        client->tx_buffer[4] = 0xFF;
        client->tx_buffer[5] = 0x00;
    } else {
        client->tx_buffer[4] = 0x00;
        client->tx_buffer[5] = 0x00;
    }
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Запись одного регистра
bool MODBUS_WriteSingleRegister(ModbusClient *client, uint8_t slave_addr, uint16_t reg_addr, uint16_t value)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_WRITE_SINGLE_REGISTER;
    client->tx_buffer[2] = (reg_addr >> 8) & 0xFF;
    client->tx_buffer[3] = reg_addr & 0xFF;
    client->tx_buffer[4] = (value >> 8) & 0xFF;
    client->tx_buffer[5] = value & 0xFF;
    
    uint16_t crc = crc_mb(client->tx_buffer, 6);
    client->tx_buffer[6] = crc & 0xFF;
    client->tx_buffer[7] = (crc >> 8) & 0xFF;
    
    client->tx_length = 8;
    
    return MODBUS_SendRequest(client);
}

// Запись нескольких катушек
bool MODBUS_WriteMultipleCoils(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, 
                               uint16_t quantity, uint8_t *data)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    uint8_t byte_count = (quantity + 7) / 8;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_WRITE_MULTIPLE_COILS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    client->tx_buffer[6] = byte_count;
    
    memcpy(&client->tx_buffer[7], data, byte_count);
    
    uint16_t crc = crc_mb(client->tx_buffer, 7 + byte_count);
    client->tx_buffer[7 + byte_count] = crc & 0xFF;
    client->tx_buffer[8 + byte_count] = (crc >> 8) & 0xFF;
    
    client->tx_length = 9 + byte_count;
    
    return MODBUS_SendRequest(client);
}

// Запись нескольких регистров
bool MODBUS_WriteMultipleRegisters(ModbusClient *client, uint8_t slave_addr, uint16_t start_addr, 
                                   uint16_t quantity, uint16_t *data)
{
    if (client->state != MODBUS_STATE_IDLE) return false;
    
    uint8_t byte_count = quantity * 2;
    
    client->tx_buffer[0] = slave_addr;
    client->tx_buffer[1] = MODBUS_FUNC_WRITE_MULTIPLE_REGISTERS;
    client->tx_buffer[2] = (start_addr >> 8) & 0xFF;
    client->tx_buffer[3] = start_addr & 0xFF;
    client->tx_buffer[4] = (quantity >> 8) & 0xFF;
    client->tx_buffer[5] = quantity & 0xFF;
    client->tx_buffer[6] = byte_count;
    
    for (uint16_t i = 0; i < quantity; i++) {
        client->tx_buffer[7 + i*2] = (data[i] >> 8) & 0xFF;
        client->tx_buffer[8 + i*2] = data[i] & 0xFF;
    }
    
    uint16_t crc = crc_mb(client->tx_buffer, 7 + byte_count);
    client->tx_buffer[7 + byte_count] = crc & 0xFF;
    client->tx_buffer[8 + byte_count] = (crc >> 8) & 0xFF;
    
    client->tx_length = 9 + byte_count;
    
    return MODBUS_SendRequest(client);
}

// Обработчики прерываний UART
void MODBUS_UART_RxCpltCallback(ModbusClient *client)
{
    if (client->state == MODBUS_STATE_WAITING_RESPONSE) {
        client->state = MODBUS_STATE_RESPONSE_RECEIVED;
    }
}

void MODBUS_UART_TxCpltCallback(ModbusClient *client)
{
    // После отправки запроса ожидаем ответ
    HAL_UART_Receive_IT(client->huart, client->rx_buffer, sizeof(client->rx_buffer));
}

void MODBUS_UART_ErrorCallback(ModbusClient *client)
{
    client->state = MODBUS_STATE_ERROR;
    
    if (client->callback) {
        client->callback(client->tx_buffer[0], client->tx_buffer[1], NULL, 0, false);
    }
    
    client->state = MODBUS_STATE_IDLE;
    client->retry_count = 0;
}