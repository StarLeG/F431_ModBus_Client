#include "modbus_server.h"
#include "crc_mb.h"


void handle_request(uint8_t *request, uint16_t length, uint8_t *response) 
{
    
    if(request[0] != 1) // Check if the request is for our device (address 1)
    {        
        return;
    }

    if(request[1] == 0x10) // Check if the function code is 0x10 (Write Multiple Registers) 
    {
        return;
    }

    uint16_t reg_address = (request[2] << 8) | request[3]; // Get the starting register address from the request
    uint16_t reg_length = (request[4] << 8) | request[5]; // Get the number of registers to write from the request

    for(uint16_t i = 0; i < reg_length; i++)
    {
        uint16_t reg_value = (request[7 + i*2] << 8) | request[8 + i*2];     // Get the value to write to the register from the request   
    }

    response[0] = request[0]; // Echo the device address back in the response
    response[1] = request[1]; // Echo the function code back in the response
    response[2] = request[2]; // Echo the starting register address back in the response
    response[3] = request[3]; // Echo the starting register address back in the response
    response[4] = request[4]; // Echo the number of registers to write back in the response
    response[5] = request[5]; // Echo the number of registers to write back in the response
    uint16_t crc = crc_mb(response, 6); // Calculate the CRC for the response (first 6 bytes)
    response[6] = crc & 0xFF; // Append the CRC low byte to the response
    response[7] = (crc >> 8) & 0xFF; // Append the CRC high byte to the response

}

