#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

void handle_request(uint8_t *request, uint16_t length, uint8_t *response); 

#endif // MODBUS_SERVER_H