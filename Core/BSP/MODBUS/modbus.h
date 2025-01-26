#ifndef __MODBUS_H__
#define __MODBUS_H__

#include "main.h"
#include "stdio.h"
#include <string.h> 
#include "usart.h"

void modbus_send(uint8_t* _pBuf, uint8_t _ucLen);
uint16_t modbus_crc16(uint8_t* _pBuf, uint16_t _usLen);
uint8_t rx_crc_again(void);
void modbus_rx_data_handle(void);
void modbus_stop_rx(void);
void modbus_start_rx(void);
void modbus_restart_rx(void);
void modbus_analyze_rx_data(void);

void modbus_init(void);
void modbus_loop(void);
void modbus_uart_rx_callback(void);

#endif 

