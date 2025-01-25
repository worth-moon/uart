#ifndef __DEBUG_PRINTF_H__
#define __DEBUG_PRINTF_H__

#include "main.h"
//#include "usbd_cdc_if.h"
#include "stdio.h"
#include <string.h> // �����ַ�������������
#include <stdarg.h> // �����ɱ�����������������
#include "usart.h"

//myprintf���
#define DMA_MODE 0
#define USB_DEBUG 0
#define UART_HANDLER huart1
#define BUFFER_SIZE 100 // ���建������СΪ100�ֽ�

void my_printf(const char *fmt, ...); 
#endif 

