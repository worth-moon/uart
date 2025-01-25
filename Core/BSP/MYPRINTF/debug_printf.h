#ifndef __DEBUG_PRINTF_H__
#define __DEBUG_PRINTF_H__

#include "main.h"
//#include "usbd_cdc_if.h"
#include "stdio.h"
#include <string.h> // 包含字符串处理函数声明
#include <stdarg.h> // 包含可变参数函数处理宏声明
#include "usart.h"

//myprintf相关
#define DMA_MODE 0
#define USB_DEBUG 0
#define UART_HANDLER huart1
#define BUFFER_SIZE 100 // 定义缓冲区大小为100字节

void my_printf(const char *fmt, ...); 
#endif 

