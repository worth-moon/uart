#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include <string.h>
#include <stdlib.h>

#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))

#define VOFA_UART huart1
#define MAX_BUFFER_SIZE 1024

void vofa_start(void);
void vofa_send_data(uint8_t num, float data); 
void vofa_sendframetail(void);
void vofa_demo(void);
void vofa_Receive(uint8_t* Buf,uint16_t *Len);
float vofa_cmd_parse(uint8_t *cmdBuf, char *arg);
#endif /* __VOFA_H__ */














