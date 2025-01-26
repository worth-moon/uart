#ifndef __OPENMV_H__
#define __OPENMV_H__

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t openmv_data_process_flag(uint8_t* rx_data, uint8_t len, uint8_t target);
uint8_t openmv_data_process_float(uint8_t* rx_data, uint8_t len, uint8_t target_len, float* target_data);


#endif 


