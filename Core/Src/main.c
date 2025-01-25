/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"

#include "debug_printf.h"
#include "vofa.h"
#include "openmv.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t address;
    uint16_t data;
} modbus_reg;

//typedef enum {
//	MOTOR_VEL_TAR = 0,  // 目标速度
//	MOTOR_VEL_CUR = 1,  // 当前速度
//	MOTOR_VEL_ERR = 2   // 速度误差
//} MOTOR_VEL_REG;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define S_TX_BUF_SIZE 128
#define S_RX_BUF_SIZE 128

#define MOTOR_VEL_TAR   0// 目标速度
#define MOTOR_VEL_CUR   1// 当前速度
#define MOTOR_VEL_ERR   2// 速度误差
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float theta, theta_add,theta_max;

//测试协议
uint8_t uart_count = 0;
uint8_t test_flag = 0;
uint8_t rx_buffer[100];
float tar_buffer[10];
float X_IN,Y_IN;

//测试外设
uint8_t test_single_char;
uint8_t test_more_char[22] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20};
uint8_t test_str[] = "helloworld!\r\n";
uint8_t test_rx_buffer[255];
uint8_t rx_count;
/*=========== 以下为modbus相关变量 ===========*/
uint8_t modbus_slave_addr = 0x01; //从机地址,1~247才是有效的
const uint8_t modbus_function_code_read = 0x03; //读功能码
const uint8_t modbus_function_code_write = 0x06; //写功能码
//上述为地址，功能码，接下来是寄存器，包含地址和数据两个部分
//MOTOR_VEL_REG MOTOR_VEL_1;
modbus_reg motor_vel[3];
//modbus_reg motor_vel_tar = {0x0001,0x03e8};//单个变量设置，回头用for轮询的时候不好办；用数组的话会失去可读性，最好的办法就是结构体数组+枚举变量指示用途
//modbus_reg motor_vel_cur = {0x0002,0x0000};
//modbus_reg motor_vel_err = {0x0003,0x0000};
//以下是接收和处理相关的变量
uint8_t modbus_rx_buffer[S_RX_BUF_SIZE]; //接收缓冲区
uint8_t modbus_rx_count = 0; //接收计数器

uint8_t modbus_tx_buffer[S_TX_BUF_SIZE]; //发送缓冲区

uint64_t modbus_timeout_count_last, modbus_timeout_count_now; //超时计数器

uint16_t crc_rx_rec;
uint16_t crc_rx_computer;
uint8_t modbus_rx_len;
uint8_t modbus_only_handle_once;
static const uint8_t s_CRCHi[] = {
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
	0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
	0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};
// CRC 低位字节值表
const uint8_t s_CRCLo[] = {
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06,
	0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
	0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
	0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A,
	0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4,
	0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
	0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3,
	0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
	0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
	0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29,
	0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED,
	0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
	0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60,
	0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
	0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
	0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
	0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E,
	0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
	0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71,
	0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
	0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
	0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B,
	0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B,
	0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
	0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42,
	0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void modbus_send(uint8_t* _pBuf, uint8_t _ucLen);
uint16_t modbus_crc16(uint8_t* _pBuf, uint16_t _usLen);
uint8_t rx_crc_again(void);
void modbus_rx_data_handle(void);
void modbus_stop_rx(void);
void modbus_start_rx(void);
void modbus_restart_rx(void);
void modbus_analyze_rx_data(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
  //
/*
仿modbus写一套程序，回头主从机接上modbus模块，可以直接用
* 先写从机部分
*/
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
  motor_vel[MOTOR_VEL_TAR].address = 0x0000;
  motor_vel[MOTOR_VEL_TAR].data = 0x03e8;
  motor_vel[MOTOR_VEL_CUR].address = 0x0001;
  motor_vel[MOTOR_VEL_CUR].data = 0x1234;
  motor_vel[MOTOR_VEL_ERR].address = 0x0002;
  motor_vel[MOTOR_VEL_ERR].data = 0x5678;

  //modbus_start_rx();
  //HAL_UART_Receive_IT(&huart1, rx_buffer + uart_count, 1);
  HAL_UART_Receive_IT(&huart1, rx_buffer, 50);
  
  //assert_param(0);
//  uint8_t test_tx_data_crc[] = {01,06,00,01,00,10};
//  modbus_send(test_tx_data_crc, 6);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if ((modbus_timeout_count_now = HAL_GetTick()) - modbus_timeout_count_last > 4 && modbus_timeout_count_last != 0)//超时检测
	  {
		  //HAL_UART_Transmit(&huart1, "超时", strlen("超时"), 1000);
		  if (modbus_only_handle_once == 0)//一次指令只响应一次，回复后需要下次接收才能响应
		  {
			  //HAL_UART_Transmit(&huart1, "单次运行", strlen("单次运行"), 1000);
			  if (modbus_rx_count > 4)
			  {

				  modbus_analyze_rx_data();
			  }
			  else
			  {
				  //HAL_UART_Transmit(&huart1, "接收个数不够", strlen("接收个数不够"), 1000);
				  modbus_restart_rx();
			  }
		  }
	  }

	  
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void modbus_analyze_rx_data(void)
{
	modbus_stop_rx();
	if (rx_crc_again() == 1)//CRC能过，才有后面的事
	{
		modbus_rx_data_handle();
	}
	else
	{
		//HAL_UART_Transmit(&huart1, "CRC没过", strlen("CRC没过"), 1000);
	}
	modbus_restart_rx();
}
void modbus_stop_rx(void)
{
	HAL_UART_AbortReceive_IT(&huart1);//停止串口接收
}

void modbus_start_rx(void)
{
	HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //开启接收中断
}

void modbus_restart_rx(void)
{
	//清空接收数组，开启串口接收
	memset(modbus_rx_buffer, 0, S_RX_BUF_SIZE);
	modbus_rx_count = 0;
	HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //开启接收中断
	modbus_only_handle_once = 1;
}
void modbus_rx_data_handle(void)
{
	if (modbus_rx_buffer[0] == modbus_slave_addr)//确定从机地址对得上
	{
		memset(modbus_tx_buffer, 0, S_RX_BUF_SIZE); //清空接收缓存
		modbus_tx_buffer[0] = modbus_slave_addr; //地址
		switch (modbus_rx_buffer[1])
		{
			case 0x03: //读寄存器
				modbus_tx_buffer[1] = 0x03;//功能码
				uint16_t add_tar = (modbus_rx_buffer[2] << 8) + modbus_rx_buffer[3];
				uint16_t read_reg_count;//寄存器个数
				read_reg_count = (modbus_rx_buffer[4] << 8) + modbus_rx_buffer[5];
				modbus_tx_buffer[2] = 2 * read_reg_count;//数据个数
				//轮询找寄存器地址,这里只写了read_reg_count为1的情况
				for (int i = 0; i < sizeof(motor_vel) / sizeof(motor_vel[0]); i++)
				{
					if (motor_vel[i].address == add_tar)
					{
						//将指定地址的寄存器数据拆分成3、4号元素
						modbus_tx_buffer[3] = motor_vel[i].data >> 8;
						modbus_tx_buffer[4] = motor_vel[i].data & 0xff;
					}
				}
				modbus_send(modbus_tx_buffer, 5);//CRC发送
				break;
			case 0x06: //写单个寄存器
				modbus_tx_buffer[1] = 0x06;//功能码
				uint16_t add_tar_write = (modbus_rx_buffer[2] << 8) + modbus_rx_buffer[3];

				uint16_t write_reg_data;//写入寄存器的值
				write_reg_data = (modbus_rx_buffer[4] << 8) + modbus_rx_buffer[5];
				
				//轮询找寄存器地址,这里只写了read_reg_count为1的情况
				for (int i = 0; i < sizeof(motor_vel) / sizeof(motor_vel[0]); i++)
				{
					if (motor_vel[i].address == add_tar_write)
					{
						motor_vel[i].data = write_reg_data;
						modbus_tx_buffer[2] = motor_vel[i].address >> 8;
						modbus_tx_buffer[3] = motor_vel[i].address & 0xff;
						modbus_tx_buffer[4] = motor_vel[i].data >> 8;
						modbus_tx_buffer[5] = motor_vel[i].data & 0xff;
					}
				}
				modbus_send(modbus_tx_buffer, 6);//CRC发送
				break;
			default:
				//HAL_UART_Transmit(&huart1, "功能码错误", strlen("功能码错误"), 1000);
				break;
		}

	}
	else
	{
		//HAL_UART_Transmit(&huart1, "地址错误", strlen("地址错误"), 1000);
	}
}
uint8_t rx_crc_again(void)
{
	int16_t rx_count_nocrc = modbus_rx_count - 2;//确保modbus_rx_count不超过128，写成16位保险一点
	assert_param(rx_count_nocrc > 0);//参数保护，防止越界——回头可以取ERROR和Hault中断都放个UART发送错误信息
	uint8_t crc_rx_buffer[modbus_rx_count - 2];
	for (int i = 0; i < modbus_rx_count - 2; i++)//除校验码，其他数据都存入新数组，而后计算CRC校验
	{
		crc_rx_buffer[i] = modbus_rx_buffer[i];
	}
	crc_rx_computer = modbus_crc16(crc_rx_buffer, modbus_rx_count - 2);//计算的CRC码
	crc_rx_rec = (modbus_rx_buffer[modbus_rx_count - 2] << 8) + modbus_rx_buffer[modbus_rx_count - 1];//接收的CRC码
	if (crc_rx_computer == crc_rx_rec)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
/*
*********************************************************************************************************
*	函 数 名: CRC16_Modbus
*	功能说明: 计算CRC。 用于Modbus协议。
*	形    参: _pBuf : 参与校验的数据
*			  _usLen : 数据长度
*	返 回 值: 16位整数值。 对于Modbus ，此结果高字节先传送，低字节后传送。
*
*   所有可能的CRC值都被预装在两个数组当中，当计算报文内容时可以简单的索引即可；
*   一个数组包含有16位CRC域的所有256个可能的高位字节，另一个数组含有低位字节的值；
*   这种索引访问CRC的方式提供了比对报文缓冲区的每一个新字符都计算新的CRC更快的方法；
*
*  注意：此程序内部执行高/低CRC字节的交换。此函数返回的是已经经过交换的CRC值；也就是说，该函数的返回值可以直接放置
*        于报文用于发送；
*********************************************************************************************************
*/
uint16_t modbus_crc16(uint8_t* _pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* 高CRC字节初始化 */
	uint8_t ucCRCLo = 0xFF; /* 低CRC 字节初始化 */
	uint16_t usIndex;  /* CRC循环中的索引 */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *_pBuf++; /* 计算CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*********************************************************************************************************
*函 数 名 : MODS_SendWithCRC
* 功能说明 : 发送一串数据, 自动追加2字节CRC
* 形    参 : _pBuf 数据；
* _ucLen 数据长度（不带CRC）
* 返 回 值 : 无
* ********************************************************************************************************
*/

void modbus_send(uint8_t * _pBuf, uint8_t _ucLen)
{
	uint16_t crc;
	uint8_t buf[S_TX_BUF_SIZE];

	memcpy(buf, _pBuf, _ucLen);
	crc = modbus_crc16(_pBuf, _ucLen);
	buf[_ucLen++] = crc >> 8;
	buf[_ucLen++] = crc;

	HAL_UART_Transmit(&huart1, buf, _ucLen, 1000);//1S超时
	//RS485_SendBuf(buf, _ucLen);//发送数据
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//	if (huart->Instance == USART1)
//	{
//		//缺一个判断超时，缺一个处理
//		if (modbus_rx_count < S_RX_BUF_SIZE-1)
//		{
//			modbus_timeout_count_last = HAL_GetTick();
//			modbus_rx_count++;
//			HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //开启接收中断
//			modbus_only_handle_once = 0;
//		}
//
//	}
//}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//	if (huart->Instance == USART1)
//	{
//		const static uint8_t uart_rx_len = 10;
//		const static uint8_t target_flag = 'B';
//		const static uint8_t target_len = 3;
//		test_flag = openmv_data_process_flag(rx_buffer, strlen((const char*)rx_buffer), target_flag);
//		//uart_count = 0;
////		memset(rx_buffer, 0, strlen((const char*)rx_buffer));
//		//uart_count++;
//		//if (uart_count > uart_rx_len)
//		//{
//		//	
//		//}
//		//HAL_UART_Receive_IT(&huart1, rx_buffer + uart_count, 1);
//		//标志位置一后，需要执行的任务
//		if (test_flag == 1)
//		{
//			my_printf("收到目标字符\r\n");
//			test_flag = 0;//如果需要单次执行
//		}
//		else
//		{
//			
//		}
//		memset(rx_buffer, 0, strlen((const char*)rx_buffer));
//		HAL_UART_Receive_IT(&huart1, rx_buffer, 10);
//
//	}
//}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	const static uint8_t uart_rx_len = 50;
	const static uint8_t target_flag = 'B';
	const static uint8_t target_len = 2;

	test_flag = openmv_data_process_float(rx_buffer, strlen((const char*)rx_buffer), target_len, tar_buffer);
	X_IN = tar_buffer[0];
	Y_IN = tar_buffer[1];
	
	//标志位置一后，需要执行的任务
	if (test_flag == 1)
	{
		my_printf("x:%.2f,y:%.2f\r\n",X_IN, Y_IN);
		test_flag = 0;//单次执行需要该语句
	}
	memset(rx_buffer, 0, 50);
	HAL_UART_Receive_IT(&huart1, rx_buffer+1, 50);
	//memset(rx_buffer, 0, 50);
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
		HAL_UART_Transmit(&huart1, "RUN Error_Handler", strlen("RUN Error_Handler"), 1000);
		HAL_Delay(1000);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
	uint8_t error_buffer[100];
	sprintf(error_buffer, "Wrong parameters value: file %s on line %d\r\n", file, line);
	HAL_UART_Transmit(&huart1, error_buffer, strlen(error_buffer), 1000);
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
