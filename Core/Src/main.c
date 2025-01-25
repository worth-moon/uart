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
//	MOTOR_VEL_TAR = 0,  // Ŀ���ٶ�
//	MOTOR_VEL_CUR = 1,  // ��ǰ�ٶ�
//	MOTOR_VEL_ERR = 2   // �ٶ����
//} MOTOR_VEL_REG;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define S_TX_BUF_SIZE 128
#define S_RX_BUF_SIZE 128

#define MOTOR_VEL_TAR   0// Ŀ���ٶ�
#define MOTOR_VEL_CUR   1// ��ǰ�ٶ�
#define MOTOR_VEL_ERR   2// �ٶ����
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float theta, theta_add,theta_max;

//����Э��
uint8_t uart_count = 0;
uint8_t test_flag = 0;
uint8_t rx_buffer[100];
float tar_buffer[10];
float X_IN,Y_IN;

//��������
uint8_t test_single_char;
uint8_t test_more_char[22] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17,0x18,0x19,0x20};
uint8_t test_str[] = "helloworld!\r\n";
uint8_t test_rx_buffer[255];
uint8_t rx_count;
/*=========== ����Ϊmodbus��ر��� ===========*/
uint8_t modbus_slave_addr = 0x01; //�ӻ���ַ,1~247������Ч��
const uint8_t modbus_function_code_read = 0x03; //��������
const uint8_t modbus_function_code_write = 0x06; //д������
//����Ϊ��ַ�������룬�������ǼĴ�����������ַ��������������
//MOTOR_VEL_REG MOTOR_VEL_1;
modbus_reg motor_vel[3];
//modbus_reg motor_vel_tar = {0x0001,0x03e8};//�����������ã���ͷ��for��ѯ��ʱ�򲻺ð죻������Ļ���ʧȥ�ɶ��ԣ���õİ취���ǽṹ������+ö�ٱ���ָʾ��;
//modbus_reg motor_vel_cur = {0x0002,0x0000};
//modbus_reg motor_vel_err = {0x0003,0x0000};
//�����ǽ��պʹ�����صı���
uint8_t modbus_rx_buffer[S_RX_BUF_SIZE]; //���ջ�����
uint8_t modbus_rx_count = 0; //���ռ�����

uint8_t modbus_tx_buffer[S_TX_BUF_SIZE]; //���ͻ�����

uint64_t modbus_timeout_count_last, modbus_timeout_count_now; //��ʱ������

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
// CRC ��λ�ֽ�ֵ��
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
��modbusдһ�׳��򣬻�ͷ���ӻ�����modbusģ�飬����ֱ����
* ��д�ӻ�����
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
	  if ((modbus_timeout_count_now = HAL_GetTick()) - modbus_timeout_count_last > 4 && modbus_timeout_count_last != 0)//��ʱ���
	  {
		  //HAL_UART_Transmit(&huart1, "��ʱ", strlen("��ʱ"), 1000);
		  if (modbus_only_handle_once == 0)//һ��ָ��ֻ��Ӧһ�Σ��ظ�����Ҫ�´ν��ղ�����Ӧ
		  {
			  //HAL_UART_Transmit(&huart1, "��������", strlen("��������"), 1000);
			  if (modbus_rx_count > 4)
			  {

				  modbus_analyze_rx_data();
			  }
			  else
			  {
				  //HAL_UART_Transmit(&huart1, "���ո�������", strlen("���ո�������"), 1000);
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
	if (rx_crc_again() == 1)//CRC�ܹ������к������
	{
		modbus_rx_data_handle();
	}
	else
	{
		//HAL_UART_Transmit(&huart1, "CRCû��", strlen("CRCû��"), 1000);
	}
	modbus_restart_rx();
}
void modbus_stop_rx(void)
{
	HAL_UART_AbortReceive_IT(&huart1);//ֹͣ���ڽ���
}

void modbus_start_rx(void)
{
	HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //���������ж�
}

void modbus_restart_rx(void)
{
	//��ս������飬�������ڽ���
	memset(modbus_rx_buffer, 0, S_RX_BUF_SIZE);
	modbus_rx_count = 0;
	HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //���������ж�
	modbus_only_handle_once = 1;
}
void modbus_rx_data_handle(void)
{
	if (modbus_rx_buffer[0] == modbus_slave_addr)//ȷ���ӻ���ַ�Ե���
	{
		memset(modbus_tx_buffer, 0, S_RX_BUF_SIZE); //��ս��ջ���
		modbus_tx_buffer[0] = modbus_slave_addr; //��ַ
		switch (modbus_rx_buffer[1])
		{
			case 0x03: //���Ĵ���
				modbus_tx_buffer[1] = 0x03;//������
				uint16_t add_tar = (modbus_rx_buffer[2] << 8) + modbus_rx_buffer[3];
				uint16_t read_reg_count;//�Ĵ�������
				read_reg_count = (modbus_rx_buffer[4] << 8) + modbus_rx_buffer[5];
				modbus_tx_buffer[2] = 2 * read_reg_count;//���ݸ���
				//��ѯ�ҼĴ�����ַ,����ֻд��read_reg_countΪ1�����
				for (int i = 0; i < sizeof(motor_vel) / sizeof(motor_vel[0]); i++)
				{
					if (motor_vel[i].address == add_tar)
					{
						//��ָ����ַ�ļĴ������ݲ�ֳ�3��4��Ԫ��
						modbus_tx_buffer[3] = motor_vel[i].data >> 8;
						modbus_tx_buffer[4] = motor_vel[i].data & 0xff;
					}
				}
				modbus_send(modbus_tx_buffer, 5);//CRC����
				break;
			case 0x06: //д�����Ĵ���
				modbus_tx_buffer[1] = 0x06;//������
				uint16_t add_tar_write = (modbus_rx_buffer[2] << 8) + modbus_rx_buffer[3];

				uint16_t write_reg_data;//д��Ĵ�����ֵ
				write_reg_data = (modbus_rx_buffer[4] << 8) + modbus_rx_buffer[5];
				
				//��ѯ�ҼĴ�����ַ,����ֻд��read_reg_countΪ1�����
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
				modbus_send(modbus_tx_buffer, 6);//CRC����
				break;
			default:
				//HAL_UART_Transmit(&huart1, "���������", strlen("���������"), 1000);
				break;
		}

	}
	else
	{
		//HAL_UART_Transmit(&huart1, "��ַ����", strlen("��ַ����"), 1000);
	}
}
uint8_t rx_crc_again(void)
{
	int16_t rx_count_nocrc = modbus_rx_count - 2;//ȷ��modbus_rx_count������128��д��16λ����һ��
	assert_param(rx_count_nocrc > 0);//������������ֹԽ�硪����ͷ����ȡERROR��Hault�ж϶��Ÿ�UART���ʹ�����Ϣ
	uint8_t crc_rx_buffer[modbus_rx_count - 2];
	for (int i = 0; i < modbus_rx_count - 2; i++)//��У���룬�������ݶ����������飬�������CRCУ��
	{
		crc_rx_buffer[i] = modbus_rx_buffer[i];
	}
	crc_rx_computer = modbus_crc16(crc_rx_buffer, modbus_rx_count - 2);//�����CRC��
	crc_rx_rec = (modbus_rx_buffer[modbus_rx_count - 2] << 8) + modbus_rx_buffer[modbus_rx_count - 1];//���յ�CRC��
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
*	�� �� ��: CRC16_Modbus
*	����˵��: ����CRC�� ����ModbusЭ�顣
*	��    ��: _pBuf : ����У�������
*			  _usLen : ���ݳ���
*	�� �� ֵ: 16λ����ֵ�� ����Modbus ���˽�����ֽ��ȴ��ͣ����ֽں��͡�
*
*   ���п��ܵ�CRCֵ����Ԥװ���������鵱�У������㱨������ʱ���Լ򵥵��������ɣ�
*   һ�����������16λCRC�������256�����ܵĸ�λ�ֽڣ���һ�����麬�е�λ�ֽڵ�ֵ��
*   ������������CRC�ķ�ʽ�ṩ�˱ȶԱ��Ļ�������ÿһ�����ַ��������µ�CRC����ķ�����
*
*  ע�⣺�˳����ڲ�ִ�и�/��CRC�ֽڵĽ������˺������ص����Ѿ�����������CRCֵ��Ҳ����˵���ú����ķ���ֵ����ֱ�ӷ���
*        �ڱ������ڷ��ͣ�
*********************************************************************************************************
*/
uint16_t modbus_crc16(uint8_t* _pBuf, uint16_t _usLen)
{
	uint8_t ucCRCHi = 0xFF; /* ��CRC�ֽڳ�ʼ�� */
	uint8_t ucCRCLo = 0xFF; /* ��CRC �ֽڳ�ʼ�� */
	uint16_t usIndex;  /* CRCѭ���е����� */

	while (_usLen--)
	{
		usIndex = ucCRCHi ^ *_pBuf++; /* ����CRC */
		ucCRCHi = ucCRCLo ^ s_CRCHi[usIndex];
		ucCRCLo = s_CRCLo[usIndex];
	}
	return ((uint16_t)ucCRCHi << 8 | ucCRCLo);
}

/*********************************************************************************************************
*�� �� �� : MODS_SendWithCRC
* ����˵�� : ����һ������, �Զ�׷��2�ֽ�CRC
* ��    �� : _pBuf ���ݣ�
* _ucLen ���ݳ��ȣ�����CRC��
* �� �� ֵ : ��
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

	HAL_UART_Transmit(&huart1, buf, _ucLen, 1000);//1S��ʱ
	//RS485_SendBuf(buf, _ucLen);//��������
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
//{
//	if (huart->Instance == USART1)
//	{
//		//ȱһ���жϳ�ʱ��ȱһ������
//		if (modbus_rx_count < S_RX_BUF_SIZE-1)
//		{
//			modbus_timeout_count_last = HAL_GetTick();
//			modbus_rx_count++;
//			HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //���������ж�
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
//		//��־λ��һ����Ҫִ�е�����
//		if (test_flag == 1)
//		{
//			my_printf("�յ�Ŀ���ַ�\r\n");
//			test_flag = 0;//�����Ҫ����ִ��
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
	
	//��־λ��һ����Ҫִ�е�����
	if (test_flag == 1)
	{
		my_printf("x:%.2f,y:%.2f\r\n",X_IN, Y_IN);
		test_flag = 0;//����ִ����Ҫ�����
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
