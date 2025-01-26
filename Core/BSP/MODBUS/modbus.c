#include "modbus.h"

#define S_TX_BUF_SIZE 128
#define S_RX_BUF_SIZE 128

#define MOTOR_VEL_TAR   0// Ŀ���ٶ�
#define MOTOR_VEL_CUR   1// ��ǰ�ٶ�
#define MOTOR_VEL_ERR   2// �ٶ����

typedef struct {
	uint16_t address;
	uint16_t data;
} modbus_reg;

/*=========== ����Ϊmodbus��ر��� ===========*/
uint8_t modbus_slave_addr = 0x01; //�ӻ���ַ,1~247������Ч��
const uint8_t modbus_function_code_read = 0x03; //��������
const uint8_t modbus_function_code_write = 0x06; //д������
//����Ϊ��ַ�������룬�������ǼĴ�����������ַ��������������
//MOTOR_VEL_REG MOTOR_VEL_1;
modbus_reg motor_vel[3];

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

void modbus_send(uint8_t* _pBuf, uint8_t _ucLen)
{
	uint16_t crc;
	uint8_t buf[S_TX_BUF_SIZE];

	memcpy(buf, _pBuf, _ucLen);
	crc = modbus_crc16(_pBuf, _ucLen);
	buf[_ucLen++] = crc >> 8;
	buf[_ucLen++] = crc;

	HAL_UART_Transmit(&huart1, buf, _ucLen, 1000);//1S��ʱ
}

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
			int start_index = 3;
			for (int i = 0; i < sizeof(motor_vel) / sizeof(motor_vel[0]) && read_reg_count > 0; i++)
			{
				if (motor_vel[i].address == add_tar)
				{
					for (int j = 0; j < read_reg_count && i + j < sizeof(motor_vel) / sizeof(motor_vel[0]); j++)
					{
						modbus_tx_buffer[start_index + 2 * j] = motor_vel[i + j].data >> 8;
						modbus_tx_buffer[start_index + 2 * j + 1] = motor_vel[i + j].data & 0xff;
					}
					break;
				}
			}
			modbus_send(modbus_tx_buffer, 3 + 2 * read_reg_count);//CRC����
			//}

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
		case 0x10:
			modbus_tx_buffer[1] = 0x10;//������
			uint16_t add_tar_write_continous = (modbus_rx_buffer[2] << 8) + modbus_rx_buffer[3];
			uint16_t write_byte_count = ((modbus_rx_buffer[4] << 8) + modbus_rx_buffer[5]) * 2;//���ﱩ¶��һ�����⣬���͵���ʮ������������ǧ��𵱳�ʮ��������������͸�������printf����ʮ������һ����Ц

			int start_index_10 = 7;
			for (int i = 0; i < sizeof(motor_vel) / sizeof(motor_vel[0]) && write_byte_count > 0; i++)
			{
				if (motor_vel[i].address == add_tar_write_continous)
				{
					for (int j = 0; j < write_byte_count && i + j < sizeof(motor_vel) / sizeof(motor_vel[0]); j++)
					{
						motor_vel[i + j].data = (modbus_rx_buffer[start_index_10 + 2 * j] << 8) + modbus_rx_buffer[start_index_10 + 2 * j + 1];
					}
					break;
				}
			}


			//���Ͳ���
			for (int i = 0; i < 6; i++)
			{
				modbus_tx_buffer[i] = modbus_rx_buffer[i];
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

void modbus_uart_rx_callback(void)
{
	if (modbus_rx_count < S_RX_BUF_SIZE - 1)
	{
		modbus_timeout_count_last = HAL_GetTick();
		modbus_rx_count++;
		HAL_UART_Receive_IT(&huart1, modbus_rx_buffer + modbus_rx_count, 1); //���������ж�
		modbus_only_handle_once = 0;
	}
}

void modbus_init(void)
{
	//�����ʼ����IO�ʹ��ڡ��ж�
	//������ʼ��
	motor_vel[MOTOR_VEL_TAR].address = 0x0000;
	motor_vel[MOTOR_VEL_TAR].data = 0x03e8;
	motor_vel[MOTOR_VEL_CUR].address = 0x0001;
	motor_vel[MOTOR_VEL_CUR].data = 0x1234;
	motor_vel[MOTOR_VEL_ERR].address = 0x0002;
	motor_vel[MOTOR_VEL_ERR].data = 0x5678;
	//��������
	modbus_start_rx();
}

//�ú��������������Ż�����ʱ��/SYSȥ������while���䲻ȷ����
void modbus_loop(void)
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
}