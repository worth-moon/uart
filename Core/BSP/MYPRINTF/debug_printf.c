#include "debug_printf.h"


//�ڲ������ɣ�ֻ���޸�.h�ļ����ĸ��궨�������
void my_printf(const char *fmt, ...) 
{
    static uint8_t debug_buffer[BUFFER_SIZE];
    va_list args; // ����һ�����������б�
    int len; // ���ڴ洢��ʽ������ַ�������

    // ��ʼ�����������б�ʹ��ָ��fmt����Ĳ���
    va_start(args, fmt);

    // ʹ��vsnprintf��������ʽ�����ַ���д��debug_buffer��BUFFER_SIZE��Ϊ��󳤶������Է�ֹ���
    len = vsnprintf((char*)debug_buffer, BUFFER_SIZE, fmt, args);
    
    // ����Ƿ������������
    if(len >= BUFFER_SIZE) {
        // ����Ǳ�ڵĻ�������������������ѡ���¼������Ϣ
        len = BUFFER_SIZE - 1; // ȷ���ַ�������ȷ��ֹ
    }

    // ȷ���ַ����Կ��ַ���ֹ
    debug_buffer[len] = '\0';

    // ͨ��CDC�ӿڷ��͸�ʽ��������ݣ�lenΪʵ�ʷ��͵��ֽ�����������ĩβ��'\0'��
    #if USB_DEBUG
        CDC_Transmit_FS(debug_buffer, len);
    #elif DMA_MODE
        HAL_UART_Transmit_DMA(&UART_HANDLER,debug_buffer,len);
    #else
        HAL_UART_Transmit(&UART_HANDLER,debug_buffer,len,HAL_MAX_DELAY);
    #endif
		
    // ������������б��ͷ������Դ
    va_end(args);
		
}
