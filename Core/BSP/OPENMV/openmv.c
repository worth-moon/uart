#include "openmv.h"

/*
* openmv����������ΪA,B,C���ֱ�־λ��ʱ�򣬵��øú���������rx���顢���Ⱥ�Ŀ�꣨A/B/C��
* ����ֵ:targetΪAʱ����rx�����ҵ���#A;���򷵻�1����֮�򷵻�0������ͬ��
*/

//# ;
//�����ַ� A~Z  #A; 'A'
//��������  #1;     '1'
//�����ַ�  #;
//������
uint8_t openmv_data_process_flag(uint8_t* rx_data, uint8_t len, uint8_t target)
{
    // ����Ŀ���ַ���
    char target_str[4];//�ַ�����β
    snprintf(target_str, sizeof(target_str), "#%c;", target);
    // ����rx_data���飬����Ŀ���ַ���
    for (uint8_t i = 0; i < len; i++) 
    {
        if (rx_data[i] == target_str[0] && memcmp(&rx_data[i], target_str, 3) == 0) 
        {
            return 1; // �ҵ�Ŀ���ַ���������1
        }
    }
    return 0; // δ�ҵ�Ŀ���ַ���������0
}

/*
* �����ĸ�������rx_data�ǽ������ݵ����飬len�����鳤�ȣ�target_len��Ŀ�����ݳ��ȣ�target_data��Ŀ����������
* ��⵽target_data�����ݳ�����target_len���򷵻�1����֮����0.
* Ŀ�곤�����Ϊ20
*/
uint8_t openmv_data_process_float(uint8_t* rx_data, uint8_t len, uint8_t target_len, float* target_data)
{
    // ���Ŀ�곤���Ƿ���1��20֮��
    if (target_len < 1 || target_len > 20) 
    {
        return 0;
    }

    // ���� '#' �� ';' ��λ��
    char* start = strchr((char*)rx_data, '#');
    char* end = strchr((char*)rx_data, ';');

    // ����Ƿ��ҵ� '#' �� ';'
    if (start == NULL || end == NULL || end <= start) 
    {
        return 0;
    }

    // ȷ�� ';' �� '#' ֮��
    if (end <= start) 
    {
        return 0;
    }

    // ������ʽ�ַ���
    char format[100] = "#A%f";
    for (uint8_t i = 1; i < target_len; i++) {
        char temp[10];
        snprintf(temp, sizeof(temp), ",%c%%f", 'A' + i);
        strcat(format, temp);
    }
    strcat(format, ";");

    // ��ȡ������
    int scanned = sscanf(start, format, &target_data[0], &target_data[1], &target_data[2], &target_data[3], &target_data[4],
        &target_data[5], &target_data[6], &target_data[7], &target_data[8], &target_data[9],
        &target_data[10], &target_data[11], &target_data[12], &target_data[13], &target_data[14],
        &target_data[15], &target_data[16], &target_data[17], &target_data[18], &target_data[19]);

    // ����Ƿ�ɹ���ȡ���и�����
    if (scanned != target_len) {
        return 0;
    }

    return 1;
}