#include "openmv.h"

/*
* openmv发来的数据为A,B,C这种标志位的时候，调用该函数，传入rx数组、长度和目标（A/B/C）
* 返回值:target为A时，在rx数组找到“#A;”则返回1，反之则返回0，其它同理
*/

//# ;
//单个字符 A~Z  #A; 'A'
//单个数字  #1;     '1'
//单个字符  #;
//多数据
uint8_t openmv_data_process_flag(uint8_t* rx_data, uint8_t len, uint8_t target)
{
    // 定义目标字符串
    char target_str[4];//字符串结尾
    snprintf(target_str, sizeof(target_str), "#%c;", target);
    // 遍历rx_data数组，查找目标字符串
    for (uint8_t i = 0; i < len; i++) 
    {
        if (rx_data[i] == target_str[0] && memcmp(&rx_data[i], target_str, 3) == 0) 
        {
            return 1; // 找到目标字符串，返回1
        }
    }
    return 0; // 未找到目标字符串，返回0
}

/*
* 传入四个参数，rx_data是接收数据的数组，len是数组长度，target_len是目标数据长度，target_data是目标数据数组
* 检测到target_data的数据长度有target_len，则返回1，反之返回0.
* 目标长度最大为20
*/
uint8_t openmv_data_process_float(uint8_t* rx_data, uint8_t len, uint8_t target_len, float* target_data)
{
    // 检查目标长度是否在1到20之间
    if (target_len < 1 || target_len > 20) 
    {
        return 0;
    }

    // 查找 '#' 和 ';' 的位置
    char* start = strchr((char*)rx_data, '#');
    char* end = strchr((char*)rx_data, ';');

    // 检查是否找到 '#' 和 ';'
    if (start == NULL || end == NULL || end <= start) 
    {
        return 0;
    }

    // 确保 ';' 在 '#' 之后
    if (end <= start) 
    {
        return 0;
    }

    // 构建格式字符串
    char format[100] = "#A%f";
    for (uint8_t i = 1; i < target_len; i++) {
        char temp[10];
        snprintf(temp, sizeof(temp), ",%c%%f", 'A' + i);
        strcat(format, temp);
    }
    strcat(format, ";");

    // 提取浮点数
    int scanned = sscanf(start, format, &target_data[0], &target_data[1], &target_data[2], &target_data[3], &target_data[4],
        &target_data[5], &target_data[6], &target_data[7], &target_data[8], &target_data[9],
        &target_data[10], &target_data[11], &target_data[12], &target_data[13], &target_data[14],
        &target_data[15], &target_data[16], &target_data[17], &target_data[18], &target_data[19]);

    // 检查是否成功提取所有浮点数
    if (scanned != target_len) {
        return 0;
    }

    return 1;
}