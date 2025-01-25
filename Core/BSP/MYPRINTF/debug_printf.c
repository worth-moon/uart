#include "debug_printf.h"


//内部耦合完成，只需修改.h文件的四个宏定义就能用
void my_printf(const char *fmt, ...) 
{
    static uint8_t debug_buffer[BUFFER_SIZE];
    va_list args; // 定义一个变量参数列表
    int len; // 用于存储格式化后的字符串长度

    // 初始化变量参数列表，使其指向fmt后面的参数
    va_start(args, fmt);

    // 使用vsnprintf函数将格式化的字符串写入debug_buffer，BUFFER_SIZE作为最大长度限制以防止溢出
    len = vsnprintf((char*)debug_buffer, BUFFER_SIZE, fmt, args);
    
    // 检查是否发生缓冲区溢出
    if(len >= BUFFER_SIZE) {
        // 处理潜在的缓冲区溢出错误，这里可以选择记录警告信息
        len = BUFFER_SIZE - 1; // 确保字符串被正确终止
    }

    // 确保字符串以空字符终止
    debug_buffer[len] = '\0';

    // 通过CDC接口发送格式化后的数据，len为实际发送的字节数（不包括末尾的'\0'）
    #if USB_DEBUG
        CDC_Transmit_FS(debug_buffer, len);
    #elif DMA_MODE
        HAL_UART_Transmit_DMA(&UART_HANDLER,debug_buffer,len);
    #else
        HAL_UART_Transmit(&UART_HANDLER,debug_buffer,len,HAL_MAX_DELAY);
    #endif
		
    // 清理变量参数列表，释放相关资源
    va_end(args);
		
}
