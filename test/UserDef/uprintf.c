/**
 * @brief  stm32串口打印重定义函数库,适用于L0系列，采用LL库
 * @note   1.实际调用打印函数为  int my_printf (uart_num un,const char *format, ...)，增加一个串口号参数un，其余用法与标准printf函数相同;
 *         2.串口选择与重定向在int fputc( int ch, FILE *f )中实现，用户可以根据实际使用芯片的串口号修改。
 *				 3.int fgetc(FILE *f)重定义实现串口输入数据扫描功能，用户直接调用标准库的scanf()可以实现对串口输入数据扫描。
 * @author  王亮  
 */

#include "uprintf.h"
#include <stdarg.h>
#include "usart.h"

//重定向c库函数printf到USART

 uart_num uart_port=U1;  //默认打印口U4

//功能： 重映射peintf到指定串口
//参数：uart_printf_flag；要映射地串口号
//		
//返回值： 串口号
int fputc( int ch, FILE *f )//printf重映射函数
{ 	
	switch(uart_port)//选择串口
	{
		case U1: while((USART1->ISR & 0X40) == 0);
											     /* 串口发送完成，将该字符发送 */
													USART1->TDR = (uint8_t) ch;    
				break;
		case U2: while((USART2->ISR & 0X40) == 0);
													/* 串口发送完成，将该字符发送 */
													USART2->TDR = (uint8_t) ch; 
				break;

		default:
				break;	
	}
	return ch;
}
//函数名：my_printf();
//功能： 封装printf函数 
//参数：ch；串口号
//		format：格式化字符串列表 （可选）
//返回值：发送的字符个数
 int my_printf (uart_num un,const char *format, ...)
{
	__disable_irq();    //禁用中断
	uart_port=un;
	int done=0;
	va_list(arg);
	va_start (arg, format);
	done = vprintf (format, arg);  //这里应该是vprintf 
	va_end (arg);
	__enable_irq();    //开启中断
	return done;
}
//重定向c库函数scanf到USART1
int fgetc(FILE *f)
{
		/* 等待串口3输入数据 */
		while (LL_USART_IsActiveFlag_RXNE(USART1) == 0);

		return (int)(uint16_t)(USART1->TDR & (uint16_t)0x01FF);
}
