/**
 * @brief  stm32���ڴ�ӡ�ض��庯����,������L0ϵ�У�����LL��
 * @note   1.ʵ�ʵ��ô�ӡ����Ϊ  int my_printf (uart_num un,const char *format, ...)������һ�����ںŲ���un�������÷����׼printf������ͬ;
 *         2.����ѡ�����ض�����int fputc( int ch, FILE *f )��ʵ�֣��û����Ը���ʵ��ʹ��оƬ�Ĵ��ں��޸ġ�
 *				 3.int fgetc(FILE *f)�ض���ʵ�ִ�����������ɨ�蹦�ܣ��û�ֱ�ӵ��ñ�׼���scanf()����ʵ�ֶԴ�����������ɨ�衣
 * @author  ����  
 */

#include "uprintf.h"
#include <stdarg.h>
#include "usart.h"

//�ض���c�⺯��printf��USART

 uart_num uart_port=U1;  //Ĭ�ϴ�ӡ��U4

//���ܣ� ��ӳ��peintf��ָ������
//������uart_printf_flag��Ҫӳ��ش��ں�
//		
//����ֵ�� ���ں�
int fputc( int ch, FILE *f )//printf��ӳ�亯��
{ 	
	switch(uart_port)//ѡ�񴮿�
	{
		case U1: while((USART1->ISR & 0X40) == 0);
											     /* ���ڷ�����ɣ������ַ����� */
													USART1->TDR = (uint8_t) ch;    
				break;
		case U2: while((USART2->ISR & 0X40) == 0);
													/* ���ڷ�����ɣ������ַ����� */
													USART2->TDR = (uint8_t) ch; 
				break;

		default:
				break;	
	}
	return ch;
}
//��������my_printf();
//���ܣ� ��װprintf���� 
//������ch�����ں�
//		format����ʽ���ַ����б� ����ѡ��
//����ֵ�����͵��ַ�����
 int my_printf (uart_num un,const char *format, ...)
{
	__disable_irq();    //�����ж�
	uart_port=un;
	int done=0;
	va_list(arg);
	va_start (arg, format);
	done = vprintf (format, arg);  //����Ӧ����vprintf 
	va_end (arg);
	__enable_irq();    //�����ж�
	return done;
}
//�ض���c�⺯��scanf��USART1
int fgetc(FILE *f)
{
		/* �ȴ�����3�������� */
		while (LL_USART_IsActiveFlag_RXNE(USART1) == 0);

		return (int)(uint16_t)(USART1->TDR & (uint16_t)0x01FF);
}
