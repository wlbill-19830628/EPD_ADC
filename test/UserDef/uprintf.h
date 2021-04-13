#ifndef __UPRINTF_H
#define	__UPRINTF_H


#include <stdio.h>
typedef enum
{
	U1 = 1,
	U2,
	U3,
	U4,
	U5,
	U6,
	U7,
	U8
}uart_num;

int fputc(int ch, FILE *f);
int fgetc(FILE *f);
int my_printf (uart_num un,const char *format, ...);
#endif /* __UPRINTF_H */
