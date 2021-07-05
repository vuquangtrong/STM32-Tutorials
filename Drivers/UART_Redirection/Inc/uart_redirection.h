/*
 * UART_REDIRECTION.h
 *
 *  Created on: Apr 28, 2021
 *      Author: vqtrong
 */

#ifndef INC_UART_REDIRECTION_H_
#define INC_UART_REDIRECTION_H_

#include <sys/stat.h>
#include "main.h" // which includes HAL headers

/* function to set global an UART handler used to redirect */
void Set_UART_Redirection_Port(UART_HandleTypeDef *huart);

/* function declaration, see syscalls.c to get function prototype */
int _read(int file, char *ptr, int len);
int _write(int file, char *ptr, int len);
int _close(int file);
int _fstat(int file, struct stat *st);
int _isatty(int file);
int _lseek(int file, int ptr, int dir);

#endif /* INC_UART_REDIRECTION_H_ */
