/*
 * uart_redirection.c
 *
 *  Created on: Apr 28, 2021
 *      Author: vqtrong
 */

#include <stdio.h>
#include <errno.h>
#include <uart_redirection.h>

/* a gloable UART handler used to redirect */
UART_HandleTypeDef *g_huart = NULL;

void Set_UART_Redirection_Port(UART_HandleTypeDef *huart) {
  g_huart = huart;
  /*
   * Disable I/O buffering for STDOUT stream, so that
   * chars are sent out as soon as they are printed.
   */
  setvbuf(stdout, NULL, _IONBF, 0);
}

int _read(int file, char *ptr, int len) {
  HAL_StatusTypeDef hstatus;
  if (g_huart == NULL) {
    return EIO;
  }
  /* read one byte only, according to _fstat returning character device type */
  hstatus = HAL_UART_Receive(g_huart, (uint8_t*) ptr, 1, HAL_MAX_DELAY);
  if (hstatus == HAL_OK)
    return 1;
  else
    return EIO;
}

int _write(int file, char *ptr, int len) {
  HAL_StatusTypeDef hstatus;
  if (g_huart == NULL) {
    return EIO;
  }
  /* write full string */
  hstatus = HAL_UART_Transmit(g_huart, (uint8_t*) ptr, len, HAL_MAX_DELAY);
  if (hstatus == HAL_OK)
    return len;
  else
    return EIO;
}

int _close(int file) {
  /* no file, just return */
  return -1;
}

int _fstat(int file, struct stat *st) {
  /* return as a character device type, read one by one character */
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file) {
  /* use as a terminal */
  return 1;
}

int _lseek(int file, int ptr, int dir) {
  /* not allow seek, just read char by char */
  return 0;
}
