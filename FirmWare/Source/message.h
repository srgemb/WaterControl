
#ifndef __MESSAGE_H
#define __MESSAGE_H

#include <stdint.h>
#include <stdbool.h>

extern const char msg_crlr[];
extern const char msg_prompt[];
extern const char msg_ok[];
extern const char msg_error[];
extern const char msg_error_rd[];
extern const char msg_error_crc[];
extern const char msg_save[];
extern const char msg_space[];
extern const char msg_no_command[];
extern const char msg_err_param[];
extern const char msg_str_delim[];

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
uint8_t AddDot( char *src, uint8_t aligment, uint8_t prev_len );

#endif 
