
#ifndef __VERSION_H
#define __VERSION_H

#include <stdint.h>
#include <stdbool.h>

uint32_t GetFwDate( void );
uint32_t GetFwTime( void );
uint32_t GetFwVersion( void );
char *FWDate( uint32_t fw_date );
char *FWTime( uint32_t fw_time );
char *FWVersion( uint32_t version );

#endif 
