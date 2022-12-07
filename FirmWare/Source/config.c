
//*************************************************************************************************
//
// Чтение/запись параметров настроек в/из FLASH
// 
//*************************************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "cmsis_os2.h"

#include "can.h"
#include "fram.h"
#include "uart.h"
#include "crc16.h"
#include "config.h"
#include "events.h"
#include "parse.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//расшифровка кодов ошибок записи в FLASH память, источника сброса контроллера
static char result[80];
static char * const read_error[]  = { "DEFAULT", "OK" };
static char * const mask_error[]  = { "UNLOCK-", "ERASE-", "PROGRAM-", "LOCK-" };
static char * const flash_error[] = { "ОК",      "ERROR",  "BUSY",     "TIMEOUT" };
static char * const reset_desc[]  = { "PINRST ", "PORRST ", "SFTRST ", "IWDGRST ", "WWDGRST ", "LPWRRST " };

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
CONFIG config;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static uint32_t reset_flg;
static FLASH_DATA  flash_data;
static ErrorStatus flash_read;

#pragma push        //принудительное выключение оптимизации для ConfigInit()
#pragma O0

//*************************************************************************************************
// Инициализация параметров конфигурации контроллера
// Параметры хранятся в FLASH памяти микроконтроллера
// После чтения параметров в переменной flash_read хранится результат проверки CRC
//*************************************************************************************************
void ConfigInit( void ) {

    uint16_t crc;
    uint8_t dw, dw_cnt;
    uint32_t *source_addr, *dest_addr;
    uint8_t key[16] = { 0x11,0x13,0x15,0x17,0x19,0x1B,0x1D,0x1F,0x10,0x12,0x14,0x16,0x18,0x1A,0x1C,0x1D };

    memset( (uint8_t *)&config, 0x00, sizeof( config ) );
    memset( (uint8_t *)&flash_data, 0x00, sizeof( flash_data ) );
    //загрузка параметров конфигурации из FLASH памяти
    source_addr = (uint32_t *)FLASH_DATA_ADDRESS;
    dest_addr = (uint32_t *)&flash_data;
    dw_cnt = sizeof( flash_data )/sizeof( uint32_t );
    //читаем значение только как WORD (по 4 байта)
    for ( dw = 0; dw < dw_cnt; dw++, source_addr++, dest_addr++ )
        *dest_addr = *(__IO uint32_t *)source_addr;
    //проверка CRC
    crc = CalcCRC16( (uint8_t *)&flash_data, sizeof( flash_data.data ) );
    if ( crc != flash_data.crc ) {
        //ошибка контрольной суммы, установка значений по умолчанию
        config.debug_speed = UART_SPEED_115200;     //скорость отладочного порта (RS232)
        //параметры инкремента счетчиков
        config.inc_cnt_cold = 1;                    //шаг инкремента значения счетчика холодной воды
        config.inc_cnt_hot = 1;                     //шаг инкремента значения счетчика горячей воды
        config.inc_cnt_filter = 10;                 //шаг инкремента значения счетчика питьевой воды
        //параметры датчиков давления
        config.pressure_max = 6.0;                  //максимальное измеряемое значение датчика давления
        config.press_out_min = 0.4;                 //минимальное напряжения на выходе датчика давления
        config.press_out_max = 4.5;                 //максимальное напряжения на выходе датчика давления
        //параметры CAN
        config.can_id = 0x550;                      //ID устройства
        config.can_addr = CAN_ADDRESS_29_BIT;       //тип адресации CAN шины 0/1 = 11/29 бит
        config.can_speed = CAN_SPEED_125;           //скорость CAN шины
        //параметры MODBUS
        config.modbus_id = 0x10;                    //ID устройства MODBUS
        config.modbus_speed = UART_SPEED_19200;     //скорость RS-485 шины
        //параметры радио модуля и параметров сети
        config.net_pan_id = 0x0001;                 //Personal Area Network ID – идентификатор сети
        config.net_group = 1;                       //номер группы
        memcpy( config.net_key, key, sizeof( config.net_key ) ); //ключ шифрования
        config.dev_numb = 0x0001;                   //адрес уст-ва в сети (логический номер уст-ва)
        config.addr_gate = 0x0000;                  //адрес шлюза с сети
        flash_read = ERROR;
       }
    else {
        //параметры прочитаны без ошибок
        memcpy( (uint8_t *)&config, (uint8_t *)&flash_data, sizeof( config ) );
        flash_read = SUCCESS;
       }
    //сохраним источник сброса
    reset_flg = ( RCC->CSR >>= 26 );
    __HAL_RCC_CLEAR_RESET_FLAGS();
 }

#pragma pop     //восстановление оптимизации

//*************************************************************************************************
// Сохранение параметров в FLASH памяти
//-------------------------------------------------------------------------------------------------
// return - код ошибки (набор ошибок) 
//*************************************************************************************************
uint8_t ConfigSave( void ) {

    uint8_t dw, dw_cnt;
    uint32_t err_addr, *ptr_glb, ptr_flash;
    HAL_StatusTypeDef stat_flash;
    FLASH_EraseInitTypeDef erase;
    
    memset( (uint8_t *)&flash_data, 0x00, sizeof( flash_data ) );
    memcpy( (uint8_t *)&flash_data, (uint8_t *)&config, sizeof( config ) );
    //расчет КС блока данных
    flash_data.crc = CalcCRC16( (uint8_t *)&flash_data, sizeof( flash_data.data ) );
    osKernelLock(); //начало критической секция кода
    //разблокируем память
    stat_flash = HAL_FLASH_Unlock();
    if ( stat_flash != HAL_OK ) {
        osKernelUnlock(); //окончание критической секция кода
        return ERR_FLASH_UNLOCK | stat_flash;
       }
    //стирание одной страницы памяти
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.PageAddress = FLASH_DATA_ADDRESS;
    erase.NbPages = 1;
    stat_flash = HAL_FLASHEx_Erase( &erase, &err_addr );
    if ( stat_flash != HAL_OK ) {
        osKernelUnlock(); //окончание критической секция кода
        return ERR_FLASH_ERASE | stat_flash;
       }
    //запись в FLASH только по 4 байта
    ptr_glb = (uint32_t *)&flash_data;
    ptr_flash = FLASH_DATA_ADDRESS;
    dw_cnt = sizeof( flash_data )/sizeof( uint32_t );
    for ( dw = 0; dw < dw_cnt; dw++, ptr_glb++, ptr_flash += 4 ) {
        stat_flash = HAL_FLASH_Program( FLASH_TYPEPROGRAM_WORD, ptr_flash, *ptr_glb );    
        if ( stat_flash != HAL_OK ) {
            osKernelUnlock(); //окончание критической секция кода
            return ERR_FLASH_PROGRAMM | stat_flash;
           }
       }
    //блокировка памяти
    stat_flash = HAL_FLASH_Lock();
    osKernelUnlock(); //окончание критической секция кода
    if ( stat_flash != HAL_OK )
        return ERR_FLASH_LOCK | stat_flash;
    return HAL_OK;
 }

//*************************************************************************************************
// Расшифровка ошибок результата чтения параметров конфигурации из FLASH памяти
//-------------------------------------------------------------------------------------------------
// return - указатель на строку с расшифровкой
//*************************************************************************************************
char *FlashReadStat( void ) {

    if ( flash_read == ERROR )
        return read_error[0];
    if ( flash_read == SUCCESS )
        return read_error[1];
    return NULL;
 }
 
//*************************************************************************************************
// Расшифровка кодов ошибок при записи в FLASH память микроконтроллера
//-------------------------------------------------------------------------------------------------
// uint8_t error - код ошибки
// return        - указатель на строку с расшифровкой кодов ошибок
//*************************************************************************************************
char *ConfigError( uint8_t error ) {

    if ( !error )
        return flash_error[error];
    memset( result, 0x00, sizeof( result ) );
    //имя источника ошибки
    if ( error & ERR_FLASH_UNLOCK )
        strcpy( result, mask_error[0] );
    if ( error & ERR_FLASH_ERASE )
        strcpy( result, mask_error[1] );
    if ( error & ERR_FLASH_PROGRAMM )
        strcpy( result, mask_error[2] );
    if ( error & ERR_FLASH_LOCK )
        strcpy( result, mask_error[3] );
    //ошибка функции
    strcat( result, flash_error[error & 0x0F] );
    return result;
 }

//*************************************************************************************************
// Функция возвращает источник сброса контроллера. 
// Биты источника сброса регистра RCC_CSR сдвинуты вправо к нулевому биту.
//-------------------------------------------------------------------------------------------------
// return - биты источника сброса.
//*************************************************************************************************
uint8_t ResetSrc( void ) {

    return (uint8_t)reset_flg;
 }

//*************************************************************************************************
// Возвращает расшифровку источника сброса микроконтроллера.
//-------------------------------------------------------------------------------------------------
// return - указатель на строку с расшифровкой источника сброса.
//*************************************************************************************************
char *ResetSrcDesc( void ) {

    uint8_t i, mask = 0x01;
    
    memset( result, 0x00, sizeof( result ) );
    for ( i = 0; i < SIZE_ARRAY( reset_desc ); i++, mask <<= 1 ) {
        if ( reset_flg & mask )
            strcat( result, reset_desc[i] );
       }
    return result;
 }
