
//*************************************************************************************************
//
// Управление часами/календарем
// Используется формат UNIX-времени, кол-во секунд прошедших от 01.01.1970
// 
//*************************************************************************************************

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "xtime.h"

#include <stm32f1xx_hal_rtc.h>

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern RTC_HandleTypeDef hrtc;

//*************************************************************************************************
// Локальные константы
// 86400 - продолжительность суток в секундах
// 
//*************************************************************************************************
#define TBIAS_DAYS              ( ( 70 * (uint32_t)365 ) + 17 )
#define TBIAS_SECS              ( TBIAS_DAYS * (uint32_t)86400 )
#define	TBIAS_YEAR              1900

#define MONTAB( year )          ((((year) & 03) || ((year) == 0)) ? mos : lmos)
#define	DaysTo32( year, mon )   (((year - 1) / 4) + MONTAB(year)[mon])

//кол-во дней в году по месяцам с накоплением (обычный год)
const uint16_t lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
//кол-во дней в году по месяцам с накоплением (високосный год)
const uint16_t mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

static char * const dows_short[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void SecToDtime( uint32_t secsarg, DATE_TIME *ptr );
static uint32_t DtimeToSec( DATE_TIME *ptr );

static ErrorStatus RTC_EnterInitMode( RTC_HandleTypeDef *hrtc ); 
static ErrorStatus RTC_ExitInitMode( RTC_HandleTypeDef *hrtc );

//*************************************************************************************************
// Заполняет структуру DATE_TIME текущими значениями время/дата 
//-------------------------------------------------------------------------------------------------
// DateTime *ptr - указатель на структуру содежащую текущее значение время-дата
//*************************************************************************************************
void GetTimeDate( DATE_TIME *ptr ) {

    uint32_t high, low, secsarg; 
    
    high = READ_REG( hrtc.Instance->CNTH & RTC_CNTH_RTC_CNT );
    low = READ_REG( hrtc.Instance->CNTL & RTC_CNTL_RTC_CNT );
    secsarg = ( ( high << 16 ) | low );
    SecToDtime( secsarg, ptr );
 }

//*************************************************************************************************
// Устанавливает новое значение дата/время
//-------------------------------------------------------------------------------------------------
// struct timedate *ptr - указатель на структуру содежащую значение для установки дата/время
// return = HAL_OK      - выполнено успешно
//          HAL_ERROR   - ошибка инициализации RTC
//*************************************************************************************************
ErrorStatus SetTimeDate( DATE_TIME *ptr ) {

    uint32_t timecounter;
    ErrorStatus status = SUCCESS;

    //преобразование дата/время в секунды
    timecounter = DtimeToSec( ptr );
    
    //Set Initialization mode
    if ( RTC_EnterInitMode( &hrtc ) != HAL_OK ) 
        status = ERROR;
    else {
        //Set RTC COUNTER MSB word
        WRITE_REG( hrtc.Instance->CNTH, ( timecounter >> 16 ) );
        //Set RTC COUNTER LSB word
        WRITE_REG( hrtc.Instance->CNTL, ( timecounter & RTC_CNTL_RTC_CNT ) );
        //Wait for synchro 
        if ( RTC_ExitInitMode( &hrtc ) != HAL_OK )
            status = ERROR;
       }
    return status;
    
 }

//*************************************************************************************************
// Расчет кол-во секунд прошедших от TBIAS_YEAR года в значение дата/время.
//-------------------------------------------------------------------------------------------------
// uint32_t secsarg - кол-во секунд прошедших от TBIAS_YEAR года
// struct tm *ptr   - указатель на структуру содежащую значение дата/время после расчета 
//*************************************************************************************************
static void SecToDtime( uint32_t secsarg, DATE_TIME *ptr ) {

    uint32_t i, secs, days, mon, year;
    const uint16_t *pm;

    secs = (uint32_t)secsarg + TBIAS_SECS;
    days = 0;
    //расчет дней, часов, минут
    days += secs / 86400;
    secs = secs % 86400;
    ptr->hour = secs / 3600;
    secs %= 3600;
    ptr->min = secs / 60;
    ptr->sec = secs % 60;
    //расчет года
    for ( year = days / 365; days < ( i = DaysTo32( year, 0 ) + 365*year ); )
        --year;
    days -= i;
    ptr->year = year + TBIAS_YEAR;
    //расчет месяца
    pm = MONTAB( year );
    for ( mon = 12; days < pm[--mon]; );
    ptr->month = mon + 1;
    ptr->day = days - pm[mon] + 1;
 }

//*************************************************************************************************
// Преобразует значение дата/время в кол-во секунд прошедших от TBIAS_YEAR года.
//-------------------------------------------------------------------------------------------------
// struct timedate *ptr - структура содежащая текущее значение время-дата
// return               - значение кол-ва секунд
//*************************************************************************************************
static uint32_t DtimeToSec( DATE_TIME *ptr ) {

    uint32_t days, secs, mon, year;
 
    //перевод даты в кол-во дней от TBIAS_YEAR года
    mon = ptr->month - 1;
    year = ptr->year - TBIAS_YEAR;
    days  = DaysTo32( year, mon ) - 1;
    days += 365 * year;
    days += ptr->day;
    days -= TBIAS_DAYS;
    //перевод текущего времени в секунды и дней в секунды
    secs  = 3600 * ptr->hour;
    secs += 60 * ptr->min;
    secs += ptr->sec;
    secs += ( days * (uint32_t)86400 );
    return secs;
 }

//*************************************************************************************************
// Расчет дня недели по дате.
// Все деления целочисленные (остаток отбрасывается).
//-------------------------------------------------------------------------------------------------
// uint8_t day   - день месяца ( 1 ... 31 )
// uint8_t month - месяц ( 1 ... 12 )
// uint16_t year - год ( TBIAS_YEAR ... )
// return        = 0 — воскресенье, 1 — понедельник и т.д.
//*************************************************************************************************
uint8_t DayOfWeek( uint8_t day, uint8_t month, uint16_t year ) {
 
    uint16_t a, y, m;
    
    a = (14 - month) / 12;
    y = year - a;
    m = month + 12 * a - 2;
    return ( 7000 + (day + y + y / 4 - y / 100 + y / 400 + (31 * m) / 12 ) ) % 7;
 }

//*************************************************************************************************
// Включение режима инициализации RTC
// @file    stm32f1xx_hal_rtc.c
// @author  MCD Application Team
// @version V1.0.4
// @date    29-April-2016
// @brief   RTC HAL module driver.
//*************************************************************************************************
static ErrorStatus RTC_EnterInitMode( RTC_HandleTypeDef *hrtc ) {

    uint32_t tickstart = 0;

    tickstart = HAL_GetTick();
    //Wait till RTC is in INIT state and if Time out is reached exit
    while ( ( hrtc->Instance->CRL & RTC_CRL_RTOFF ) == (uint32_t)RESET ) {
        if ( ( HAL_GetTick() - tickstart ) > RTC_TIMEOUT_VALUE )
            return ERROR; //HAL_TIMEOUT
       }
    //Disable the write protection for RTC registers
    __HAL_RTC_WRITEPROTECTION_DISABLE( hrtc );
    return SUCCESS;  
 }

//*************************************************************************************************
// Завершение режима инициализации RTC
// @file    stm32f1xx_hal_rtc.c
// @author  MCD Application Team
// @version V1.0.4
// @date    29-April-2016
// @brief   RTC HAL module driver.
//*************************************************************************************************
static ErrorStatus RTC_ExitInitMode( RTC_HandleTypeDef *hrtc ) {

    uint32_t tickstart = 0;

    //Disable the write protection for RTC registers
    __HAL_RTC_WRITEPROTECTION_ENABLE( hrtc );
    tickstart = HAL_GetTick();
    //Wait till RTC is in INIT state and if Time out is reached exit
    while ( ( hrtc->Instance->CRL & RTC_CRL_RTOFF ) == (uint32_t)RESET ) {
        if ( ( HAL_GetTick() - tickstart ) >  RTC_TIMEOUT_VALUE )
            return ERROR; //HAL_TIMEOUT;
       }
    return SUCCESS;  
 }

//*************************************************************************************************
// Фозвращает текущее значение дата/время в буфере
//-------------------------------------------------------------------------------------------------
// char *buff        - указатель на буфер для размещения результата
// DataTimeMask mask - маска вывода
// return            - значение указателя на следующий свободный байт в буфере (на '\0')
//*************************************************************************************************
char *DateTimeStr( char *buff, DataTimeMask mask ) {

    DATE_TIME dt;
    char *ptr = buff;

    GetTimeDate( &dt );
    if ( mask & MASK_DATE )
        ptr += sprintf( ptr, "Date: %02u.%02u.%04u %s ", dt.day, dt.month, dt.year, 
        dows_short[ DayOfWeek( dt.day, dt.month, dt.year ) ] );
    if ( mask & MASK_TIME )
        ptr += sprintf( ptr, "Time: %02u:%02u:%02u ", dt.hour, dt.min, dt.sec );
    if ( mask )
        ptr += sprintf( ptr, "\r\n" );
    return ptr;
 }

//*************************************************************************************************
// Установка текущего значения времени, входной формат HH:MI:SS или HH:MI
//-------------------------------------------------------------------------------------------------
// char *time       - строка с временем в формате HH:MI:SS или HH:MI
//                    допустимые значения: HH:MI:SS 00-23:00-59:00-59
// return = SUCCESS - параметры указаны верно, время изменено
//          ERROR   - ошибка в формате или неправильное значение
//*************************************************************************************************
ErrorStatus TimeSet( char *time ) {

    DATE_TIME dt;
    uint8_t idx, chk = 0;
    char *mask = NULL, mask1[] = "NN:NN:NN", mask2[] = "NN:NN";

    if ( strlen( time ) == strlen( mask1 ) )
        mask = mask1;
    if ( strlen( time ) == strlen( mask2 ) )
        mask = mask2;
    if ( mask == NULL )
        return ERROR;
    //проверка формата
    for ( idx = 0; idx < strlen( mask ); idx++ ) {
        if ( mask[idx] == 'N' && isdigit( (int)*(time+idx) ) )
            chk++;
        if ( mask[idx] == ':' && ispunct( (int)*(time+idx) ) )
            chk++;
       }
    //читаем текущее значение
    GetTimeDate( &dt );
    if ( chk == strlen( mask1 ) ) {
        //полный формат "NN:NN:NN"
        //проверка значений
        dt.hour = atoi( time );
        dt.min = atoi( time + 3 );
        dt.sec = atoi( time + 6 );
        if ( dt.hour > 23 || dt.min > 59 || dt.sec > 59 )
            return ERROR;
        if ( SetTimeDate( &dt ) != HAL_OK )
            return ERROR;
        else return SUCCESS;
       }
    if ( chk == strlen( mask2 ) ) {
        //сокращенный формат "NN:NN"
        dt.hour = atoi( time );
        dt.min = atoi( time + 3 );
        dt.sec = 0;
        if ( dt.hour > 23 || dt.min > 59 )
            return ERROR;
        if ( SetTimeDate( &dt ) == ERROR )
            return ERROR;
        else return SUCCESS;
       }
    return ERROR;
 }

//*************************************************************************************************
// Установка текущего значения даты, входной формат DD.MM.YYYY
//-------------------------------------------------------------------------------------------------
// char *date       - строка с датой в формате DD.MM.YYYY
//                    допустимые значения: DD.MM.YYYY 01-31:01-31:00-99
// return = SUCCESS - параметры указаны верно, дата изменена
//          ERROR   - ошибка в формате или неправильное значение
//*************************************************************************************************
ErrorStatus DateSet( char *date ) {

    DATE_TIME dt;
    uint8_t idx, chk = 0;
    char mask[] = "NN.NN.NNNN";

    //проверка формата
    for ( idx = 0; idx < strlen( mask ); idx++ ) {
        if ( mask[idx] == 'N' && isdigit( (int)*(date+idx) ) )
            chk++;
        if ( mask[idx] == '.' && ispunct( (int)*(date+idx) ) )
            chk++;
       }
    if ( chk != strlen( mask ) )
        return ERROR; //не соответствие формату
    //читаем текущее значение
    GetTimeDate( &dt );
    //проверка значений
    dt.day = atoi( date );
    dt.month = atoi( date + 3 );
    dt.year = atoi( date + 6 );
    if ( dt.day < 1 || dt.day > 31 || dt.month < 1 || dt.month > 12 || dt.year > 2100 )
        return ERROR; //недопустимые значения
    //проверка прошла
    if ( SetTimeDate( &dt ) == ERROR )
        return ERROR;
    else return SUCCESS;
 }
