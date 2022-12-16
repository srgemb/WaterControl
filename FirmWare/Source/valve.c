
//*************************************************************************************************
//
// Управление электроприводами кранов холодной/горячей воды
//
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "led.h"
#include "main.h"
#include "uart.h"
#include "valve.h"
#include "events.h"
#include "parse.h"

//#define DEBUG_VALVE                         //вывод отладочных событий

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
VALVE valve_data;
osEventFlagsId_t valve_event = NULL;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIME_DELAY_CHK_PWR      100         //задержка на проверку наличи тока в цепи 
                                            //электропривода при включении (msec)
#define TIME_DELAY_CHK_OFF      15000       //задержка на проверку выключения привода
                                            //продолжительность работы (msec)

//расшифровка состояния электропривода
static char * const status_desc[] = { "UNDEF", "CLOSE", "OPEN" };

//расшифровка ошибок электропривода
static char * const error_desc[] = { "OK", "OVERLOAD", "NO POWER", "TIMEOUT" };

static osTimerId_t timer1_chk, timer2_chk, timer3_chk, timer4_chk;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void LedCtrl( Valve valve );
static void TaskValve( void *pvParameters );
static void Timer1Callback( void *arg );
static void Timer2Callback( void *arg );
static void Timer3Callback( void *arg );
static void Timer4Callback( void *arg );
static ErrorStatus ValveCtrl( Valve valve, ValveCtrlMode ctrl );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Valve", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "ValveEvents" };
static const osTimerAttr_t timer1_attr = { .name = "ValveTimer1" };
static const osTimerAttr_t timer2_attr = { .name = "ValveTimer2" };
static const osTimerAttr_t timer3_attr = { .name = "ValveTimer3" };
static const osTimerAttr_t timer4_attr = { .name = "ValveTimer4" };

//*************************************************************************************************
// Инициализация задачи и очереди событий управления электроприводами
//*************************************************************************************************
void ValveInit( void ) {

    //очередь событий
    valve_event = osEventFlagsNew( &evn_attr );
    //таймеры интервалов
    timer1_chk = osTimerNew( Timer1Callback, osTimerOnce, NULL, &timer1_attr );
    timer2_chk = osTimerNew( Timer2Callback, osTimerOnce, NULL, &timer2_attr );
    timer3_chk = osTimerNew( Timer3Callback, osTimerOnce, NULL, &timer3_attr );
    timer4_chk = osTimerNew( Timer4Callback, osTimerOnce, NULL, &timer4_attr );
    //создаем задачу
    osThreadNew( TaskValve, NULL, &task_attr );
    //актуализация состояния индикации
    LedCtrl( VALVE_COLD );
    LedCtrl( VALVE_HOT );
 }

//*************************************************************************************************
// Задача обработки событий управления индикацией
//*************************************************************************************************
static void TaskValve( void *pvParameters ) {

    int32_t event;
    uint32_t evn_key;
    ValveStat vl_stat;

    for ( ;; ) {
        event = osEventFlagsWait( valve_event, EVN_VALVE_MASK, osFlagsWaitAny, osWaitForever );
        //**********************************************************************************
        //управления электроприводом холодной воды от внешних кнопок
        //**********************************************************************************
        if ( event & EVN_VALVE_COLD_KEY ) {
            #ifdef DEBUG_VALVE
            UartSendStr( "EVN_VALVE_COLD_KEY\r\n" );
            #endif
            evn_key = 0;
            ValveSetError( VALVE_COLD, VALVE_OK ); //сброс ошибки
            vl_stat = ValveGetStatus( VALVE_COLD );
            //проверка состояния электропривода: открыт/закрыт
            if ( vl_stat == VALVE_CLOSE )
                evn_key = EVN_VALVE_COLD_OPN;
            if ( vl_stat == VALVE_OPEN || vl_stat == VALVE_UNDEF )
                evn_key = EVN_VALVE_COLD_CLS;
            if ( evn_key )
                osEventFlagsSet( valve_event, evn_key );
           }
        //**********************************************************************************
        //события управления электроприводом холодной воды
        //**********************************************************************************
        if ( event & EVN_VALVE_COLD_OPN ) {
            if ( event & EVN_VALVE_PREV_CHECK ) {
                if ( ValveGetStatus( VALVE_COLD ) == VALVE_CLOSE ) {
                    //окрытие крана холодной воды при условии что он закрыт
                    if ( ValveCtrl( VALVE_COLD, VALVE_CTRL_OPEN ) == SUCCESS ) {
                        //запуск таймеров проверки
                        osTimerStart( timer1_chk, TIME_DELAY_CHK_PWR );
                        osTimerStart( timer2_chk, TIME_DELAY_CHK_OFF );
                       }
                   }
               }
            else {
                //окрытие крана холодной воды
                if ( ValveCtrl( VALVE_COLD, VALVE_CTRL_OPEN ) == SUCCESS ) {
                    //запуск таймеров проверки
                    osTimerStart( timer1_chk, TIME_DELAY_CHK_PWR );
                    osTimerStart( timer2_chk, TIME_DELAY_CHK_OFF );
                   }
               }
           }
        if ( event & EVN_VALVE_COLD_CLS ) {
            if ( event & EVN_VALVE_PREV_CHECK ) {
                if ( ValveGetStatus( VALVE_COLD ) == VALVE_OPEN ) {
                    //закрытие крана холодной воды при условии что он открыт
                    if ( ValveCtrl( VALVE_COLD, VALVE_CTRL_CLOSE ) == SUCCESS ) {
                        //запуск таймеров проверки
                        osTimerStart( timer1_chk, TIME_DELAY_CHK_PWR );
                        osTimerStart( timer2_chk, TIME_DELAY_CHK_OFF );
                       }
                   }
               }
            else {
                //закрытие крана холодной воды
                if ( ValveCtrl( VALVE_COLD, VALVE_CTRL_CLOSE ) == SUCCESS ) {
                    //запуск таймеров проверки
                    osTimerStart( timer1_chk, TIME_DELAY_CHK_PWR );
                    osTimerStart( timer2_chk, TIME_DELAY_CHK_OFF );
                   }
               }
           }
        if ( event & EVN_VALVE_COLD_STOP ) {
            //выключение крана холодной воды
            ValveCtrl( VALVE_COLD, VALVE_CTRL_STOP );
            //выключение индикации
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_OFF | EVN_LED_COLD_CLS_OFF );
           }
        //**********************************************************************************
        //события таймеров проверки состояния электропривода крана холодной воды
        //**********************************************************************************
        if ( event & EVN_VALVE_COLD_CHK1 ) {
            //проверка наличия нагрузки
            if ( ValveGetCtrl( VALVE_COLD, VALVE_STAT_PWR ) == RESET ) {
                //отсутствие тока в цепи 
                osTimerStop( timer2_chk );
                ValveCtrl( VALVE_COLD, VALVE_CTRL_STOP ); //выключаем управление
                ValveSetError( VALVE_COLD, VALVE_NOPWR ); //установка ошибки
                //включение индикации
                osEventFlagsSet( led_event, EVN_LED_COLD_OPN_BLK | EVN_LED_COLD_CLS_BLK );
                #ifdef DEBUG_VALVE
                UartSendStr( "No cold water drive current.\r\n" );
                #endif
               }
           }
        if ( event & EVN_VALVE_COLD_CHK2 ) {
            //проверка продолжительности работы
            if ( ValveGetCtrl( VALVE_COLD, VALVE_STAT_CTRL ) == SET ) {
                //привод включен
                ValveCtrl( VALVE_COLD, VALVE_CTRL_STOP );   //выключаем управление
                ValveSetError( VALVE_COLD, VALVE_TIMEOUT ); //установка ошибки
                //включение индикации
                osEventFlagsSet( led_event, EVN_LED_COLD_OPN_BLK | EVN_LED_COLD_CLS_BLK );
                #ifdef DEBUG_VALVE
                UartSendStr( "COLD WATER OVERTIME DRIVE\r\n" );
                #endif
               }
           }
        //**********************************************************************************
        //актуализация состояния индикации по завершению работы электроприводов
        //**********************************************************************************
        if ( event & EVN_VALVE_COLD_PWR ) {
            ValveCtrl( VALVE_COLD, VALVE_CTRL_STOP ); //выключаем управление
            LedCtrl( VALVE_COLD ); //управление индикацией
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_VALVE ); //сообщение координатору сети
            #ifdef DEBUG_VALVE
            UartSendStr( "COLD STOP\r\n" );
            #endif
           }
        //**********************************************************************************
        //выключение электропривода крана при наличии перегрузки в цепи питания привода
        //**********************************************************************************
        if ( event & EVN_VALVE_COLD_OVR ) {
            ValveCtrl( VALVE_COLD, VALVE_CTRL_STOP ); //выключение привода
            ValveSetError( VALVE_COLD, VALVE_OVR );   //установка ошибки
            //включение индикации
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_BLK | EVN_LED_COLD_CLS_BLK );
            #ifdef DEBUG_VALVE
            UartSendStr( "EVN_VALVE_COLD_OVR\r\n" );
            #endif
           }
        
        //**********************************************************************************
        //управления электроприводом горячей воды от внешних кнопок
        //**********************************************************************************
        if ( event & EVN_VALVE_HOT_KEY ) {
            #ifdef DEBUG_VALVE
            UartSendStr( "EVN_VALVE_HOT_KEY\r\n" );
            #endif
            evn_key = 0;
            ValveSetError( VALVE_HOT, VALVE_OK ); //сброс ошибки 
            //проверка состояния крана: открыт/закрыт
            vl_stat = ValveGetStatus( VALVE_HOT );
            if ( vl_stat == VALVE_CLOSE )
                evn_key = EVN_VALVE_HOT_OPN;
            if ( vl_stat == VALVE_OPEN || vl_stat == VALVE_UNDEF )
                evn_key = EVN_VALVE_HOT_CLS;
            if ( evn_key )
                osEventFlagsSet( valve_event, evn_key );
           }
        //**********************************************************************************
        //события управления электроприводом крана горячей воды
        //**********************************************************************************
        if ( event & EVN_VALVE_HOT_OPN ) {
            if ( event & EVN_VALVE_PREV_CHECK ) {
                if ( ValveGetStatus( VALVE_HOT ) == VALVE_CLOSE ) {
                    //открытие крана горячей воды при условии что он закрыт
                    if ( ValveCtrl( VALVE_HOT, VALVE_CTRL_OPEN ) == SUCCESS ) {
                        //запуск таймеров проверки
                        osTimerStart( timer3_chk, TIME_DELAY_CHK_PWR );
                        osTimerStart( timer4_chk, TIME_DELAY_CHK_OFF );
                       }
                   }
               }
            else {
                //закрытие крана горячей воды
                if ( ValveCtrl( VALVE_HOT, VALVE_CTRL_OPEN ) == SUCCESS ) {
                    //запуск таймеров проверки
                    osTimerStart( timer3_chk, TIME_DELAY_CHK_PWR );
                    osTimerStart( timer4_chk, TIME_DELAY_CHK_OFF );
                   }
               }
           }
        if ( event & EVN_VALVE_HOT_CLS ) {
            if ( event & EVN_VALVE_PREV_CHECK ) {
                if ( ValveGetStatus( VALVE_HOT ) == VALVE_OPEN ) {
                    //закрытие крана горячей воды при условии что он открыт
                    if ( ValveCtrl( VALVE_HOT, VALVE_CTRL_CLOSE ) == SUCCESS ) {
                        //запуск таймеров проверки
                        osTimerStart( timer3_chk, TIME_DELAY_CHK_PWR );
                        osTimerStart( timer4_chk, TIME_DELAY_CHK_OFF );
                       }
                   }
               }
            else {
                //закрытие крана горячей воды
                if ( ValveCtrl( VALVE_HOT, VALVE_CTRL_CLOSE ) == SUCCESS ) {
                    //запуск таймеров проверки
                    osTimerStart( timer3_chk, TIME_DELAY_CHK_PWR );
                    osTimerStart( timer4_chk, TIME_DELAY_CHK_OFF );
                   }
               }
           }
        if ( event & EVN_VALVE_HOT_STOP ) {
            //выключение крана холодной воды
            ValveCtrl( VALVE_HOT, VALVE_CTRL_STOP );
            //выключение индикации
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_OFF | EVN_LED_HOT_CLS_OFF );
           }
        //**********************************************************************************
        //события таймеров проверки состояния электропривода крана горячей воды
        //**********************************************************************************
        if ( event & EVN_VALVE_HOT_CHK1 ) {
            //проверка наличия нагрузки
            if ( ValveGetCtrl( VALVE_HOT, VALVE_STAT_PWR ) == RESET ) {
                //отсутствие тока в цепи привода 
                osTimerStop( timer4_chk );
                ValveCtrl( VALVE_HOT, VALVE_CTRL_STOP ); //выключаем управление
                ValveSetError( VALVE_HOT, VALVE_NOPWR ); //установка ошибки
                //включение индикации
                osEventFlagsSet( led_event, EVN_LED_HOT_OPN_BLK | EVN_LED_HOT_CLS_BLK );
                #ifdef DEBUG_VALVE
                UartSendStr( "No hot water drive current.\r\n" );
                #endif
               }
           }
        if ( event & EVN_VALVE_HOT_CHK2 ) {
            //проверка продолжительности работы
            if ( ValveGetCtrl( VALVE_HOT, VALVE_STAT_CTRL ) == SET ) {
                //привод включен
                ValveCtrl( VALVE_HOT, VALVE_CTRL_STOP );   //выключаем управление
                ValveSetError( VALVE_HOT, VALVE_TIMEOUT ); //установка ошибки
                //включение индикации
                osEventFlagsSet( led_event, EVN_LED_HOT_OPN_BLK | EVN_LED_HOT_CLS_BLK );
                #ifdef DEBUG_VALVE
                UartSendStr( "Exceeding the operating time of the hot water electric drive.\r\n" );
                #endif
               }
           }
        //**********************************************************************************
        //актуализация состояния индикации по завершению работы электроприводов
        //**********************************************************************************
        if ( event & EVN_VALVE_HOT_PWR ) {
            ValveCtrl( VALVE_HOT, VALVE_CTRL_STOP ); //выключаем управление
            //управление индикацией
            LedCtrl( VALVE_HOT ); //управление индикацией
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_VALVE ); //сообщение координатору сети
            #ifdef DEBUG_VALVE
            UartSendStr( "HOT STOP\r\n" );
            #endif
           }
        //**********************************************************************************
        //выключение электропривода крана при наличии перегрузки в цепи питания привода
        //**********************************************************************************
        if ( event & EVN_VALVE_HOT_OVR ) {
            ValveCtrl( VALVE_HOT, VALVE_CTRL_STOP ); //выключение привода
            ValveSetError( VALVE_HOT, VALVE_OVR );   //установка ошибки
            //включение индикации
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_BLK | EVN_LED_HOT_CLS_BLK );
            #ifdef DEBUG_VALVE
            UartSendStr( "EVN_VALVE_HOT_OVR\r\n" );
            #endif
           }
        }
 }

//*************************************************************************************************
// Функция обратного вызова таймера, проверка тока в цепи питания электропривода крана холодной воды
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    osEventFlagsSet( valve_event, EVN_VALVE_COLD_CHK1 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, проверка продолжительности работы электропривода крана холодной воды
//*************************************************************************************************
static void Timer2Callback( void *arg ) {

    osEventFlagsSet( valve_event, EVN_VALVE_COLD_CHK2 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, проверка тока в цепи питания электропривода крана горячей воды
//*************************************************************************************************
static void Timer3Callback( void *arg ) {

    osEventFlagsSet( valve_event, EVN_VALVE_HOT_CHK1 );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, проверка продолжительности работы электропривода крана горячей воды
//*************************************************************************************************
static void Timer4Callback( void *arg ) {

    osEventFlagsSet( valve_event, EVN_VALVE_HOT_CHK2 );
 }

//*************************************************************************************************
// Фуникция управления электроприводами
// Управление выполняется при отсутствии ошибок привода, для возобновления 
// управления необходимо сбросить код ошибки соответствующего привода
//-------------------------------------------------------------------------------------------------
// Valve valve      - тип электропривода: горячая/холодная вода
// ValveMode ctrl   - режим управления: открыть/закрыть/стоп
// return = ERROR   - управление не возможно, необходимо сбрость ошибку или тип привода не указан
//        = SUCCESS - управление выполняется
//*************************************************************************************************
static ErrorStatus ValveCtrl( Valve valve, ValveCtrlMode ctrl ) {

    //электропривод холодной воды
    if ( valve == VALVE_COLD ) {
        if ( valve_data.error_cold != VALVE_OK )
            return ERROR;
        if ( ctrl == VALVE_CTRL_STOP ) {
            //выключить управление
            HAL_GPIO_WritePin( COLD_DIR_GPIO_Port, COLD_DIR_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( COLD_EN_GPIO_Port, COLD_EN_Pin, GPIO_PIN_RESET );
            return SUCCESS;
           }
        if ( ctrl == VALVE_CTRL_OPEN ) {
            //открыть
            HAL_GPIO_WritePin( COLD_DIR_GPIO_Port, COLD_DIR_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( COLD_EN_GPIO_Port, COLD_EN_Pin, GPIO_PIN_SET );
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_BLK | EVN_LED_COLD_CLS_OFF );
            return SUCCESS;
           }
        if ( ctrl == VALVE_CTRL_CLOSE ) {
            //закрыть
            HAL_GPIO_WritePin( COLD_DIR_GPIO_Port, COLD_DIR_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( COLD_EN_GPIO_Port, COLD_EN_Pin, GPIO_PIN_SET );
            osEventFlagsSet( led_event, EVN_LED_COLD_CLS_BLK | EVN_LED_COLD_OPN_OFF );
            return SUCCESS;
           }
        return ERROR;
       }
    //электропривод горячей воды
    if ( valve == VALVE_HOT ) {
        if ( valve_data.error_hot != VALVE_OK )
            return ERROR;
        if ( ctrl == VALVE_CTRL_STOP ) {
            //выключить управление
            HAL_GPIO_WritePin( HOT_DIR_GPIO_Port, HOT_DIR_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( HOT_EN_GPIO_Port, HOT_EN_Pin, GPIO_PIN_RESET );
            return SUCCESS;
           }
        if ( ctrl == VALVE_CTRL_OPEN ) {
            //открыть
            HAL_GPIO_WritePin( HOT_DIR_GPIO_Port, HOT_DIR_Pin, GPIO_PIN_RESET );
            HAL_GPIO_WritePin( HOT_EN_GPIO_Port, HOT_EN_Pin, GPIO_PIN_SET );
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_BLK | EVN_LED_HOT_CLS_OFF );
            return SUCCESS;
           }
        if ( ctrl == VALVE_CTRL_CLOSE ) {
            //закрыть
            HAL_GPIO_WritePin( HOT_DIR_GPIO_Port, HOT_DIR_Pin, GPIO_PIN_SET );
            HAL_GPIO_WritePin( HOT_EN_GPIO_Port, HOT_EN_Pin, GPIO_PIN_SET );
            osEventFlagsSet( led_event, EVN_LED_HOT_CLS_BLK | EVN_LED_HOT_OPN_OFF );
            return SUCCESS;
           }
        return ERROR;
       }
    return ERROR;
 }

//*************************************************************************************************
// Возращает состояние управления приводом
//-------------------------------------------------------------------------------------------------
// Valve valve       - тип привода: горячая/холодная вода
// FlagStatus status - тип состояния: управление (вкл/выкл)/наличие нагрузки
// return = ERROR    - выкл/нагрузки нет
//        = SUCCESS  - вкл/есть нагрузка
//*************************************************************************************************
FlagStatus ValveGetCtrl( Valve valve, StatCtrl type ) {

    if ( valve == VALVE_COLD ) {
        //для холодной воды
        if ( type == VALVE_STAT_CTRL )
            return (FlagStatus)HAL_GPIO_ReadPin( COLD_EN_GPIO_Port, COLD_EN_Pin );
        if ( type == VALVE_STAT_PWR )
            return (FlagStatus)HAL_GPIO_ReadPin( PWR_COLD_GPIO_Port, PWR_COLD_Pin );
       }
    if ( valve == VALVE_HOT ) {
        //для горячей воды
        if ( type == VALVE_STAT_CTRL )
            return (FlagStatus)HAL_GPIO_ReadPin( HOT_EN_GPIO_Port, HOT_EN_Pin );
        if ( type == VALVE_STAT_PWR )
            return (FlagStatus)HAL_GPIO_ReadPin( PWR_HOT_GPIO_Port, PWR_HOT_Pin );
       }
    return RESET;
 }

//*************************************************************************************************
// Возращает состояние привода
//-------------------------------------------------------------------------------------------------
// Valve valve      - тип привода: горячая/холодная вода
// Status status    - тип состояния: открыт/закрыт
// return ValveStat - открыт/закрыт/не определено
//*************************************************************************************************
ValveStat ValveGetStatus( Valve valve ) {

    GPIO_PinState opn, cls;
    
    if ( valve != VALVE_COLD && valve != VALVE_HOT )
        return VALVE_UNDEF; //тип привода не определен
    if ( valve == VALVE_COLD ) {
        //состояние
        opn = HAL_GPIO_ReadPin( COLD_OPN_GPIO_Port, COLD_OPN_Pin );
        cls = HAL_GPIO_ReadPin( COLD_CLS_GPIO_Port, COLD_CLS_Pin );
       }
    if ( valve == VALVE_HOT ) {
        //состояние
        opn = HAL_GPIO_ReadPin( HOT_OPN_GPIO_Port, HOT_OPN_Pin );
        cls = HAL_GPIO_ReadPin( HOT_CLS_GPIO_Port, HOT_CLS_Pin );
       }
    //определение состояния электропривода
    if ( ( opn == GPIO_PIN_SET && cls == GPIO_PIN_SET ) || ( opn == GPIO_PIN_RESET && cls == GPIO_PIN_RESET ) )
        return VALVE_UNDEF;
    if ( opn == GPIO_PIN_RESET && cls == GPIO_PIN_SET )
        return VALVE_OPEN;
    if ( opn == GPIO_PIN_SET && cls == GPIO_PIN_RESET ) 
        return VALVE_CLOSE;
    return VALVE_UNDEF;
 }

//*************************************************************************************************
// Установка типа ошибки при управлении электроприводами
// Выключение индикации ошибки выполняется только при выключенном управлении приводами
//-------------------------------------------------------------------------------------------------
// Valve valve      - тип электропривода: горячая/холодная вода
// ValveError error - тип ошибки: нет нагрузки/перегрузка
//*************************************************************************************************
void ValveSetError( Valve valve, ValveError error ) {

    if ( valve == VALVE_COLD ) {
        valve_data.error_cold = error; //установка ошибки
        if ( error == VALVE_OK && ValveGetCtrl( VALVE_COLD, VALVE_STAT_CTRL ) == RESET ) 
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_OFF | EVN_LED_COLD_CLS_OFF ); //выключение индикации ошибки
       }
    if ( valve == VALVE_HOT ) {
        valve_data.error_hot = error;  //установка ошибки
        if ( error == VALVE_OK && ValveGetCtrl( VALVE_HOT, VALVE_STAT_CTRL ) == RESET ) 
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_OFF | EVN_LED_HOT_CLS_OFF ); //выключение индикации ошибки
       }
 }

//*************************************************************************************************
// Возвращает расшифровку статуса электропривода
//-------------------------------------------------------------------------------------------------
// Valve valve - тип электропривода: горячая/холодная вода
// return      - указатель на строку с результатом
//*************************************************************************************************
char *ValveStatusDesc( Valve valve ) {

    if ( valve == VALVE_COLD || valve == VALVE_HOT )
        return status_desc[ValveGetStatus( valve )];
    return NULL;
 }

//*************************************************************************************************
// Возвращает расшифровку команды электропривода
//-------------------------------------------------------------------------------------------------
// ValveCtrlMode mode - тип команды
// return             - указатель на строку с результатом
//*************************************************************************************************
char *ValveCtrlDesc( ValveCtrlMode mode ) {

    if ( mode < SIZE_ARRAY( status_desc )  )
        return status_desc[mode];
    return NULL;
 }

//*************************************************************************************************
// Возвращает расшифровку ошибку электропривода
//-------------------------------------------------------------------------------------------------
// Valve valve - тип электропривода: горячая/холодная вода
// return      - указатель на строку с результатом
//*************************************************************************************************
char *ValveErrorDesc( Valve valve ) {


    if ( valve == VALVE_COLD )
        return error_desc[valve_data.error_cold];
    if ( valve == VALVE_HOT )
        return error_desc[valve_data.error_hot];
    return NULL;
 }

//*************************************************************************************************
// Актуализирует индикацию состояния электроприводов холодной и горячей воды
//-------------------------------------------------------------------------------------------------
// Valve valve - тип электропривода: горячая/холодная вода
//*************************************************************************************************
static void LedCtrl( Valve valve ) {

    ValveStat vl_stat;
    
    if ( valve == VALVE_COLD ) {
        //индикация начального состояния электропривода крана холодной воды
        vl_stat = ValveGetStatus( VALVE_COLD );
        if ( vl_stat == VALVE_UNDEF )
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_BLK | EVN_LED_COLD_CLS_BLK );
        if ( vl_stat == VALVE_CLOSE )
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_OFF | EVN_LED_COLD_CLS_ON );
        if ( vl_stat == VALVE_OPEN )
            osEventFlagsSet( led_event, EVN_LED_COLD_OPN_ON | EVN_LED_COLD_CLS_OFF );
       }
    if ( valve == VALVE_HOT ) {
        //индикация начального состояния электропривода крана горячей воды
        vl_stat = ValveGetStatus( VALVE_HOT );
        if ( vl_stat == VALVE_UNDEF )
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_BLK | EVN_LED_HOT_CLS_BLK );
        if ( vl_stat == VALVE_CLOSE )
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_OFF | EVN_LED_HOT_CLS_ON );
        if ( vl_stat == VALVE_OPEN )
            osEventFlagsSet( led_event, EVN_LED_HOT_OPN_ON | EVN_LED_HOT_CLS_OFF );
       }
 }
