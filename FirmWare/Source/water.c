
//*************************************************************************************************
//
// Обработка состояния датчиков утечки, расхода и давления воды
//
//*************************************************************************************************

#include <string.h>
#include <stdbool.h>
#include <stdio.h>

#include "cmsis_os2.h"

#include "can.h"
#include "data.h"
#include "main.h"
#include "fram.h"
#include "uart.h"
#include "water.h"
#include "config.h"
#include "events.h"
#include "xtime.h"
#include "message.h"

//#define DEBUG_WATER                                     //вывод отладочных событий
//#define DEBUG_PRESSURE                                  //вывод отладочных значений давления

//*************************************************************************************************
// Внешние переменные
//*************************************************************************************************
extern CONFIG config;
extern VALVE valve_data;
extern ADC_HandleTypeDef hadc1;

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define VREF_VOLTAGE            3.292                   //опорное напряжение
#define ADC_MAX_VALUE           4096                    //максимальное значение АЦП
#define CONV_3V3_5V             (4.961/VREF_VOLTAGE)    //коэффициент пересчета в 5 вольтовый диапазон

#define TIME_READ_PRESSURE      500                     //интервал чтения давления воды (msec)

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
CURR_DATA curr_data;
WATER_LOG water_log;
uint16_t pressure_cold, pressure_hot;
osEventFlagsId_t water_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static char str[60];
static CAN_DATA can_data;
static uint16_t adc_data[2];
static osTimerId_t timer1 = NULL;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void IncCount( CountType type );
static void WaterPressure( void );
static void TaskWater( void *pvParameters );
static void Timer1Callback( void *arg );
static void ValueWater( void );
static void SaveData( EventType type );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Water", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };

static const osTimerAttr_t timer_attr = { .name = "WaterTimer" };
static const osEventFlagsAttr_t evn_attr = { .name = "WaterEvents" };

//*************************************************************************************************
// Инициализация задачи и очереди событий счетчиков воды и датчиков утечки
//*************************************************************************************************
void WaterInit( void ) {

    //очередь событий
    water_event = osEventFlagsNew( &evn_attr );
    //таймеры интервалов
    timer1 = osTimerNew( Timer1Callback, osTimerPeriodic, NULL, &timer_attr );
    //создаем задачу
    osThreadNew( TaskWater, NULL, &task_attr );
    //калибровка АЦП
    HAL_ADCEx_Calibration_Start( &hadc1 );
 }
 
//*************************************************************************************************
// Задача обработки событий от датчиков утечки и счетчиков воды
//*************************************************************************************************
static void TaskWater( void *pvParameters ) {

    int32_t event;
    DATE_TIME date_time; 

    //запуск таймера измерения давления
    if ( timer1 != NULL )
        osTimerStart( timer1, TIME_READ_PRESSURE );
    //вывод результата чтения текущих значений расхода воды из FRAM
    osEventFlagsSet( water_event, EVN_WTR_VALUE );
    for ( ;; ) {
        event = osEventFlagsWait( water_event, EVN_WTR_MASK, osFlagsWaitAny, osWaitForever );
        //обработка событий
        if ( event & EVN_WTR_CNT_COLD )
            IncCount( COUNT_COLD ); //инкремет значения счетчика холодной воды
        if ( event & EVN_WTR_CNT_HOT )
            IncCount( COUNT_HOT );  //инкремет значения счетчика горячей воды
        if ( event & EVN_WTR_CNT_FILTER )
            IncCount( COUNT_FILTER ); //инкремет значения счетчика питьевой воды
        if ( event & EVN_WTR_PRESSURE_START )
            HAL_ADC_Start_DMA( &hadc1, (uint32_t *)&adc_data, 2 ); //запуска измерения давления воды
        if ( event & EVN_WTR_PRESSURE_END )
            WaterPressure(); //расчет давления воды
        if ( event & EVN_WTR_LEAK1 || event & EVN_WTR_LEAK2 ) {
            //события "утечка воды"
            SaveData( EVENT_ALARM );
            #if defined( DEBUG_WATER ) && defined( DEBUG_TARGET )
            if ( event & EVN_WTR_LEAK1 )
                UartSendStr( "Water leak#1\r\n" );
            if ( event & EVN_WTR_LEAK2 )
                UartSendStr( "Water leak#2\r\n" );
            #endif
            //сообщение координатору сети о наличии утечки воды
            osEventFlagsSet( zb_ctrl, EVN_ZC_SEND_LEAKS );
           }
        if ( event & EVN_WTR_LOG ) {
            //проверка текущего времени, сохранение данных в журнал
            GetTimeDate( &date_time );
            if ( date_time.hour == 23 && date_time.min == 59 && date_time.sec == 59 )
                SaveData( EVENT_DATA ); //запись данных в 00:00:00
           }
        if ( event & EVN_WTR_VALUE )
            ValueWater();
       }
 }

//*************************************************************************************************
// Функция выводит результат чтения текущих значений расхода воды из FRAM
//*************************************************************************************************
static void ValueWater( void ) {

    //читаем текущие значения счетчиков расхода воды
    UartSendStr( "\r\nChecking current water flow values ... " );
    sprintf( str, "read: %s write: %s\r\n", FramErrorDesc( FramError( FRAM_ERR_READ ) ), FramErrorDesc( FramError( FRAM_ERR_WRITE ) ) );
    UartSendStr( str );
 }

//*************************************************************************************************
// Функция обратного вызова таймера, запуск измерения давления воды
//*************************************************************************************************
static void Timer1Callback( void *arg ) {

    adc_data[WATER_COLD] = 0;
    adc_data[WATER_HOT] = 0;
    osEventFlagsSet( water_event, EVN_WTR_PRESSURE_START );
 }

//*************************************************************************************************
// Функция поготовки данных и записи в FRAM
//-------------------------------------------------------------------------------------------------
// EventType type - тип события: интервальные данные, событие утечки
//*************************************************************************************************
static void SaveData( EventType type ) {

    uint8_t *ptr, len = 0;
    DATE_TIME date_time;

    if ( type == EVENT_ALARM ) {
        //подготовка сообщения для CAN шины
        ptr = GetDataCan1( &len );
        can_data.data_len = len;
        can_data.msg_id = 0;
        memcpy( can_data.data, ptr, len );
        //передача текущих данных в очередь CAN сообщений
        osMessageQueuePut( send_can, &can_data, 0, 0 );
       }
    //дата - время события
    GetTimeDate( &date_time );
    water_log.day = date_time.day;                              //день
    water_log.month = date_time.month;                          //месяц
    water_log.year = date_time.year;                            //год
    water_log.hour = date_time.hour;                            //часы
    water_log.min = date_time.min;                              //минуты
    water_log.sec = date_time.sec;                              //секунды
    //состояния электроприводов
    water_log.stat_valve_cold = ValveGetStatus( VALVE_COLD );   //статус электропривода холодный воды
    water_log.error_valve_cold = valve_data.error_cold;         //код ошибки электропривода холодный воды
    water_log.stat_valve_hot = ValveGetStatus( VALVE_HOT );     //статус электропривода горячей воды
    water_log.error_valve_hot = valve_data.error_hot;           //код ошибки электропривода горячей воды
    //значения учета расхода воды
    water_log.count_cold = curr_data.count_cold;                //значения счетчика холодной воды
    water_log.count_hot = curr_data.count_hot;                  //значения счетчика горячей воды
    water_log.count_filter = curr_data.count_filter;            //значения счетчика питьевой воды
    //значения датчиков                                 
    water_log.pressr_cold = pressure_cold;                      //давление холодной воды
    water_log.pressr_hot = pressure_hot;                        //давление горячей воды
    water_log.leak1 = LeakStatus( LEAK1 );                      //состояние датчика утечки #1
    water_log.leak2 = LeakStatus( LEAK2 );                      //состояние датчика утечки #2
    water_log.dc12_chk = DC12VStatus();                         //контроль напряжения 12VDc для питания датчиков утечки
    water_log.type_event = type;
    //сохранение данные в журнал
    if ( FramSaveData( WATER_DATA_LOG, (uint8_t *)&water_log, sizeof( water_log ) ) == FRAM_OK ) {
        //сохраним адрес размещения следующей записи в журнал
        FramSaveData( CURRENT_DATA, (uint8_t *)&curr_data, sizeof( curr_data ) );
       }
 }

//*************************************************************************************************
// Запрос состояния датчиков утечки воды
//-------------------------------------------------------------------------------------------------
// LeakType type   - номер датчика 1/2
// return LeakStat - состояние датчика
//*************************************************************************************************
LeakStat LeakStatus( LeakType type ) {

    //любой из датчиков
    if ( type == LEAK_ALL )
        return (LeakStat)( !HAL_GPIO_ReadPin( LEAK1_GPIO_Port, LEAK1_Pin ) | !HAL_GPIO_ReadPin( LEAK2_GPIO_Port, LEAK2_Pin ) ); 
    //датчик #1
    if ( type == LEAK1 )
        return (LeakStat)!HAL_GPIO_ReadPin( LEAK1_GPIO_Port, LEAK1_Pin );
    //датчик #2
    if ( type == LEAK2 )
        return (LeakStat)!HAL_GPIO_ReadPin( LEAK2_GPIO_Port, LEAK2_Pin );
    return LEAK_NO;
 }

//*************************************************************************************************
// Увеличение значений счетчиков воды
//-------------------------------------------------------------------------------------------------
// CountType type - тип счетчика
//*************************************************************************************************
static void IncCount( CountType type ) {

    bool change = false;
    
    if ( type == COUNT_COLD ) {
        change = true;
        curr_data.count_cold += config.inc_cnt_cold;
       }
    if ( type == COUNT_HOT ) {
        change = true;
        curr_data.count_hot += config.inc_cnt_hot;
       }
    if ( type == COUNT_FILTER ) {
        change = true;
        curr_data.count_filter += config.inc_cnt_filter;
       }
    //при изменении данных, запишем новые значения в FRAM
    if ( change == true )
        FramSaveData( CURRENT_DATA, (uint8_t *)&curr_data, sizeof( curr_data ) ); //запись данных в FRAM
 }

//*************************************************************************************************
// Возвращает состояние наличия напряжения 12VDC питания датчиков утечки воды
//-------------------------------------------------------------------------------------------------
// return DC12VStat - состояние наличия напряжения
//*************************************************************************************************
DC12VStat DC12VStatus( void ) {

    return (DC12VStat)!HAL_GPIO_ReadPin( CHK_12V_GPIO_Port, CHK_12V_Pin );
 }

//*************************************************************************************************
// Обработка результата измерений АЦП, результат измерений записывается 
// в переменные: pressure_cold, pressure_hot с пересчетом в давление
// Массив adc_data[], для хранения считанных данных АЦП, должен быть объявлен как uint16_t !
//*************************************************************************************************
static void WaterPressure( void ) {

    float coef_volt_pres;
    float prs_cold, prs_hot;
    #if defined( DEBUG_PRESSURE ) && defined( DEBUG_TARGET )
    float prs_cold1, prs_hot1;
    #endif

    //коэффициент коэффициента пересчета напряжения в давление
    coef_volt_pres = config.pressure_max / ( config.press_out_max - config.press_out_min );
    //пересчет значений АЦП в напряжение, диапазон 3.3 вольт
    prs_cold = ( (float)adc_data[WATER_COLD] * VREF_VOLTAGE ) / ADC_MAX_VALUE; 
    prs_hot  = ( (float)adc_data[WATER_HOT] * VREF_VOLTAGE ) / ADC_MAX_VALUE; 
    #if defined( DEBUG_PRESSURE ) && defined( DEBUG_TARGET )
    prs_cold1 = prs_cold;
    prs_hot1  = prs_hot;
    #endif
    //пересчет в диапазон 5 вольт
    prs_cold *= CONV_3V3_5V;
    prs_hot  *= CONV_3V3_5V;
    #if defined( DEBUG_PRESSURE ) && defined( DEBUG_TARGET )
    sprintf( str, "ADC1: 0x%04X/%5.3f Vin: %5.3f\r\n", adc_data[WATER_COLD], prs_cold1, prs_cold );
    UartSendStr( str );
    sprintf( str, "ADC2: 0x%04X/%5.3f Vin: %5.3f\r\n\r\n", adc_data[WATER_HOT], prs_hot1, prs_hot );
    UartSendStr( str );
    #endif
    //проверка диапазона и пересчет в давление
    if ( prs_cold < config.press_out_min || prs_cold > config.press_out_max )
        prs_cold = 0;
    else prs_cold = ( prs_cold - config.press_out_min ) * coef_volt_pres;
    if ( prs_hot < config.press_out_min || prs_hot > config.press_out_max )
        prs_hot = 0;
    else prs_hot  = ( prs_hot - config.press_out_min ) * coef_volt_pres;
    //пересчет значений давления в целое значение
    pressure_cold = (uint16_t)( prs_cold * 100 );
    pressure_hot = (uint16_t)( prs_hot * 100 );
 }
