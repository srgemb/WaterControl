
//*************************************************************************************************
//
// Управление индикацией контроллера
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "led.h"
#include "fram.h"
#include "valve.h"
#include "events.h"
#include "modbus.h"
#include "zigbee.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
//интервал переключения контрольного индикатора
#define TIME_DELAY_NORM         200         //штатный режим работы (msec)
#define TIME_DELAY_ACTIVE       50          //обмен данными - радио модуль (msec)
#define TIME_ACTIVE             300         //длительность сигнала активности (TIME_DELAY_ACTIVE)

#define LED_ON                  0           //светодиод вкл
#define LED_OFF                 1           //светодиод выкл

#define LED_MODE_NONE           0           //светодиод в режиме вкл или выкл
#define LED_MODE_BLK            1           //светодиод в режиме переключения (мигания)
                                            //с интервалом TIME_DELAY

//*************************************************************************************************
// Переменные с внешним доступом
//*************************************************************************************************
osEventFlagsId_t led_event = NULL, chk_event = NULL;

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static osTimerId_t timer_led;
static uint32_t led_mode = TIME_DELAY_NORM;
static uint8_t led_hot_opn = LED_MODE_NONE,  led_hot_cls = LED_MODE_NONE;
static uint8_t led_cold_opn = LED_MODE_NONE, led_cold_cls = LED_MODE_NONE;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskLed( void *pvParameters );
static void TaskChk( void *pvParameters );
static void TimerCallback( void *arg );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task1_attr = {
    .name = "Led", 
    .stack_size = 512,
    .priority = osPriorityNormal
 };

static const osThreadAttr_t task2_attr = {
    .name = "Chk", 
    .stack_size = 256,
    .priority = osPriorityNormal
 };

static const osEventFlagsAttr_t evn_attr = { .name = "LedEvents" };
static const osTimerAttr_t timer_attr = { .name = "TimerLed" };

//*************************************************************************************************
// Инициализация задачи и очереди событий управления индикаторами
//*************************************************************************************************
void LedInit( void ) {

    //очередь событий
    led_event = osEventFlagsNew( &evn_attr );
    chk_event = osEventFlagsNew( &evn_attr );
    timer_led = osTimerNew( TimerCallback, osTimerOnce, NULL, &timer_attr );
    //создаем задачу
    osThreadNew( TaskLed, NULL, &task1_attr );
    osThreadNew( TaskChk, NULL, &task2_attr );
 }
 
//*************************************************************************************************
// Задача обработки событий управления индикаторами
//*************************************************************************************************
static void TaskLed( void *pvParameters ) {

    int32_t event;

    for ( ;; ) {
        osDelay( TIME_DELAY_NORM );
        event = osEventFlagsWait( led_event, EVN_LED_MASK, osFlagsWaitAny, 0 );
        if ( event > 0 ) {
            //т.к. timeout = 0, при отсутствии события возвращается osFlagsErrorResource = 0xFFFFFFFDU 
            //индикатор "открыт" электропривода крана холодной воды 
            if ( event & EVN_LED_ZB_ACTIVE )
                osEventFlagsSet( chk_event, EVN_LED_ZB_ACTIVE ); //транзитное сообщение
            if ( event & EVN_LED_COLD_OPN_OFF ) {
                led_cold_opn = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_COLD_OPN_GPIO_Port, LED_COLD_OPN_Pin, GPIO_PIN_SET );
               }
            if ( event & EVN_LED_COLD_OPN_ON ) {
                led_cold_opn = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_COLD_OPN_GPIO_Port, LED_COLD_OPN_Pin, GPIO_PIN_RESET );
               }
            if ( event & EVN_LED_COLD_OPN_BLK ) {
                led_cold_opn = LED_MODE_BLK;
                HAL_GPIO_WritePin( LED_COLD_OPN_GPIO_Port, LED_COLD_OPN_Pin, GPIO_PIN_SET );
               }
            //индикатор "закрыт" электропривода крана холодной воды 
            if ( event & EVN_LED_COLD_CLS_OFF ) {
                led_cold_cls = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_COLD_CLS_GPIO_Port, LED_COLD_CLS_Pin, GPIO_PIN_SET );
               }
            if ( event & EVN_LED_COLD_CLS_ON ) {
                led_cold_cls = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_COLD_CLS_GPIO_Port, LED_COLD_CLS_Pin, GPIO_PIN_RESET );
               }
            if ( event & EVN_LED_COLD_CLS_BLK ) {
                led_cold_cls = LED_MODE_BLK;
                HAL_GPIO_WritePin( LED_COLD_CLS_GPIO_Port, LED_COLD_CLS_Pin, GPIO_PIN_SET );
               }
            //индикатор "открыт" электропривода крана горячей воды 
            if ( event & EVN_LED_HOT_OPN_OFF ) {
                led_hot_opn = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_HOT_OPN_GPIO_Port, LED_HOT_OPN_Pin, GPIO_PIN_SET );
               }
            if ( event & EVN_LED_HOT_OPN_ON ) { 
                led_hot_opn = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_HOT_OPN_GPIO_Port, LED_HOT_OPN_Pin, GPIO_PIN_RESET );
               }
            if ( event & EVN_LED_HOT_OPN_BLK ) {
                led_hot_opn = LED_MODE_BLK;
                HAL_GPIO_WritePin( LED_HOT_OPN_GPIO_Port, LED_HOT_OPN_Pin, GPIO_PIN_SET );
               }
            //индикатор "закрыт" электропривода крана горячей воды 
            if ( event & EVN_LED_HOT_CLS_OFF ) { 
                led_hot_cls = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_HOT_CLS_GPIO_Port, LED_HOT_CLS_Pin, GPIO_PIN_SET );
               }
            if ( event & EVN_LED_HOT_CLS_ON ) {
                led_hot_cls = LED_MODE_NONE;
                HAL_GPIO_WritePin( LED_HOT_CLS_GPIO_Port, LED_HOT_CLS_Pin, GPIO_PIN_RESET );
               }
            if ( event & EVN_LED_HOT_CLS_BLK ) {
                led_hot_cls = LED_MODE_BLK;
                HAL_GPIO_WritePin( LED_HOT_CLS_GPIO_Port, LED_HOT_CLS_Pin, GPIO_PIN_SET );
               }
           }
        //режим "мигание" индикаторов "открыт"
        if ( led_cold_opn == LED_MODE_BLK )
            HAL_GPIO_TogglePin( LED_COLD_OPN_GPIO_Port, LED_COLD_OPN_Pin );
        if ( led_hot_opn == LED_MODE_BLK )
            HAL_GPIO_TogglePin( LED_HOT_OPN_GPIO_Port, LED_HOT_OPN_Pin );
        //режим "мигание" индикаторов "закрыт"
        if ( led_cold_cls == LED_MODE_BLK )
            HAL_GPIO_TogglePin( LED_COLD_CLS_GPIO_Port, LED_COLD_CLS_Pin );
        if ( led_hot_cls == LED_MODE_BLK )
            HAL_GPIO_TogglePin( LED_HOT_CLS_GPIO_Port, LED_HOT_CLS_Pin );
       }
 }

//*************************************************************************************************
// Задача обработки событий управления контрольным индикатором
//*************************************************************************************************
static void TaskChk( void *pvParameters ) {

    for ( ;; ) {
        osDelay( led_mode );
        if ( osEventFlagsWait( chk_event, EVN_LED_ZB_ACTIVE, osFlagsWaitAny, 0 ) == EVN_LED_ZB_ACTIVE ) {
            led_mode = TIME_DELAY_ACTIVE;
            osTimerStart( timer_led, TIME_ACTIVE );
           }
        //контрольный индикатор
        HAL_GPIO_TogglePin( LED_CHK_GPIO_Port, LED_CHK_Pin );
       }
 }

//*************************************************************************************************
// CallBack функция таймера, восстановление режима индикации
//*************************************************************************************************
static void TimerCallback( void *arg ) {

    led_mode = TIME_DELAY_NORM;
 }
