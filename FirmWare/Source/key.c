
//*************************************************************************************************
//
// Чтение состояния кнопок управления
// 
//*************************************************************************************************

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdbool.h>

#include "cmsis_os2.h"

#include "main.h"
#include "key.h"
#include "valve.h"
#include "events.h"

//*************************************************************************************************
// Локальные константы
//*************************************************************************************************
#define TIME_DELAY              20          //интервал сканирования (msec)

//*************************************************************************************************
// Локальные переменные
//*************************************************************************************************
static GPIO_PinState cold_key1, cold_key2, prev_cold_key = GPIO_PIN_SET;
static GPIO_PinState hot_key1,  hot_key2,  prev_hot_key  = GPIO_PIN_SET;
static GPIO_PinState cold_pwr1, cold_pwr2, prev_cold_pwr = GPIO_PIN_RESET;
static GPIO_PinState hot_pwr1,  hot_pwr2,  prev_hot_pwr  = GPIO_PIN_RESET;
static GPIO_PinState cold_ovr1, cold_ovr2, prev_cold_ovr = GPIO_PIN_RESET;
static GPIO_PinState hot_ovr1,  hot_ovr2,  prev_hot_ovr  = GPIO_PIN_RESET;

//*************************************************************************************************
// Прототипы локальных функций
//*************************************************************************************************
static void TaskKey( void *pvParameters );

//*************************************************************************************************
// Атрибуты объектов RTOS
//*************************************************************************************************
static const osThreadAttr_t task_attr = {
    .name = "Key", 
    .stack_size = 256,
    .priority = osPriorityNormal
 };

//*************************************************************************************************
// Инициализация задачи
//*************************************************************************************************
void KeyInit( void ) {

    osThreadNew( TaskKey, NULL, &task_attr );
 }
 
//*************************************************************************************************
// Задача сканирования кнопок и входов контроля
//*************************************************************************************************
static void TaskKey( void *pvParameters ) {

    for ( ;; ) {
        //первое сканирование кнопок управления
        cold_key1 = HAL_GPIO_ReadPin( KEY_COLD_GPIO_Port, KEY_COLD_Pin );
        hot_key1 = HAL_GPIO_ReadPin( KEY_HOT_GPIO_Port, KEY_HOT_Pin );
        //первое сканирование входов наличия тока в цепи электроприводов
        cold_pwr1 = HAL_GPIO_ReadPin( PWR_COLD_GPIO_Port, PWR_COLD_Pin );
        hot_pwr1 = HAL_GPIO_ReadPin( PWR_HOT_GPIO_Port, PWR_HOT_Pin );
        //первое сканирование входов наличия тока в цепи электроприводов
        cold_ovr1 = HAL_GPIO_ReadPin( OVR_COLD_GPIO_Port, OVR_COLD_Pin );
        hot_ovr1 = HAL_GPIO_ReadPin( OVR_HOT_GPIO_Port, OVR_HOT_Pin );
        osDelay( TIME_DELAY );
        //второе сканирование кнопок управления
        cold_key2 = HAL_GPIO_ReadPin( KEY_COLD_GPIO_Port, KEY_COLD_Pin );
        hot_key2 = HAL_GPIO_ReadPin( KEY_HOT_GPIO_Port, KEY_HOT_Pin );
        //второе сканирование входов наличия тока в цепи электроприводов
        cold_pwr2 = HAL_GPIO_ReadPin( PWR_COLD_GPIO_Port, PWR_COLD_Pin );
        hot_pwr2 = HAL_GPIO_ReadPin( PWR_HOT_GPIO_Port, PWR_HOT_Pin );
        //второе сканирование входов наличия тока в цепи электроприводов
        cold_ovr1 = HAL_GPIO_ReadPin( OVR_COLD_GPIO_Port, OVR_COLD_Pin );
        hot_ovr2 = HAL_GPIO_ReadPin( OVR_HOT_GPIO_Port, OVR_HOT_Pin );
        //определение состояния кнопок
        if ( cold_key1 == cold_key2 ) {
            if ( prev_cold_key == GPIO_PIN_SET && cold_key2 == GPIO_PIN_RESET )
                osEventFlagsSet( valve_event, EVN_VALVE_COLD_KEY );
            prev_cold_key = cold_key1;
           }
        if ( hot_key1 == hot_key2 ) {
            if ( prev_hot_key == GPIO_PIN_SET && hot_key2 == GPIO_PIN_RESET )
                osEventFlagsSet( valve_event, EVN_VALVE_HOT_KEY );
            prev_hot_key = hot_key1;
           }
        //определение состояния входов наличия тока в цепи электроприводов
        if ( cold_pwr1 == cold_pwr2 ) {
            if ( prev_cold_pwr == GPIO_PIN_SET && cold_pwr2 == GPIO_PIN_RESET )
                osEventFlagsSet( valve_event, EVN_VALVE_COLD_PWR ); //выключение нагрузки
            prev_cold_pwr = cold_pwr1;
           }
        if ( hot_pwr1 == hot_pwr2 ) {
            if ( prev_hot_pwr == GPIO_PIN_SET && hot_pwr2 == GPIO_PIN_RESET )
                osEventFlagsSet( valve_event, EVN_VALVE_HOT_PWR ); //нажатие кнопки
            prev_hot_pwr = hot_pwr1;
           }
        //сканирование входов наличия перегрузки в цепи электроприводов
        if ( cold_ovr1 == cold_ovr2 ) {
            if ( prev_cold_ovr == GPIO_PIN_RESET && cold_ovr2 == GPIO_PIN_SET )
                osEventFlagsSet( valve_event, EVN_VALVE_COLD_OVR ); //перегрузка
            prev_cold_ovr = cold_ovr2;
           }
        if ( hot_ovr1 == hot_ovr2 ) {
            if ( prev_hot_ovr == GPIO_PIN_RESET && hot_ovr2 == GPIO_PIN_SET )
                osEventFlagsSet( valve_event, EVN_VALVE_HOT_OVR ); //перегрузка
            prev_hot_ovr = hot_ovr2;
           }
       }
 }
