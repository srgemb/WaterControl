
#ifndef __VALVE_H
#define __VALVE_H

#include <stdbool.h>
#include <stdio.h>

#include "main.h"

//тип электропривода
typedef enum {
    VALVE_COLD,                             //электропривод крана холодной воды
    VALVE_HOT                               //электропривод крана горячей воды
 } Valve;

//команды управления электроприводом
typedef enum {
    VALVE_CTRL_NOTHING,                     //ничего не делаем
    VALVE_CTRL_OPEN,                        //открыть кран
    VALVE_CTRL_CLOSE,                       //закрыть кран
    VALVE_CTRL_STOP                         //остановить привод
 } ValveCtrlMode;

//запрос состояние управления электропривода
typedef enum {
    VALVE_STAT_CTRL,                        //управление вкл/выкл
    VALVE_STAT_PWR                          //наличие нагрузки на приводе 
 } StatCtrl;

//состояние электропривода
typedef enum {
    VALVE_UNDEF,                            //не определено
    VALVE_CLOSE,                            //закрыт
    VALVE_OPEN                              //открыт
 } ValveStat;

//коды ошибок управления электроприводом
typedef enum {
    VALVE_OK,                               //ок
    VALVE_OVR,                              //перегрузка
    VALVE_NO_POWER,                         //нет нагрузки в цепи
    VALVE_TIMEOUT                           //превышение длительности работы 
                                            //нет подтверждения завершения работы привода
 } ValveError;

#pragma pack( push, 1 )

typedef struct {
    ValveError  error_cold : 4;             //ошибка электропривода холодной воды
    ValveError  error_hot  : 4;             //ошибка электропривода горячей воды
 } VALVE;
 
//Структура данных для хранения списка уст-в и их адресов
typedef struct {
    ValveStat   stat_valve_cold  : 2;       //статус электропривода холодный воды    
    ValveError  error_valve_cold : 2;       //код ошибки электропривода холодный воды
    ValveStat   stat_valve_hot : 2;         //статус электропривода горячей воды     
    ValveError  error_valve_hot : 2;        //код ошибки электропривода горячей воды 
} VALVE_STAT_ERR;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ValveInit( void );
void ValveSetError( Valve valve, ValveError error );
FlagStatus ValveGetCtrl( Valve valve, StatCtrl type );
ValveStat ValveGetStatus( Valve valve );
char *ValveStatusDesc( Valve valve );
char *ValveErrorDesc( Valve valve );
char *ValveCtrlDesc( ValveCtrlMode mode );

#endif 
