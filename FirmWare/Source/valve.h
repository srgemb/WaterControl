
#ifndef __VALVE_H
#define __VALVE_H

#include <stdbool.h>
#include <stdio.h>

#include "main.h"

//#define DEBUG_VALVE                         //вывод отладочных событий

//тип электропривода
typedef enum {
    VALVE_COLD,                             //электропривод крана холодной воды
    VALVE_HOT                               //электропривод крана горячей воды
 } Valve;

//команды управления приводом
typedef enum {
    VALVE_CTRL_STOP,                        //остановить привод
    VALVE_CTRL_CLOSE,                       //закрыть кран
    VALVE_CTRL_OPEN                         //открыть кран
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
    VALVE_NOPWR,                            //нет нагрузки в цепи
    VALVE_TIMEOUT                           //превышение длительности работы 
                                            //нет подтверждения завершения работы привода
 } ValveError;

#pragma pack( push, 1 )

typedef struct {
    ValveError error_cold : 4;              //ошибка электропривода холодной воды
    ValveError error_hot  : 4;              //ошибка электропривода горячей воды
 } VALVE;

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
