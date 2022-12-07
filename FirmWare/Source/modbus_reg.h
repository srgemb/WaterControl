
#ifndef __MODBUS_REG_H
#define __MODBUS_REG_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//Регистры протокола MODBUS
#define MBUS_REG_CTRL           0x0000  //Состояние датчиков, состояние электроприводов
#define MBUS_REG_WTR_COLD       0x0001  //Расход холодной воды
#define MBUS_REG_COLD_PRESR     0x0003  //Давление холодной воды
#define MBUS_REG_WTR_HOT        0x0004  //Расход горячей воды
#define MBUS_REG_HOT_PRESR      0x0006  //Давление горячей воды
#define MBUS_REG_WTR_FILTER     0x0007  //Расход питьевой воды
#define MBUS_REG_DAYMON         0x0009  //Дата/время (месяц/день)
#define MBUS_REG_YEAR           0x000A  //Дата/время (год)
#define MBUS_REG_HOURMIN        0x000B  //Дата/время (часы/минуты)

//Команды для регистра MBUS_REG_CTRL, протокол MODBUS (только запись)
#define MBUS_CMD_ALL_CLOSE      0x0000  //закрыть все
#define MBUS_CMD_COLD_OPEN      0x0001  //открыть кран холодной воды
#define MBUS_CMD_COLD_CLOSE     0x0002  //закрыть кран холодной воды
#define MBUS_CMD_HOT_OPEN       0x0004  //открыть кран горячей воды
#define MBUS_CMD_HOT_CLOSE      0x0008  //закрыть кран горячей воды
#define MBUS_CMD_ALL_OPEN       0x0010  //открыть все

//Маска для всех команд
#define MBUS_CMD_MASK           ( MBUS_CMD_COLD_OPEN | MBUS_CMD_COLD_CLOSE | \
                                MBUS_CMD_HOT_OPEN | MBUS_CMD_HOT_CLOSE | MBUS_CMD_ALL_OPEN )

//Атрибуты окончания списка
#define REG_END                 0xFFFF  //код окончания списка регистров
#define FUNC_END                0xFF    //код окончания списка функций


#pragma pack( push, 1 )                 //выравнивание структуры по границе 1 байта
                                                                                                             
//Структура для запроса чтения (01,02,03,04)
//Структура для записи одного регистра (05,06) дискретная/16-битная
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t reg_addr;                  //Адрес первого регистра HI/LO байт
    uint16_t reg_cntval;                //Количество регистров/значение для записи HI/LO байт
    uint16_t crc;                       //Контрольная сумма CRC
 } MBUS_REQ_REG;

//Структура для записи значений в несколько регистров (0F,10)
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код
    uint16_t reg_addr;                  //Адрес первого регистра HI/LO байт
    uint16_t reg_cnt;                   //Количество регистров HI/LO байт
    uint8_t  byte_cnt;                  //Количество байт данных регистров
    //далее идут данные и КС
 } MBUS_WRT_REGS;

//Структура ответа на запрос с ошибкой
typedef struct {
    uint8_t  dev_addr;                  //Адрес устройства
    uint8_t  function;                  //Функциональный код с признаком ошибки
    uint8_t  error;                     //Код ошибки
    uint16_t crc;                       //Контрольная сумма CRC
 } MBUS_ERROR;
 
//Структура набора регистров доступных для чтения
typedef struct {
    uint16_t id_reg;
    uint8_t  cnt_reg[8];
 } RegsRead;

//Структура набора регистров для записи и допустимых значений
typedef struct {
    uint16_t id_reg;
    uint16_t min_value;
    uint16_t max_value;
 } ValueValid;

#pragma pack( pop )

#endif
