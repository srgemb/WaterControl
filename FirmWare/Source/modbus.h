
#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

//#define DEBUG_MODBUS                        //вывод отладочной информации для MODBUS

//Тип адресации 8/16-битная
typedef enum {
    MBUS_REG_8BIT,                          //8-битная адресации
    MBUS_REG_16BIT,                         //16-битная адресации
    MBUS_REG_OTHER                          //адресации определяется уст-м
 } ModbusAddrReg;

//Валидация кода функции
typedef enum {
    MBUS_FUNC_ERR,                          //указанный код функции не верный
    MBUS_FUNC_OK                            //проверка прошла
 } ModbusFuncValid;

//Тип запроса master -> slave
typedef enum {
    MBUS_REQST_UNKNOW,                      //Структура не определена
    MBUS_REQST_READ,                        //Структура регистров для запроса чтения (01,02,03,04)
    MBUS_REQST_WRITE1,                      //Структура регистров для записи (05,06) дискретная/16-битная
    MBUS_REQST_WRITEN                       //Структура для записи значений в несколько регистров (0F,10)
 } ModbusRequst;

//*************************************************************************************************
// Коды ответа уст-ва по протоколу MODBUS
//*************************************************************************************************
typedef enum {
    MBUS_REQUEST_OK,                        //Ошибок нет
    //ошибки протокола MODBUS
    MBUS_ERROR_FUNC,                        //Принятый код функции не может быть обработан.
    MBUS_ERROR_ADDR,                        //Адрес данных, указанный в запросе, недоступен.
    MBUS_ERROR_DATA,                        //Значение, содержащееся в поле данных запроса, является недопустимой величиной.
    MBUS_ERROR_DEV,                         //Невосстанавливаемая ошибка имела место, пока ведомое устройство
                                            //пыталось выполнить затребованное действие.
    MBUS_ERROR_ACKWAIT,                     //Ведомое устройство приняло запрос и обрабатывает его, но это требует много времени.
                                            //Этот ответ предохраняет ведущее устройство от генерации ошибки тайм-аута.
    MBUS_ERROR_BUSY,                        //Ведомое устройство занято обработкой команды. Ведущее устройство должно повторить
                                            //сообщение позже, когда ведомое освободится.
    MBUS_ERROR_NOACK,                       //Ведомое устройство не может выполнить программную функцию, заданную в запросе.
                                            //Этот код возвращается для неуспешного программного запроса, использующего
                                            //функции с номерами 13 или 14. Ведущее устройство должно запросить диагностическую
                                            //информацию или информацию об ошибках от ведомого.
    MBUS_ERROR_MEMCRC,                      //Ведомое устройство при чтении расширенной памяти обнаружило ошибку контроля четности.
    //ошибки обмена данными
    MBUS_REQST_CRC,                         //ошибка КС принятого фрейма
    MBUS_REQST_NOT_FOR_DEV,                 //запрос не к этому уст-ву
    //ошибки данных или параметров
    MBUS_ERROR_PARAM                        //ошибки в параметрах вызова функции
} ModBusError;

//пересчет кол-ва бит в кол-во байт
#define CALC_BYTE( bits )                   ( bits%8 ? (bits/8)+1 : bits/8 )

//*************************************************************************************************
// Структура данных запроса MODBUS 
//*************************************************************************************************
#pragma pack( push, 1 )                     //выравнивание структуры по границе 1 байта

typedef struct {
    uint8_t  dev_addr;                      //Адрес устройства
    uint8_t  function;                      //Функциональный код
    uint16_t reg_addr;                      //Адрес первого регистра (HI/LO байт)
    uint16_t reg_cnt;                       //Количество регистров (HI/LO байт)
    uint8_t  *ptr_data;                     //Указатель на данные регистров
} MBUS_REQ;

#pragma pack( pop )

//*************************************************************************************************
// Функции управления
//*************************************************************************************************
void ModBusInit( void );
void ModBusErrClr( void );

//*************************************************************************************************
// Функции статуса/состояния
//*************************************************************************************************
ModBusError CheckRequest( uint8_t *data, uint8_t len, MBUS_REQ *reqst );
uint8_t CreateFrame( MBUS_REQ *reqst, ModBusError error );
void IncError( ModBusError error );
uint32_t ModBusErrCnt( ModBusError error );
char *ModBusErrDesc( ModBusError error );
char *ModBusErrCntDesc( ModBusError error, char *str );

#endif
