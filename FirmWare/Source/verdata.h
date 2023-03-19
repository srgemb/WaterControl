
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   236
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       21
    #define FW_TIME_MINUTES    32
    #define FW_TIME_SECONDS    16

    #define FW_DATE_DAY        12
    #define FW_DATE_MONTH      3
    #define FW_DATE_YEAR       2023
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   47
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       20
    #define FW_TIME_MINUTES    6
    #define FW_TIME_SECONDS    57

    #define FW_DATE_DAY        19
    #define FW_DATE_MONTH      3
    #define FW_DATE_YEAR       2023
#endif

#endif

