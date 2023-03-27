
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   255
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       21
    #define FW_TIME_MINUTES    4
    #define FW_TIME_SECONDS    0

    #define FW_DATE_DAY        22
    #define FW_DATE_MONTH      3
    #define FW_DATE_YEAR       2023
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   54
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       20
    #define FW_TIME_MINUTES    58
    #define FW_TIME_SECONDS    26

    #define FW_DATE_DAY        27
    #define FW_DATE_MONTH      3
    #define FW_DATE_YEAR       2023
#endif

#endif

