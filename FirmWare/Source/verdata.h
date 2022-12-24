
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   186
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       11
    #define FW_TIME_MINUTES    47
    #define FW_TIME_SECONDS    11

    #define FW_DATE_DAY        24
    #define FW_DATE_MONTH      12
    #define FW_DATE_YEAR       2022
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   23
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       11
    #define FW_TIME_MINUTES    47
    #define FW_TIME_SECONDS    6

    #define FW_DATE_DAY        24
    #define FW_DATE_MONTH      12
    #define FW_DATE_YEAR       2022
#endif

#endif

