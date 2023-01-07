
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   192
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       14
    #define FW_TIME_MINUTES    38
    #define FW_TIME_SECONDS    44

    #define FW_DATE_DAY        7
    #define FW_DATE_MONTH      1
    #define FW_DATE_YEAR       2023
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   27
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       14
    #define FW_TIME_MINUTES    38
    #define FW_TIME_SECONDS    50

    #define FW_DATE_DAY        7
    #define FW_DATE_MONTH      1
    #define FW_DATE_YEAR       2023
#endif

#endif

