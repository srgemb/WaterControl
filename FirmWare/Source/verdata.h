
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   235
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       21
    #define FW_TIME_MINUTES    18
    #define FW_TIME_SECONDS    43

    #define FW_DATE_DAY        12
    #define FW_DATE_MONTH      3
    #define FW_DATE_YEAR       2023
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   39
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       23
    #define FW_TIME_MINUTES    9
    #define FW_TIME_SECONDS    29

    #define FW_DATE_DAY        15
    #define FW_DATE_MONTH      2
    #define FW_DATE_YEAR       2023
#endif

#endif

