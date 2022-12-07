
#ifndef __VERDATA_H
#define __VERDATA_H

#ifdef DEBUG_TARGET
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   141
    #define FW_VERSION_RC      'D'

    #define FW_TIME_HOUR       23
    #define FW_TIME_MINUTES    22
    #define FW_TIME_SECONDS    48

    #define FW_DATE_DAY        5
    #define FW_DATE_MONTH      11
    #define FW_DATE_YEAR       2022
#else
    #define FW_VERSION_MAJOR   1
    #define FW_VERSION_MINOR   0
    #define FW_VERSION_BUILD   9
    #define FW_VERSION_RC      'R'

    #define FW_TIME_HOUR       23
    #define FW_TIME_MINUTES    22
    #define FW_TIME_SECONDS    58

    #define FW_DATE_DAY        5
    #define FW_DATE_MONTH      11
    #define FW_DATE_YEAR       2022
#endif

#endif

