
#ifndef  __INCLUDE_HD_RECON_LOGGER_H_
#define  __INCLUDE_HD_RECON_LOGGER_H_

#include <stdio.h>

#ifdef TARGET_ANDROID
#include <android/log.h>
#endif

#define HD_LOG_DEBUG   0
#define HD_LOG_NOTICE  1
#define HD_LOG_TRACE   2
#define HD_LOG_WARNING 3
#define HD_LOG_FATAL   4

#ifdef OUT_RELEASE
#define HD_LOG_LEVEL 5 
#else 
#define HD_LOG_LEVEL 0
#endif

#ifdef TARGET_ANDROID

#define HD_LOG(type, fmt, ...) do { \
    if (type >= HD_LOG_LEVEL) { \
        __android_log_print(ANDROID_LOG_ERROR, "time_huyh", "[%s:%s:%s:%d ]" fmt, __DATE__, __TIME__, __FILE__, __LINE__, ##__VA_ARGS__); \
    }  \
} while(0)

#else

#define HD_LOG(type, fmt, ...) do { \
    if (type >= HD_LOG_LEVEL) { \
        printf("[%s:%s:%s:%d]" fmt, __DATE__, __TIME__, __FILE__, __LINE__, ##__VA_ARGS__); \
    } \
} while(0)

#endif

#endif  // __INCLUDE_HD_RECON_LOGGER_H_

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
