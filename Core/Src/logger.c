//
//#include <string.h>
//#include <stdarg.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <errno.h>
//#include <limits.h>
//#include <signal.h>
//#include "logger.h"
//#include "daemon.h"
//#define LOGGER_BUFFER_SIZE 2048
//#define DEBUG_LOGGING 0
//
//// #if !defined(STM32_UMOTE2) && !defined(picoSTM32H753) && !defined(PICOSTM_COMBO) && !defined(PICOSTM_COMBO_REV_B)
////     #include <fcntl.h>
////     #include <unistd.h>
////     #include <sys/types.h>
////     #include <sys/stat.h>
////     #include <sys/time.h>
////     #include <pthread.h>
////     // Note we need larger buffers in the Simulator since the logs names
////     // are like this:  /application/logs/ic3.log
////     #define LOG_FILE_NAME_SIZE 128
////     #define LOG_FILE_LIM 60
////     char filelim[LOG_FILE_LIM];
////     char dirn[LOG_FILE_LIM];
//// #else
////     #include "cmsis_os2.h"     // ::CMSIS:RTOS2
////     #include "stm32h7xx_hal.h" // Keil::Device:STM32Cube HAL:Common
////     #include "rl_fs.h"         // Keil.MDK-Pro::File System:CORE
//     #define LOG_FILE_NAME_SIZE 24
//// #endif
//
//// How many files to rotate.
//#define NUMBER_OF_LOG_FILES_TO_ROTATE 10
//
//// Base name for the log files
//static char logBaseFileName[LOG_FILE_NAME_SIZE];
//static char logBaseDirName[LOG_FILE_NAME_SIZE];
//
//// Base name for the data log files
//static char datalogBaseFileName[LOG_FILE_NAME_SIZE];
//static char datalogBaseDirName[LOG_FILE_NAME_SIZE];
//
//// Name of the log files currently open
//static char logFileName[LOG_FILE_NAME_SIZE];
//static char datalogFileName[LOG_FILE_NAME_SIZE];
//
//static int logLineMax = 0;        // Max number of log lines before rotating the file
//static int logLineCount = 0;      // Current number of log lines
//static int dataLogLineCount = 0;  // Current number of log lines
//static int rotateLogFile = 0;     // Flag, if true enable log rotation
//
//// Current open files
//static FILE *LOG_FILE = NULL;
//static FILE *DATA_LOG_FILE = NULL;
//
//// Create all the log functions for every level
//#define REGISTER_LOG_FUNCTION(func_name, loglevel_str, loglevel) \
//    void func_name(const char *caller, const char *message, ...) { \
//        if (ic3.logsLevel < loglevel) { return; } \
//        va_list args; \
//        va_start(args, message); \
//        log_write_vargs(loglevel_str, caller, message, args); \
//        va_end(args); \
//    }
//REGISTER_LOG_FUNCTION(log_trace, TRACE, LOGLEVEL_TRACE);
//REGISTER_LOG_FUNCTION(log_debug, DEBUG, LOGLEVEL_DEBUG);
//REGISTER_LOG_FUNCTION(log_error, ERROR1, LOGLEVEL_ERRORWARN);
//REGISTER_LOG_FUNCTION(log_info, INFO1, LOGLEVEL_INFO);
//REGISTER_LOG_FUNCTION(log_crit, CRIT, LOGLEVEL_CRIT);
//REGISTER_LOG_FUNCTION(log_tcp, "TCP", -1);
//
//// special case
//void log_data(const char *caller, const char *message, ...) {
//    if(!ic3.logsDataEnabled){
//        return;
//    }
//    va_list args;
//    va_start(args, message);
//    log_write_vargs(DATA, caller, message, args);
//    va_end(args);
//}
//
//#define LOG_EVERY_N_LINES 10
//#define TIMESTRING_SIZE 32
//void log_write_vargs(const char *errorLevel, const char *caller,
//                     const char *format, va_list ap) {
//    int pos = 0;
//    time_t currentTime;
//    struct tm *tmTime;
//    char timeString[TIMESTRING_SIZE];
//    char log_buffer[LOGGER_BUFFER_SIZE];
//
//    LOG_LOCK();
//
//    // Init time
//    currentTime = time(NULL);
//    tmTime = GET_CURRENT_TIME(&currentTime);
//
//    // Define log timestamp
//    memset(timeString, 0, TIMESTRING_SIZE);
//    snprintf(timeString, TIMESTRING_SIZE, "%d-%02d-%02d %02d:%02d:%02d",
//            1900 + tmTime->tm_year, tmTime->tm_mon + 1, tmTime->tm_mday,
//            tmTime->tm_hour, tmTime->tm_min, tmTime->tm_sec);
//
//    // Build the log line
//    memset(log_buffer, 0, LOGGER_BUFFER_SIZE);
//    pos = snprintf(log_buffer, LOGGER_BUFFER_SIZE-1, "%s|%s|%s|[%s %s] ",
//                    timeString, ic3.uid_name, FIRMWARE_VERSION, errorLevel, caller);
//    if (pos > 0 && LOGGER_BUFFER_SIZE-1-pos > 0) {
//        vsnprintf(log_buffer+pos, LOGGER_BUFFER_SIZE-1-pos, format, ap);
//    }
//
//    // Always write log line to stdout
//    if (strcmp(errorLevel, DATA) != 0) {
//        printf("%s\n", log_buffer);
//    }
//
//    LOG_UNLOCK();
//}
//
