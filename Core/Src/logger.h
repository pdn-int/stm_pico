//#ifndef _LOGGER_
//#define _LOGGER_
//
//#include <stdarg.h>
//#include <stdio.h>
//
//#define DEBUG "DEBUG"
//#define INFO1 "INFO"
//#define WARN "WARN"
//#define ERROR1 "ERROR"
//#define TRACE "TRACE"
//#define DATA "DATA"
//#define CRIT "CRIT"
//
//#define LOGLEVEL_TRACE  3
//#define LOGLEVEL_DEBUG  2
//#define LOGLEVEL_INFO 1
//#define LOGLEVEL_ERRORWARN 0
//#define LOGLEVEL_CRIT 0
//
//int log_init(char* dirName, char* fileName,int maxFileLines,int rotateFile);
//void log_trace(const char *caller, const char *message, ...);
//void log_debug(const char * caller, const char *format, ...);
//void log_error(const char * caller, const char *format, ...);
//void log_info(const char * caller, const char *format, ...);
//void log_data(const char * caller, const char *format, ...);
//void log_crit(const char * caller, const char *format, ...);
//void log_write_vargs(const char *errorLevel, const char* caller, const char *format, va_list ap);
//
////
////  Log macros with line number
////
////
////#define L_TCP(...) \
////    if (ic3.logsLevel >= LOGLEVEL_ERRORWARN) { \
////        log_tcp(__func__, __VA_ARGS__); \
////    }
////
////#define L_CRIT(...) \
////    if (ic3.logsLevel >= LOGLEVEL_CRIT) { \
////        log_crit(__func__, __VA_ARGS__); \
////    }
////
////#define L_ERROR(...) \
////    if (ic3.logsLevel >= LOGLEVEL_ERRORWARN) { \
////        log_error(__func__, __VA_ARGS__); \
////    }
////
////#define L_INFO(...) \
////    if (ic3.logsLevel >= LOGLEVEL_INFO) { \
////        log_info(__func__, __VA_ARGS__); \
////    }
////
////#define L_DEBUG(...) \
////    if (ic3.logsLevel >= LOGLEVEL_DEBUG) { \
////        log_debug(__func__, __VA_ARGS__); \
////    }
////
////#define L_TRACE(...) \
////    if (ic3.logsLevel >= LOGLEVEL_TRACE) { \
////        log_trace(__func__, __VA_ARGS__); \
////    }
////#endif
