#ifndef DEBUGLOG_H
#define DEBUGLOG_H
#include "Configuration.h"
#define DEBUGLOG_ENABLE 1

#if DEBUGLOG_ENABLE
#if BENCHDEBUG
#define DEBUGLOG_INIT(x) Serial.begin(x)
#define DEBUGLOG_PRINT(x) Serial.print(x)
#define DEBUGLOG_PRINTLN(x) Serial.println(x)
#else
#define DEBUGLOG_INIT(x) Serial1.begin(x)
#define DEBUGLOG_PRINT(x) Serial1.print(x)
#define DEBUGLOG_PRINTLN(x) Serial1.println(x)
#endif
#else
#define DEBUGLOG_INIT(x)
#define DEBUGLOG_PRINT(x)
#define DEBUGLOG_PRINTLN(x)
#endif

#endif // DEBUGLOG_H