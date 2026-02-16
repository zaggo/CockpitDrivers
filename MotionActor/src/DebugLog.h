#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#define DEBUGLOG_ENABLE 0

#if DEBUGLOG_ENABLE
#define DEBUGLOG_INIT(x) Serial1.begin(x)
#define DEBUGLOG_PRINT(x) Serial1.print(x)
#define DEBUGLOG_PRINTLN(x) Serial1.println(x)
#else
#define DEBUGLOG_INIT(x)
#define DEBUGLOG_PRINT(x)
#define DEBUGLOG_PRINTLN(x)
#endif

#endif // DEBUGLOG_H