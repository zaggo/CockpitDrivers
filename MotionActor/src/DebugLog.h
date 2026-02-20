#ifndef DEBUGLOG_H
#define DEBUGLOG_H

#define DEBUGLOG_ENABLE 0

// Prozessor erkennen
#if defined(__AVR_ATmega328P__)
    #include <SoftwareSerial.h>
    static SoftwareSerial DebugSoftSerial(8, 9); // RX, TX 
    #define DEBUGLOG_PORT DebugSoftSerial
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    #define DEBUGLOG_PORT Serial1
#else
    #define DEBUGLOG_PORT Serial
#endif

#if DEBUGLOG_ENABLE
    #define DEBUGLOG_INIT(x)    DEBUGLOG_PORT.begin(x)
    #define DEBUGLOG_PRINT(x)   DEBUGLOG_PORT.print(x)
    #define DEBUGLOG_PRINTLN(x) DEBUGLOG_PORT.println(x)
#else
    #define DEBUGLOG_INIT(x)
    #define DEBUGLOG_PRINT(x)
    #define DEBUGLOG_PRINTLN(x)
#endif

#endif // DEBUGLOG_H