#include "DebugLog.h"

#if defined(__AVR_ATmega328P__)
SoftwareSerial DebugSoftSerial(kDebugLogRxPin, kDebugLogTxPin);
#endif
