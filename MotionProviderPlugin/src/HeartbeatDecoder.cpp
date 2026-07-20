#include "HeartbeatDecoder.h"

void HeartbeatDecoder::reset() {
    state_ = S::SyncH;
    pending_ = 0;
}

bool HeartbeatDecoder::feed(uint8_t b) {
    switch (state_) {
        case S::SyncH:
            state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
        case S::SyncB:
            if (b == 'B')      state_ = S::Payload;
            else               state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
        case S::Payload:
            pending_ = b;
            state_ = S::CR;
            return false;
        case S::CR:
            if (b == 0x0D) {
                armed_ = (pending_ != 0);
                state_ = S::SyncH;
                return true;
            }
            state_ = (b == 'H') ? S::SyncB : S::SyncH;
            return false;
    }
    return false;
}
