#ifndef AIRMANAGER_H
#define AIRMANAGER_H

#include <Arduino.h>
#include <si_message_port.hpp>
#include "Servos.h"

class AirManager {
    public:
        AirManager();
        ~AirManager();

        void loop();

        // Singleton instance
        static AirManager* instance;

    private:
        static void new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload);
        Servos* servos; // Pointer to the Servos class instance

        SiMessagePort* messagePort;
};

#endif // AIRMANAGER_H
