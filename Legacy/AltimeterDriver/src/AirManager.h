#ifndef AIRMANAGER_H
#define AIRMANAGER_H

#include <Arduino.h>
#include "Configuration.h"
#include <si_message_port.hpp>
#include "Altimeter.h"

class AirManager {
    public:
        AirManager();
        ~AirManager();

        void loop();

        // Singleton instance
        static AirManager* instance;

    private:
        static void new_message_callback(uint16_t message_id, struct SiMessagePortPayload* payload);

        SiMessagePort* messagePort;
        Altimeter* altimeter;
        float currentPressure = 0.0f;
};

#endif // AIRMANAGER_H