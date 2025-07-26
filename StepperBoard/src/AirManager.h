#ifndef AIRMANAGER_H
#define AIRMANAGER_H

#include <Arduino.h>
#include "GyroDrive.h"
#include "Configuration.h"
#include "X25Motors.h"
#include <si_message_port.hpp>

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
        GyroDrive* gyroDrive;
        X25Motors* x25Motors;
};

#endif // AIRMANAGER_H
