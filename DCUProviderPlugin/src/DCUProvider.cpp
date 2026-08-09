#include "DCUProvider.h"
#include "XPLMUtilities.h"
#include "ConfigUtils.h"
#include <cstdio>
#include <ctime>
#include <SerialMessageId.h>

DCUProvider::DCUProvider() = default;

DCUProvider::~DCUProvider()
{
    shutdown();
}

bool DCUProvider::initialize()
{
    // Initialize components
    dataRefMgr_ = std::make_unique<DataRefManager>();
    dataRefMgr_->initialize();

    msgQueue_ = std::make_unique<MessageQueue>();

    // Ports ermitteln und letzten Port laden
    auto ports = enumerateSerialPorts();
    std::string lastPort = loadLastUsedPort();
    currentPort_.clear();
    // Nur vorwählen, nicht verbinden
    int preselectIdx = -1;
    if (!lastPort.empty())
    {
        for (size_t i = 0; i < ports.size(); ++i)
        {
            if (ports[i] == lastPort)
            {
                preselectIdx = (int)i;
                currentPort_ = lastPort;
                break;
            }
        }
    }

    connMgr_.reset();
    // Nur verbinden, wenn ein Port gesetzt ist
    if (!currentPort_.empty())
    {
        connMgr_ = std::make_unique<ConnectionManager>(currentPort_, 115200, *msgQueue_);
        connMgr_->connect();
    }

    statusWin_ = std::make_unique<StatusWindow>();
    statusWin_->initialize();
    statusWin_->setAvailablePorts(ports);
    if (preselectIdx >= 0)
        statusWin_->selectedPortIdx_ = preselectIdx;
    statusWin_->setPortChangedCallback([this](const std::string &port)
                                       { changePort(port); });
    statusWin_->setWindowShownCallback([this]()
                                       { refreshPorts(); });

    char msg[256];
    std::snprintf(msg, sizeof(msg),
                  "DCUProvider: Initialized (Connection: %s)\n",
                  (connMgr_ && connMgr_->isConnected()) ? "OK" : "FAILED");
    XPLMDebugString(msg);

    return true; // Even if initial connection fails, plugin loads
}

void DCUProvider::refreshPorts()
{
    if (!statusWin_)
        return;
    auto ports = enumerateSerialPorts();
    int selectIdx = -1;
    if (!currentPort_.empty())
    {
        for (size_t i = 0; i < ports.size(); ++i)
        {
            if (ports[i] == currentPort_)
            {
                selectIdx = (int)i;
                break;
            }
        }
    }
    statusWin_->setAvailablePorts(ports);
    if (selectIdx >= 0)
        statusWin_->selectedPortIdx_ = selectIdx;
}

void DCUProvider::changePort(const std::string &newPort)
{
    saveLastUsedPort(newPort);
    if (newPort == currentPort_)
        return;
    currentPort_ = newPort;
    if (connMgr_)
    {
        connMgr_->disconnect();
        connMgr_.reset();
    }
    // Release the axes here too: with no connMgr_, onFlightLoopTick() bails out
    // before the watchdog runs, so it could never hand them back itself.
    rudderSilenceAccumulator_ = RUDDER_SIGNAL_TIMEOUT;
    if (dataRefMgr_)
    {
        dataRefMgr_->setRudderOverrideEnabled(false);
    }
    // Queue leeren
    if (msgQueue_)
    {
        msgQueue_->resetStats(); // setzt auch Zähler zurück
        msgQueue_->clearTxQueue();
        while (msgQueue_->hasRxPending())
            msgQueue_->dequeueRx();
    }
    if (!currentPort_.empty())
    {
        connMgr_ = std::make_unique<ConnectionManager>(currentPort_, 115200, *msgQueue_);
        connMgr_->connect();
        // Reset-settle delay now lives in ConnectionManager::ioThreadLoop()
        // (kPostConnectSettleMs) - runs on the I/O thread, not here, so this
        // no longer blocks the flight loop.
    }
    // Optionally, update status window immediately
    updateStatusWindow();
}

void DCUProvider::shutdown()
{
    XPLMDebugString("DCUProvider: Shutting down\n");

    // Hand the rudder/toe-brake axes back before tearing down, otherwise X-Plane
    // keeps the override flags set with nobody left to write the values.
    if (dataRefMgr_)
    {
        dataRefMgr_->setRudderOverrideEnabled(false);
    }

    if (statusWin_)
    {
        statusWin_->destroy();
        statusWin_.reset();
    }

    if (connMgr_)
    {
        connMgr_->disconnect();
        connMgr_.reset();
    }

    msgQueue_.reset();
    dataRefMgr_.reset();
}

void DCUProvider::onAircraftLoaded()
{
    if (dataRefMgr_) {
        dataRefMgr_->onAircraftLoaded();
    }
}

void DCUProvider::onFlightLoopTick(float elapsedTime)
{
    if (!connMgr_ || !msgQueue_ || !dataRefMgr_)
    {
        return;
    }

    // ============ Connection Management ============
    connMgr_->update(elapsedTime);

    // ============ I/O Processing ============
    // Handled continuously by ConnectionManager's own I/O thread now - not
    // ticked from here, so USB/serial driver overhead can't eat into the
    // render frame budget. This thread only reads/writes the (mutex-guarded)
    // MessageQueue via updateDownlink()/updateUplink() below.

    // ============ Downlink: X-Plane → Gateway ============
    updateDownlink(elapsedTime);

    // ============ Uplink: Gateway → X-Plane ============
    updateUplink();

    // ============ Rudder Override Watchdog ============
    updateRudderOverride(elapsedTime);

    // ============ Status Window Update (1Hz) ============
    static float statusUpdateAccum = 0.0f;
    statusUpdateAccum += elapsedTime;

    if (statusUpdateAccum >= 1.0f)
    {
        updateStatusWindow();
        statusUpdateAccum = 0.0f;
    }
}

bool DCUProvider::isConnected() const
{
    return connMgr_ && connMgr_->isConnected();
}

void DCUProvider::updateDownlink(float dt)
{
    if (!isConnected())
    {
        return;
    }

    // ============ Fuel Data (5 Hz) ============
    fuelAccumulator_ += dt;
    float fuelRate = 1.0f / 5.0f; // 5 Hz = every 0.2s

    if (fuelAccumulator_ >= fuelRate)
    {
        struct FuelData
        {
            float fuelLeft;
            float fuelRight;
        };

        // Read fuel from X-Plane
        FuelData fuel;
        fuel.fuelLeft = dataRefMgr_->getFuelLeft();
        fuel.fuelRight = dataRefMgr_->getFuelRight();

        msgQueue_->enqueueTx(MessageType::SerialMessageFuel, &fuel, sizeof(fuel));

        fuelAccumulator_ = 0.0f;
    }

    // ============ Lights Data (10 Hz) ============
    lightsAccumulator_ += dt;
    float lightsRate = 1.0f / LIGHTS_RATE; // 10 Hz = every 0.1s

    if (lightsAccumulator_ >= lightsRate)
    {
        struct LightsData
        {
            float panelDim;     // 0..1
            float radioDim;     // 0..1
            float domeLightDim; // 0..1
        };

        LightsData lights;
        auto brightness = dataRefMgr_->getPanelBrightness();
        lights.panelDim = brightness[0];
        lights.radioDim = brightness[1];
        lights.domeLightDim = dataRefMgr_->getDomeLightBrightness();

        msgQueue_->enqueueTx(MessageType::SerialMessageLights, &lights, sizeof(lights));

        lightsAccumulator_ = 0.0f;
    }

    // ============ Transponder Data (10 Hz) ============
    transponderAccumulator_ += dt;
    float transponderRate = 1.0f / TRANSPONDER_RATE; // 10 Hz = every 0.1s

    if (transponderAccumulator_ >= transponderRate)
    {
        struct TransponderData
        {
            uint16_t code;
            uint8_t mode;
            uint8_t light;
        };

        TransponderData transponder;
        transponder.code = static_cast<uint16_t>(dataRefMgr_->getTransponderCode());
        transponder.mode = static_cast<uint8_t>(dataRefMgr_->getTransponderMode());
        transponder.light = dataRefMgr_->getTransponderLight() > 0 ? true : false;

        msgQueue_->enqueueTx(MessageType::SerialMessageTransponder, &transponder, sizeof(transponder));

        transponderAccumulator_ = 0.0f;
    }

    // ============ RPM Data (50 Hz) ============
    rpmAccumulator_ += dt;
    float rpmRate = 1.0f / RPM_RATE;

    if (rpmAccumulator_ >= rpmRate)
    {
        struct RpmData
        {
            float rpm;
        };

        RpmData rpm;
        rpm.rpm = dataRefMgr_->getRpm();

        msgQueue_->enqueueTx(MessageType::SerialMessageRPM, &rpm, sizeof(rpm));

        rpmAccumulator_ = 0.0f;
    }

    // ============ Odometer Data (10 Hz) ============
    odometerAccumulator_ += dt;
    float odometerRate = 1.0f / ODOMETER_RATE;

    if (odometerAccumulator_ >= odometerRate)
    {
        struct OdometerData
        {
            int8_t hrs1000;
            int8_t hrs100;
            int8_t hrs10;
            int8_t hrs1;
            float hrsTenths;
            float hrsHundredths;
        };

        OdometerData odometer;
        odometer.hrs1000 = dataRefMgr_->getTachHours1000();
        odometer.hrs100 = dataRefMgr_->getTachHours100();
        odometer.hrs10 = dataRefMgr_->getTachHours10();
        odometer.hrs1 = dataRefMgr_->getTachHours1();
        odometer.hrsTenths = dataRefMgr_->getTachHoursTenths();
        odometer.hrsHundredths = dataRefMgr_->getTachHoursHundredths();

        msgQueue_->enqueueTx(MessageType::SerialMessageOdometer, &odometer, sizeof(odometer));

        odometerAccumulator_ = 0.0f;
    }

    // TODO: Add more downlink data based on CAN Message IDs
    // - Altimeter settings
    // - Heading bug
    // - Course selector
    // - etc.
}

void DCUProvider::updateUplink()
{
    if (!msgQueue_)
    {
        return;
    }

    // Process all received messages
    while (msgQueue_->hasRxPending())
    {
        auto msg = msgQueue_->dequeueRx();

        if (!msg)
        {
            continue;
        }

        // Route message by type
        switch (msg->type)
        {
        case MessageType::SerialMessageLights:
            // Gateway → Plugin: This would be for reading light switches from device
            // (not typical for Piper Arrow, but possible)
            break;

        case MessageType::SerialMessageTransponder:
            // Gateway → Plugin: This would be for reading transponder controls from device
            if (msg->payload.size() >= sizeof(TransponderToDcuMessage))
            {
                const TransponderToDcuMessage *message = reinterpret_cast<const TransponderToDcuMessage *>(msg->payload.data());
                if (static_cast<uint8_t>(message->command) & static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandSetCode))
                {
                    dataRefMgr_->setTransponderCode(message->code);
                }
                if (static_cast<uint8_t>(message->command) & static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandSetMode))
                {
                    dataRefMgr_->setTransponderMode(message->mode);
                }
                if (static_cast<uint8_t>(message->command) & static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandIdent))
                {
                    dataRefMgr_->transponderIdentOnce();
                }
            }
            break;

        case MessageType::SerialMessageHandbrake:
            // Gateway → Plugin: This would be for reading handbrake controls from device
            if (msg->payload.size() >= sizeof(uint8_t))
            {
                uint8_t breakStatus = *reinterpret_cast<const uint8_t *>(msg->payload.data());
                dataRefMgr_->setParkingBrakeRatio(static_cast<float>(breakStatus) / 100.0f);
            }
            break;

        case MessageType::SerialMessageRudder:
            // Gateway → Plugin: rudder pedal axis + toe brakes from the RudderCAN node
            if (msg->payload.size() >= sizeof(RudderToDcuMessage))
            {
                const RudderToDcuMessage *message = reinterpret_cast<const RudderToDcuMessage *>(msg->payload.data());

                // Claim the axes before writing, otherwise this frame's values are
                // still the ones X-Plane discards.
                rudderSilenceAccumulator_ = 0.0f;
                dataRefMgr_->setRudderOverrideEnabled(true);

                dataRefMgr_->setRudderRatio(static_cast<float>(message->rudder) / 1000.0f);
                dataRefMgr_->setLeftBrakeRatio(static_cast<float>(message->leftBrake) / 1000.0f);
                dataRefMgr_->setRightBrakeRatio(static_cast<float>(message->rightBrake) / 1000.0f);
            }
            break;

        default:
            // Unknown message type
            break;
        }
    }
}

void DCUProvider::updateRudderOverride(float dt)
{
    if (!dataRefMgr_)
    {
        return;
    }

    if (rudderSilenceAccumulator_ < RUDDER_SIGNAL_TIMEOUT)
    {
        rudderSilenceAccumulator_ += dt;
    }

    // Give the axes straight back if the hardware goes quiet or the link drops,
    // so a dead DCU never costs the user their rudder.
    const bool active = isConnected() && rudderSilenceAccumulator_ < RUDDER_SIGNAL_TIMEOUT;
    dataRefMgr_->setRudderOverrideEnabled(active);
}

void DCUProvider::updateStatusWindow()
{
    if (!statusWin_)
    {
        return;
    }

    StatusData data;
    data.isConnected = isConnected();
    data.devicePath = currentPort_;
    data.baudRate = 115200;
    data.txBytesSent = msgQueue_ ? msgQueue_->getTxBytesSent() : 0;
    data.rxBytesReceived = msgQueue_ ? msgQueue_->getRxBytesReceived() : 0;
    data.lastTxTime = connMgr_ ? connMgr_->getLastTxTime() : 0.0f;
    data.lastRxTime = connMgr_ ? connMgr_->getLastRxTime() : 0.0f;
    data.lastWriteOk = connMgr_ ? connMgr_->getLastWriteOk() : false;
    data.lastOpenOk = connMgr_ ? connMgr_->getLastOpenOk() : false;

    statusWin_->update(data);
}