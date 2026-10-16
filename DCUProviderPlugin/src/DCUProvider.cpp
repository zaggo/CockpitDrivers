#include "DCUProvider.h"
#include "XPLMUtilities.h"
#include "ConfigUtils.h"
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cmath>
#include <SerialMessageId.h>
#include <HsiWire.h>

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
    statusWin_->setCommandCallback([this](int action)
                                   { onUiAction(action); });

    rudderCurve_ = loadAxisCurve("rudder");
    curveWin_ = std::make_unique<CurveWindow>("Rudder", &rudderCurve_);
    curveWin_->initialize();
    curveWin_->setChangedCallback([this]()
                                  { saveAxisCurve("rudder", rudderCurve_); });
    statusWin_->appendMenuItem("Rudder Curve...", [this]()
                               { if (curveWin_) curveWin_->setVisible(!curveWin_->isVisible()); });

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
        // The port we are talking to is present by definition. Never let a
        // rescan hide it - setAvailablePorts() would otherwise clamp the stale
        // index to 0 and mark whatever port happens to be first as selected.
        if (selectIdx < 0)
        {
            ports.push_back(currentPort_);
            selectIdx = (int)ports.size() - 1;
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

void DCUProvider::disconnectPort()
{
    XPLMDebugString("DCUProvider: Manual disconnect\n");
    if (connMgr_)
    {
        connMgr_->disconnect();
        connMgr_.reset();
    }
    // Release the axes here too: with no connMgr_, the rudder watchdog's
    // isConnected() is false anyway, but hand them back immediately rather
    // than waiting for the next tick.
    rudderSilenceAccumulator_ = RUDDER_SIGNAL_TIMEOUT;
    if (dataRefMgr_)
    {
        dataRefMgr_->setRudderOverrideEnabled(false);
    }
    if (msgQueue_)
    {
        msgQueue_->resetStats();
        msgQueue_->clearTxQueue();
        while (msgQueue_->hasRxPending())
            msgQueue_->dequeueRx();
    }
    // Clear currentPort_ so re-selecting the same port passes changePort()'s
    // same-port early-return and actually reconnects. The saved last-used
    // port stays untouched - disconnect is session-local.
    currentPort_.clear();
}

void DCUProvider::onUiAction(int action)
{
    switch (action)
    {
    case UI_RESCAN_PORTS:
        refreshPorts();
        rescanFlashRemaining_ = 2.0f;
        break;
    case UI_DISCONNECT:
        disconnectPort();
        break;
    default:
        break;
    }
    // Refresh immediately so the click has instant visual feedback.
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

    if (curveWin_)
    {
        curveWin_->destroy();
        curveWin_.reset();
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
    if (!msgQueue_ || !dataRefMgr_)
    {
        return;
    }

    // ============ Connection Management ============
    // connMgr_ is null before the first port selection and after a manual
    // disconnect - the status window (below) must keep updating regardless.
    if (connMgr_)
    {
        connMgr_->update(elapsedTime);
    }

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

    // ============ Rescan Confirmation Flash ============
    if (rescanFlashRemaining_ > 0.0f)
    {
        rescanFlashRemaining_ -= elapsedTime;
        if (rescanFlashRemaining_ < 0.0f)
            rescanFlashRemaining_ = 0.0f;
    }

    // ============ Status Window Update (1Hz) ============
    static float statusUpdateAccum = 0.0f;
    statusUpdateAccum += elapsedTime;

    if (statusUpdateAccum >= 1.0f)
    {
        updateStatusWindow();
        if (curveWin_)
            curveWin_->update();
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

    // ============ Bus Power / Brownout ============
    // One central reading drives the whole dashboard: lights dim and finally
    // go dark as the bus sags, and below ~10V the bus-powered instruments get
    // their "no power" positions sent instead of live values. Vacuum
    // (attitude), pitot-static (ASI, altimeter, VSI), magnetic (compass) and
    // mechanical (tach/odometer) instruments are exempt by design and keep
    // receiving live data. Power-up simply resumes the live values - no homing
    // is triggered; homing stays a board-power-up affair.
    busPower_.update(dataRefMgr_->getBusVolts());
    const float busLightFactor = busPower_.lightFactor();
    const bool busPowered = busPower_.instrumentsPowered();
    if (busPowered != lastLoggedInstrumentsPowered_)
    {
        lastLoggedInstrumentsPowered_ = busPowered;
        XPLMDebugString(busPowered
                            ? "DCUProvider: Bus power restored - instruments live, lighting unbrowned\n"
                            : "DCUProvider: Bus power lost - instruments to no-power positions, lighting browned out\n");
    }

    // ============ Vacuum Gyro (attitude indicator) ============
    // The sim's _vacuum datarefs don't model suction loss (verified on the
    // rig), so the horizon's degradation is simulated centrally here: slow
    // wander as the rotor droops, wobble mid spin-down, rest pose when dead.
    attitudeGyro_.update(dataRefMgr_->getVacuumInHg(), dt);
    const bool gyroDead = attitudeGyro_.rotorSpeed() < AttitudeGyro::kRestFullSpeed;
    if (gyroDead != lastLoggedGyroDead_)
    {
        lastLoggedGyroDead_ = gyroDead;
        XPLMDebugString(gyroDead
                            ? "DCUProvider: Vacuum gyro spun down - attitude indicator at rest pose\n"
                            : "DCUProvider: Vacuum gyro spun up - attitude indicator live\n");
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

        // The fuel gauge is electric: a dead bus drops its needle to empty.
        if (!busPowered)
        {
            fuel.fuelLeft = 0.0f;
            fuel.fuelRight = 0.0f;
        }

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

        // Brownout: all lighting follows the bus voltage. Panel/radio dim
        // progressively; the dome light is on/off on the wire, so it snaps
        // dark only when the fade reaches zero.
        lights.panelDim *= busLightFactor;
        lights.radioDim *= busLightFactor;
        lights.domeLightDim *= busLightFactor;

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

        // The transponder head is electric: a dead bus blanks the display
        // (mode off, reply light out). The code is kept so power-up restores
        // the exact pre-loss state.
        if (!busPowered)
        {
            transponder.mode = 0;
            transponder.light = 0;
        }

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

    // ============ Airspeed Data (50 Hz) ============
    airspeedAccumulator_ += dt;
    float airspeedRate = 1.0f / AIRSPEED_RATE;

    if (airspeedAccumulator_ >= airspeedRate)
    {
        struct AirspeedData
        {
            float ias;
            float tas;
        };

        AirspeedData airspeed;
        airspeed.ias = dataRefMgr_->getIas();
        airspeed.tas = dataRefMgr_->getTas();

        msgQueue_->enqueueTx(MessageType::SerialMessageAirspeed, &airspeed, sizeof(airspeed));

        airspeedAccumulator_ = 0.0f;
    }

    // ============ Altimeter + VSI Data (50 Hz) ============
    altimeterVsiAccumulator_ += dt;
    float altimeterVsiRate = 1.0f / ALTIMETER_VSI_RATE;

    if (altimeterVsiAccumulator_ >= altimeterVsiRate)
    {
        // Both halves travel together: the gateway packs them into the single CAN
        // frame 0x102 that feeds VerticalSpeedCAN and the altimeter board.
        struct AltimeterVsiData
        {
            float altitudeFt;
            float vsiFpm;
        };

        AltimeterVsiData altimeterVsi;
        altimeterVsi.altitudeFt = dataRefMgr_->getAltitudeFt();
        altimeterVsi.vsiFpm = dataRefMgr_->getVsiFpm();

        msgQueue_->enqueueTx(MessageType::SerialMessageAltimeterVsi, &altimeterVsi, sizeof(altimeterVsi));

        altimeterVsiAccumulator_ = 0.0f;
    }

    // ============ Compass Data (50 Hz) ============
    compassAccumulator_ += dt;
    float compassRate = 1.0f / COMPASS_RATE;

    if (compassAccumulator_ >= compassRate)
    {
        struct CompassData
        {
            float headingDegMag;
        };

        CompassData compass;
        compass.headingDegMag = dataRefMgr_->getCompassHeadingDegMag();

        msgQueue_->enqueueTx(MessageType::SerialMessageCompass, &compass, sizeof(compass));

        compassAccumulator_ = 0.0f;
    }

    // ============ HSI heading / OBS / bug / flags (50 Hz) ============
    hsiHeadingAccumulator_ += dt;
    float hsiHeadingRate = 1.0f / HSI_HEADING_RATE;

    if (hsiHeadingAccumulator_ >= hsiHeadingRate)
    {
        HsiHeadingMessage hsi;
        hsi.headingDegMag = dataRefMgr_->getHsiHeadingDegMag();
        hsi.obsDeg = dataRefMgr_->getHsiObsDeg();
        hsi.hdgBugDeg = dataRefMgr_->getHsiHeadingBugDeg();

        // Same helper the DCU uses, from shared/CANBase/include/HsiWire.h, so
        // the flags semantics have one tested implementation rather than two.
        hsi.flags = hsiFlagsByte(dataRefMgr_->getNav1FromTo(),
                                 dataRefMgr_->getNav1GlideslopeValid());

        // The HSI's gyro and its nav needles are electric. On a dead bus the
        // card simply stays put (we keep forwarding the sim's heading, tumble
        // included), but the TO/FROM/GS flags drop and the deviation needles
        // (below) centre.
        if (!busPowered)
        {
            hsi.flags = 0;
        }

        msgQueue_->enqueueTx(MessageType::SerialMessageHsiHeading, &hsi, sizeof(hsi));

        hsiHeadingAccumulator_ = 0.0f;
    }

    // ============ HSI deviation needles (10 Hz) ============
    hsiNavAccumulator_ += dt;
    float hsiNavRate = 1.0f / HSI_NAV_RATE;

    if (hsiNavAccumulator_ >= hsiNavRate)
    {
        HsiNavMessage nav;
        nav.cdiDots = dataRefMgr_->getNav1CdiDots();
        nav.gsDots = dataRefMgr_->getNav1GsDots();

        // De-energized meter movements: both needles park centred.
        if (!busPowered)
        {
            nav.cdiDots = 0.0f;
            nav.gsDots = 0.0f;
        }

        msgQueue_->enqueueTx(MessageType::SerialMessageHsiNav, &nav, sizeof(nav));

        hsiNavAccumulator_ = 0.0f;
    }

    // ============ Attitude indicator (50 Hz) ============
    attitudeAccumulator_ += dt;
    float attitudeRate = 1.0f / ATTITUDE_RATE;

    if (attitudeAccumulator_ >= attitudeRate)
    {
        AttitudeMessage attitude;
        attitude.pitchDeg = dataRefMgr_->getPitchVacuumDeg();
        attitude.rollDeg = dataRefMgr_->getRollVacuumDeg();

        // Vacuum degradation, modelled centrally (see top of function). With a
        // healthy gyro this is a no-op.
        attitudeGyro_.apply(attitude.pitchDeg, attitude.rollDeg);

        msgQueue_->enqueueTx(MessageType::SerialMessageAttitude, &attitude, sizeof(attitude));

        attitudeAccumulator_ = 0.0f;
    }

    // ============ Turn coordinator (50 Hz) ============
    turnCoordinatorAccumulator_ += dt;
    float turnCoordinatorRate = 1.0f / TURN_COORDINATOR_RATE;

    if (turnCoordinatorAccumulator_ >= turnCoordinatorRate)
    {
        TurnCoordinatorMessage tc;
        tc.turnRateDeg = dataRefMgr_->getTurnRateDeg();
        tc.slipDeg = dataRefMgr_->getSlipDeg();

        // The turn needle is electric and centres when the gyro dies; the slip
        // ball is a mechanical inclinometer and keeps working without power.
        if (!busPowered)
        {
            tc.turnRateDeg = 0.0f;
        }

        msgQueue_->enqueueTx(MessageType::SerialMessageTurnCoordinator, &tc, sizeof(tc));

        turnCoordinatorAccumulator_ = 0.0f;
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

        case MessageType::SerialMessageBaro:
            // Gateway → Plugin: barometer knob on the altimeter (CAN 0x340)
            if (msg->payload.size() >= sizeof(float))
            {
                float inHg = 0.0f;
                std::memcpy(&inHg, msg->payload.data(), sizeof(float));
                dataRefMgr_->setBarometerSetting(inHg);
            }
            break;

        case MessageType::SerialMessageHsiInput:
            // Gateway -> Plugin: OBS and HDG knobs on the HSI (CAN 0x341).
            // The payload is a *delta*, never an absolute position: the board
            // displays whatever the sim reports back over 0x104, so the loop
            // closes here and a GPS or autopilot moving OBS causes no fight.
            if (msg->payload.size() >= sizeof(HsiInputToDcuMessage))
            {
                HsiInputToDcuMessage in;
                std::memcpy(&in, msg->payload.data(), sizeof(in));

                auto wrap360 = [](float deg) {
                    deg = std::fmod(deg, 360.0f);
                    if (deg < 0.0f)
                        deg += 360.0f;
                    return deg;
                };

                if (in.action == 0)
                {
                    if (in.control == 0)
                    {
                        const float next = dataRefMgr_->getHsiObsDeg() +
                                           static_cast<float>(in.delta) * OBS_DEG_PER_CLICK;
                        dataRefMgr_->setHsiObsDeg(wrap360(next));
                    }
                    else if (in.control == 1)
                    {
                        const float next = dataRefMgr_->getHsiHeadingBugDeg() +
                                           static_cast<float>(in.delta) * HDG_DEG_PER_CLICK;
                        dataRefMgr_->setHeadingBug(wrap360(next));
                    }
                }
                else if (in.action == 1 && in.pushState == 1 && in.control == 1)
                {
                    // HDG knob press syncs the bug onto the current heading.
                    dataRefMgr_->setHeadingBug(wrap360(dataRefMgr_->getHsiHeadingDegMag()));
                }
                // control 0 with action 1 is the OBS push: reserved, no-op.
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

                const float rudderRaw = static_cast<float>(message->rudder) / 1000.0f;
                if (curveWin_)
                    curveWin_->setLiveInput(rudderRaw);
                dataRefMgr_->setRudderRatio(rudderCurve_.apply(rudderRaw));
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
    data.rescanFlash = rescanFlashRemaining_ > 0.0f;

    statusWin_->update(data);
}