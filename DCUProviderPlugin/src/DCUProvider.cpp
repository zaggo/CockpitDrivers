#include "DCUProvider.h"
#include "XPLMUtilities.h"
#include "ConfigUtils.h"
#include <cstdio>
#include <ctime>
#include <thread>
#include <chrono>

DCUProvider::DCUProvider() = default;

DCUProvider::~DCUProvider() {
    shutdown();
}

bool DCUProvider::initialize() {
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
    if (!lastPort.empty()) {
        for (size_t i = 0; i < ports.size(); ++i) {
            if (ports[i] == lastPort) {
                preselectIdx = (int)i;
                currentPort_ = lastPort;
                break;
            }
        }
    }

    connMgr_.reset();
    // Nur verbinden, wenn ein Port gesetzt ist
    if (!currentPort_.empty()) {
        connMgr_ = std::make_unique<ConnectionManager>(currentPort_, 115200);
        connMgr_->connect();
    }

    statusWin_ = std::make_unique<StatusWindow>();
    statusWin_->initialize();
    statusWin_->setAvailablePorts(ports);
    if (preselectIdx >= 0) statusWin_->selectedPortIdx_ = preselectIdx;
    statusWin_->setPortChangedCallback([this](const std::string& port) {
        changePort(port);
    });

    char msg[256];
    std::snprintf(msg, sizeof(msg),
                  "DCUProvider: Initialized (Connection: %s)\n",
                  (connMgr_ && connMgr_->isConnected()) ? "OK" : "FAILED");
    XPLMDebugString(msg);

    return true;  // Even if initial connection fails, plugin loads
}

void DCUProvider::changePort(const std::string& newPort) {
        saveLastUsedPort(newPort);
        if (newPort == currentPort_)
            return;
        currentPort_ = newPort;
        if (connMgr_) {
            connMgr_->disconnect();
            connMgr_.reset();
        }
        // Queue leeren
        if (msgQueue_) {
            msgQueue_->resetStats(); // setzt auch Zähler zurück
            msgQueue_->clearTxQueue();
            while (msgQueue_->hasRxPending()) msgQueue_->dequeueRx();
        }
        if (!currentPort_.empty()) {
            connMgr_ = std::make_unique<ConnectionManager>(currentPort_, 115200);
            connMgr_->connect();
            // Längeres Delay nach Port-Öffnung (1 Sekunde) für Arduino-Reset
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        // Optionally, update status window immediately
        updateStatusWindow();
}

void DCUProvider::shutdown() {
    XPLMDebugString("DCUProvider: Shutting down\n");
    
    if (statusWin_) {
        statusWin_->destroy();
        statusWin_.reset();
    }
    
    if (connMgr_) {
        connMgr_->disconnect();
        connMgr_.reset();
    }
    
    msgQueue_.reset();
    dataRefMgr_.reset();
}

void DCUProvider::onFlightLoopTick(float elapsedTime) {
    if (!connMgr_ || !msgQueue_ || !dataRefMgr_) {
        return;
    }
    
    // ============ Connection Management ============
    connMgr_->update(elapsedTime);
    
    // ============ I/O Processing ============
    connMgr_->processIO(*msgQueue_);
    
    // ============ Downlink: X-Plane → Gateway ============
    updateDownlink(elapsedTime);
    
    // ============ Uplink: Gateway → X-Plane ============
    updateUplink();
    
    // ============ Status Window Update (1Hz) ============
    static float statusUpdateAccum = 0.0f;
    statusUpdateAccum += elapsedTime;
    
    if (statusUpdateAccum >= 1.0f) {
        updateStatusWindow();
        statusUpdateAccum = 0.0f;
    }
}

bool DCUProvider::isConnected() const {
    return connMgr_ && connMgr_->isConnected();
}

void DCUProvider::updateDownlink(float dt) {
    if (!isConnected()) {
        return;
    }
    
    // ============ Fuel Data (5 Hz) ============
    fuelAccumulator_ += dt;
    float fuelRate = 1.0f / 5.0f;  // 5 Hz = every 0.2s
    
    if (fuelAccumulator_ >= fuelRate) {
        struct FuelData {
            float fuelLeft;
            float fuelRight;
        };
        
        // Read fuel from X-Plane
        FuelData fuel;
        fuel.fuelLeft = dataRefMgr_->getFuelLeft();
        fuel.fuelRight = dataRefMgr_->getFuelRight();
        
        msgQueue_->enqueueTx(MessageType::Fuel, &fuel, sizeof(fuel));
        
        fuelAccumulator_ = 0.0f;
    }
    
    // ============ Lights Data (10 Hz) ============
    lightsAccumulator_ += dt;
    float lightsRate = 1.0f / 10.0f;  // 10 Hz = every 0.1s
    
    if (lightsAccumulator_ >= lightsRate) {
        struct LightsData {
            float panelDim;  // 0..1
            float radioDim;  // 0..1
            float domeLightDim; // 0..1
        };
                
        LightsData lights;
        auto brightness = dataRefMgr_->getPanelBrightness();
        lights.panelDim = brightness[0];
        lights.radioDim = brightness[1];
        lights.domeLightDim = dataRefMgr_->getDomeLightBrightness();
        
        msgQueue_->enqueueTx(MessageType::Lights, &lights, sizeof(lights));
        
        lightsAccumulator_ = 0.0f;
    }
    
    // TODO: Add more downlink data based on CAN Message IDs
    // - Altimeter settings
    // - Heading bug
    // - Course selector
    // - etc.
}

void DCUProvider::updateUplink() {
    if (!msgQueue_) {
        return;
    }
    
    // Process all received messages
    while (msgQueue_->hasRxPending()) {
        auto msg = msgQueue_->dequeueRx();
        
        if (!msg) {
            continue;
        }
        
        // Route message by type
        switch (msg->type) {
            case MessageType::Fuel:
                // Gateway → Plugin: This would be for reading fuel from a device
                // (not typical for Piper Arrow, but possible)
                break;
            
            case MessageType::Lights:
                // Gateway → Plugin: This would be for reading light switches from device
                // (not typical for Piper Arrow, but possible)
                break;
            
            // TODO: Handle more message types
            // - Barometer setting from altimeter
            // - Heading bug from HSI
            // - Course selector from CDI
            // - Button presses from panels
            // - Encoder values
            // etc.
            
            default:
                // Unknown message type
                break;
        }
    }
}

void DCUProvider::updateStatusWindow() {
    if (!statusWin_) {
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