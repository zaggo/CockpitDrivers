#include "DCUProvider.h"
#include "XPLMUtilities.h"
#include <cstdio>
#include <ctime>

DCUProvider::DCUProvider() = default;

DCUProvider::~DCUProvider() {
    shutdown();
}

bool DCUProvider::initialize() {
    // Initialize components
    dataRefMgr_ = std::make_unique<DataRefManager>();
    dataRefMgr_->initialize();
    
    msgQueue_ = std::make_unique<MessageQueue>();
    
    connMgr_ = std::make_unique<ConnectionManager>(
        "/dev/cu.usbserial-1420",
        115200
    );
    
    statusWin_ = std::make_unique<StatusWindow>();
    statusWin_->initialize();
    
    // Try initial connection
    bool connected = connMgr_->connect();
    
    char msg[256];
    std::snprintf(msg, sizeof(msg), 
                  "DCUProvider: Initialized (Connection: %s)\n",
                  connected ? "OK" : "FAILED");
    XPLMDebugString(msg);
    
    return true;  // Even if initial connection fails, plugin loads
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
            uint16_t fuelLeft;   // 0.1 liter units
            uint16_t fuelRight;  // 0.1 liter units
        };
        
        // Read fuel from X-Plane (in kg, convert to liters)
        float fuelLKg = dataRefMgr_->getFuelLeft();
        float fuelRKg = dataRefMgr_->getFuelRight();
        
        // Approximate: 1 liter ≈ 0.8 kg (Avgas)
        float fuelLLiters = fuelLKg / 0.8f;
        float fuelRLiters = fuelRKg / 0.8f;
        
        FuelData fuel;
        fuel.fuelLeft = static_cast<uint16_t>(fuelLLiters * 10.0f);   // 0.1L units
        fuel.fuelRight = static_cast<uint16_t>(fuelRLiters * 10.0f);
        
        msgQueue_->enqueueTx(MessageType::Fuel, &fuel, sizeof(fuel));
        
        fuelAccumulator_ = 0.0f;
    }
    
    // ============ Lights Data (2 Hz) ============
    lightsAccumulator_ += dt;
    float lightsRate = 1.0f / 2.0f;  // 2 Hz = every 0.5s
    
    if (lightsAccumulator_ >= lightsRate) {
        struct LightsData {
            uint8_t panelBrightness;  // 0-255
            uint8_t radioBrightness;  // 0-255
            uint8_t domeLight;        // 0=off, 1=on
            uint8_t reserved;         // padding
        };
        
        float panelBright = dataRefMgr_->getPanelBrightness();
        float radioBright = dataRefMgr_->getRadioBrightness();
        bool domeOn = dataRefMgr_->getDomeLightOn();
        
        LightsData lights;
        lights.panelBrightness = static_cast<uint8_t>(panelBright * 255.0f);
        lights.radioBrightness = static_cast<uint8_t>(radioBright * 255.0f);
        lights.domeLight = domeOn ? 1 : 0;
        lights.reserved = 0;
        
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
    data.devicePath = "/dev/cu.usbserial-1440";
    data.baudRate = 115200;
    data.txBytesSent = msgQueue_->getTxBytesSent();
    data.rxBytesReceived = msgQueue_->getRxBytesReceived();
    data.lastTxTime = connMgr_->getLastTxTime();
    data.lastRxTime = connMgr_->getLastRxTime();
    data.lastWriteOk = connMgr_->getLastWriteOk();
    data.lastOpenOk = connMgr_->getLastOpenOk();
    
    statusWin_->update(data);
}