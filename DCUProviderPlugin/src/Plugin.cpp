#include "DCUProvider.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include <memory>
#include <cstring>

// Global plugin instance
static std::unique_ptr<DCUProvider> gProvider;

// Flight loop callback
static float FlightLoopCB(float elapsedTime, float, int, void*) {
    if (gProvider) {
        gProvider->onFlightLoopTick(elapsedTime);
    }
    return -1.0f;  // Call every frame
}

// ============ X-Plane Plugin API ============

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    // Plugin identification
    std::strcpy(outName, "DCU Provider");
    std::strcpy(outSig, "com.pleasantsoftware.dcu.provider");
    std::strcpy(outDesc, "Piper Arrow III Cockpit CAN Gateway - X-Plane Plugin");
    
    // Initialize provider
    gProvider = std::make_unique<DCUProvider>();
    
    if (!gProvider->initialize()) {
        XPLMDebugString("DCUProvider: Failed to initialize\n");
        gProvider.reset();
        return 0;
    }
    
    // Register flight loop callback
    XPLMRegisterFlightLoopCallback(FlightLoopCB, -1.0f, nullptr);
    
    XPLMDebugString("DCUProvider: Plugin started successfully\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    XPLMDebugString("DCUProvider: Plugin stopping\n");
    
    // Unregister flight loop callback
    XPLMUnregisterFlightLoopCallback(FlightLoopCB, nullptr);
    
    // Shutdown provider
    if (gProvider) {
        gProvider->shutdown();
        gProvider.reset();
    }
    
    XPLMDebugString("DCUProvider: Plugin stopped\n");
}

PLUGIN_API void XPluginDisable(void) {
    // Plugin is being disabled (but not unloaded)
    // We could pause operations here if needed
    XPLMDebugString("DCUProvider: Plugin disabled\n");
}

PLUGIN_API int XPluginEnable(void) {
    // Plugin is being re-enabled
    // We could resume operations here if needed
    XPLMDebugString("DCUProvider: Plugin enabled\n");
    return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam) {
    // Handle inter-plugin messages
    // For now, we don't need to handle any
    (void)inFromWho;
    (void)inMessage;
    (void)inParam;
}