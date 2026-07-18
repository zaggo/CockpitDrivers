#include "MotionProvider.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"
#include "XPLMUtilities.h"
#include <memory>
#include <cstring>

static std::unique_ptr<MotionProvider> gProvider;

// Fixed motion-output tick, decoupled from render frame rate. 60 Hz target.
static constexpr float kFlightLoopIntervalSec = 1.0f / 60.0f;

static float FlightLoopCB(float elapsedTime, float, int, void*) {
    if (gProvider) {
        gProvider->onFlightLoopTick(elapsedTime);
    }
    return kFlightLoopIntervalSec;
}

PLUGIN_API int XPluginStart(char* outName, char* outSig, char* outDesc) {
    std::strcpy(outName, "Motion Provider");
    std::strcpy(outSig,  "com.pleasantsoftware.motion.provider");
    std::strcpy(outDesc, "6DOF Motion Platform Cueing - X-Plane Plugin");

    gProvider = std::make_unique<MotionProvider>();
    if (!gProvider->initialize()) {
        XPLMDebugString("MotionProvider: Failed to initialize\n");
        gProvider.reset();
        return 0;
    }

    XPLMRegisterFlightLoopCallback(FlightLoopCB, kFlightLoopIntervalSec, nullptr);
    XPLMDebugString("MotionProvider: Plugin started successfully\n");
    return 1;
}

PLUGIN_API void XPluginStop(void) {
    XPLMDebugString("MotionProvider: Plugin stopping\n");
    XPLMUnregisterFlightLoopCallback(FlightLoopCB, nullptr);
    if (gProvider) {
        gProvider->shutdown();
        gProvider.reset();
    }
    XPLMDebugString("MotionProvider: Plugin stopped\n");
}

PLUGIN_API void XPluginDisable(void) {
    XPLMDebugString("MotionProvider: Plugin disabled\n");
}

PLUGIN_API int XPluginEnable(void) {
    XPLMDebugString("MotionProvider: Plugin enabled\n");
    return 1;
}

PLUGIN_API void XPluginReceiveMessage(XPLMPluginID inFromWho, int inMessage, void* inParam) {
    (void)inFromWho;
    (void)inParam;
    if (inMessage == XPLM_MSG_PLANE_LOADED) {
        if (gProvider) {
            gProvider->onAircraftLoaded();
        }
    }
}
