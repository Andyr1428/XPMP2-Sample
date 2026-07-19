/// @file       XPMP2-Sample.cpp
/// @brief      AeroPath Traffic - initial XPMP2 rendering foundation
/// @details    Creates one AeroPath-controlled test aircraft using the recommended
///             XPMP2::Aircraft interface. This replaces the original three-aircraft
///             demonstration and provides a clean base for the AeroPath multiplayer feed.
///
///             This first development build keeps the aircraft at a fixed relative
///             position in front of the user's aircraft. Once this build is confirmed
///             in X-Plane, the relative test position will be replaced by live AeroPath
///             pilot telemetry.
///
/// @copyright  Based on the XPMP2-Sample project:
///             Copyright (c) 2020 Birger Hoppe
///             Used under the MIT License.

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>

// X-Plane SDK
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"

// XPMP2
#include "XPMPAircraft.h"
#include "XPMPMultiplayer.h"

#if !XPLM300
    #error This plugin requires X-Plane SDK 3.0 or newer
#endif

using namespace XPMP2;

namespace
{
    constexpr double PI = 3.141592653589793238462643383279502884;

    // Keep the first controlled aircraft clearly visible during the development test.
    constexpr float TEST_DISTANCE_METRES = 300.0f;
    constexpr float TEST_RIGHT_OFFSET_METRES = 80.0f;
    constexpr float TEST_ALTITUDE_OFFSET_METRES = 40.0f;

    constexpr const char* TEST_ICAO_TYPE = "A319";
    constexpr const char* TEST_AIRLINE = "BAW";
    constexpr const char* TEST_LIVERY = "";
    constexpr const char* TEST_CALLSIGN = "AEROPATH TEST";

    XPLMMenuID gMenu = nullptr;

    bool gTrafficEnabled = true;
    bool gAircraftVisible = true;

    XPLMDataRef gLocalX = nullptr;
    XPLMDataRef gLocalY = nullptr;
    XPLMDataRef gLocalZ = nullptr;
    XPLMDataRef gHeading = nullptr;

    struct LocalPosition
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    enum MenuItem
    {
        MENU_TRAFFIC_ENABLED = 0,
        MENU_AIRCRAFT_VISIBLE,
        MENU_AI_CONTROL,
#ifdef DEBUG
        MENU_RELOAD_PLUGINS,
#endif
    };

    void LogMessage(const char* format, ...)
    {
        char buffer[1024] = {};

        va_list arguments;
        va_start(arguments, format);
        std::vsnprintf(buffer, sizeof(buffer) - 2, format, arguments);
        va_end(arguments);

        std::strcat(buffer, "\n");
        XPLMDebugString(buffer);
    }

    double DegreesToRadians(const double degrees)
    {
        return degrees * PI / 180.0;
    }

    char* SafeCopy(char* destination, const char* source, const std::size_t destinationSize)
    {
        if (!destination || destinationSize == 0)
            return destination;

        std::strncpy(destination, source ? source : "", destinationSize);
        destination[destinationSize - 1] = '\0';
        return destination;
    }

    LocalPosition CalculateTestPosition()
    {
        LocalPosition position
        {
            XPLMGetDatad(gLocalX),
            XPLMGetDatad(gLocalY),
            XPLMGetDatad(gLocalZ)
        };

        const double headingRadians = DegreesToRadians(XPLMGetDataf(gHeading));

        // Forward vector in X-Plane's local coordinate system.
        position.x += std::sin(headingRadians) * TEST_DISTANCE_METRES;
        position.z -= std::cos(headingRadians) * TEST_DISTANCE_METRES;

        // Right vector, perpendicular to the forward vector.
        position.x += std::cos(headingRadians) * TEST_RIGHT_OFFSET_METRES;
        position.z += std::sin(headingRadians) * TEST_RIGHT_OFFSET_METRES;

        position.y += TEST_ALTITUDE_OFFSET_METRES;
        return position;
    }

    int PreferencesCallback(const char*, const char* item, int defaultValue)
    {
        // Use the full OBJ8 model feature set.
        if (!std::strcmp(item, XPMP_CFG_ITM_REPLDATAREFS))
            return 1;

        if (!std::strcmp(item, XPMP_CFG_ITM_REPLTEXTURE))
            return 1;

#if DEBUG
        if (!std::strcmp(item, XPMP_CFG_ITM_LOGLEVEL))
            return 0;
#endif

        return defaultValue;
    }
}

class AeroPathAircraft final : public XPMP2::Aircraft
{
public:
    AeroPathAircraft(
        const std::string& icaoType,
        const std::string& airline,
        const std::string& livery,
        XPMPPlaneID modeSId)
        : Aircraft(icaoType, airline, livery, modeSId)
    {
        label = TEST_CALLSIGN;

        colLabel[0] = 0.0f;
        colLabel[1] = 1.0f;
        colLabel[2] = 0.0f;

        acRadar.code = 1200;
        acRadar.mode = xpmpTransponderMode_ModeC;

        SafeCopy(acInfoTexts.icaoAcType, icaoType.c_str(), sizeof(acInfoTexts.icaoAcType));
        SafeCopy(acInfoTexts.icaoAirline, airline.c_str(), sizeof(acInfoTexts.icaoAirline));
        SafeCopy(acInfoTexts.flightNum, TEST_CALLSIGN, sizeof(acInfoTexts.flightNum));
        SafeCopy(acInfoTexts.tailNum, "AP-TEST", sizeof(acInfoTexts.tailNum));
    }

    void UpdatePosition(float, int) override
    {
        const LocalPosition localPosition = CalculateTestPosition();

        double latitude = 0.0;
        double longitude = 0.0;
        double elevationMetres = 0.0;

        XPLMLocalToWorld(
            localPosition.x,
            localPosition.y,
            localPosition.z,
            &latitude,
            &longitude,
            &elevationMetres);

        const double elevationFeet = elevationMetres / M_per_FT;

        SetLocation(latitude, longitude, elevationFeet, false);

        // Match the user's direction so the aircraft remains stable and easy to inspect.
        SetHeading(XPLMGetDataf(gHeading));
        SetPitch(0.0f);
        SetRoll(0.0f);

        // Airborne development state.
        SetGearRatio(0.0f);
        SetFlapRatio(0.0f);
        SetSpoilerRatio(0.0f);
        SetSpeedbrakeRatio(0.0f);
        SetSlatRatio(0.0f);
        SetThrustRatio(0.45f);

        SetLightsTaxi(false);
        SetLightsLanding(false);
        SetLightsBeacon(true);
        SetLightsStrobe(true);
        SetLightsNav(true);

        SetTireDeflection(0.0f);
        SetTireRotAngle(0.0f);
        SetTireRotRpm(0.0f);

        SetEngineRotRpm(1, 1800.0f);
        SetEngineRotRpm(2, 1800.0f);

        SetThrustReversRatio(0.0f);
        SetReversDeployRatio(0.0f);
        SetTouchDown(false);
    }
};

namespace
{
    AeroPathAircraft* gTestAircraft = nullptr;

    bool IsAircraftCreated()
    {
        return gTestAircraft != nullptr;
    }

    void UpdateMenuCheckmarks()
    {
        if (!gMenu)
            return;

        XPLMCheckMenuItem(
            gMenu,
            MENU_TRAFFIC_ENABLED,
            gTrafficEnabled ? xplm_Menu_Checked : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_AIRCRAFT_VISIBLE,
            gAircraftVisible ? xplm_Menu_Checked : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_AI_CONTROL,
            XPMPHasControlOfAIAircraft() ? xplm_Menu_Checked : xplm_Menu_Unchecked);
    }

    void CreateAircraft()
    {
        if (!gTrafficEnabled || gTestAircraft)
            return;

        try
        {
            gTestAircraft = new AeroPathAircraft(
                TEST_ICAO_TYPE,
                TEST_AIRLINE,
                TEST_LIVERY,
                0xAE0001);

            gTestAircraft->SetVisible(gAircraftVisible);

            const XPMP2::CSLModelInfo_t modelInfo = gTestAircraft->GetModelInfo();

            LogMessage(
                "AeroPath Traffic: Created %s/%s using CSL model '%s'",
                TEST_ICAO_TYPE,
                TEST_AIRLINE,
                modelInfo.modelName.c_str());
        }
        catch (const XPMP2::XPMP2Error& exception)
        {
            LogMessage(
                "AeroPath Traffic: Could not create test aircraft: %s",
                exception.what());

            delete gTestAircraft;
            gTestAircraft = nullptr;
        }

        UpdateMenuCheckmarks();
    }

    void RemoveAircraft()
    {
        delete gTestAircraft;
        gTestAircraft = nullptr;

        LogMessage("AeroPath Traffic: Test aircraft removed");
        UpdateMenuCheckmarks();
    }

    void SetAircraftVisibility()
    {
        gAircraftVisible = !gAircraftVisible;

        if (gTestAircraft)
            gTestAircraft->SetVisible(gAircraftVisible);

        UpdateMenuCheckmarks();
    }

    void RequestAIAgain(void*)
    {
        XPMPMultiplayerEnable(RequestAIAgain);
        UpdateMenuCheckmarks();
    }

    void MenuCallback(void*, void* itemReference)
    {
        const auto selectedItem =
            static_cast<MenuItem>(reinterpret_cast<unsigned long long>(itemReference));

        switch (selectedItem)
        {
            case MENU_TRAFFIC_ENABLED:
                gTrafficEnabled = !gTrafficEnabled;

                if (gTrafficEnabled)
                    CreateAircraft();
                else
                    RemoveAircraft();

                break;

            case MENU_AIRCRAFT_VISIBLE:
                SetAircraftVisibility();
                break;

            case MENU_AI_CONTROL:
                if (XPMPHasControlOfAIAircraft())
                    XPMPMultiplayerDisable();
                else
                    XPMPMultiplayerEnable(RequestAIAgain);

                break;

#ifdef DEBUG
            case MENU_RELOAD_PLUGINS:
                XPLMReloadPlugins();
                break;
#endif
        }

        UpdateMenuCheckmarks();
    }

    void PlaneNotifier(
        XPMPPlaneID planeId,
        XPMPPlaneNotification notification,
        void*)
    {
        XPMP2::Aircraft* aircraft = XPMP2::AcFindByID(planeId);
        if (!aircraft)
            return;

        const char* notificationText =
            notification == xpmp_PlaneNotification_Created ? "created" :
            notification == xpmp_PlaneNotification_ModelChanged ? "model changed" :
            "destroyed";

        LogMessage(
            "AeroPath Traffic: Aircraft '%s', type %s, model %s - %s",
            aircraft->label.c_str(),
            aircraft->acIcaoType.c_str(),
            aircraft->GetModelName().c_str(),
            notificationText);
    }
}

PLUGIN_API int XPluginStart(char* outName, char* outSignature, char* outDescription)
{
    std::strcpy(outName, "AeroPath Traffic");
    std::strcpy(outSignature, "uk.co.aeropath.traffic");
    std::strcpy(
        outDescription,
        "AeroPath multiplayer aircraft rendering for X-Plane");

    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    const int menuSlot = XPLMAppendMenuItem(
        XPLMFindPluginsMenu(),
        "AeroPath Traffic",
        nullptr,
        0);

    gMenu = XPLMCreateMenu(
        "AeroPath Traffic",
        XPLMFindPluginsMenu(),
        menuSlot,
        MenuCallback,
        nullptr);

    XPLMAppendMenuItem(
        gMenu,
        "Traffic Enabled",
        reinterpret_cast<void*>(MENU_TRAFFIC_ENABLED),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Test Aircraft Visible",
        reinterpret_cast<void*>(MENU_AIRCRAFT_VISIBLE),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Toggle AI/TCAS Control",
        reinterpret_cast<void*>(MENU_AI_CONTROL),
        0);

#ifdef DEBUG
    XPLMAppendMenuItem(
        gMenu,
        "Reload Plugins",
        reinterpret_cast<void*>(MENU_RELOAD_PLUGINS),
        0);
#endif

    gLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
    gLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
    gLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
    gHeading = XPLMFindDataRef("sim/flightmodel/position/psi");

    UpdateMenuCheckmarks();

    LogMessage("AeroPath Traffic: Plugin started");
    return 1;
}

PLUGIN_API void XPluginStop()
{
    if (gMenu)
    {
        XPLMDestroyMenu(gMenu);
        gMenu = nullptr;
    }

    LogMessage("AeroPath Traffic: Plugin stopped");
}

PLUGIN_API int XPluginEnable()
{
    const char pathSeparator = XPLMGetDirectorySeparator()[0];

    char pluginPath[1024] = {};
    XPLMGetPluginInfo(
        XPLMGetMyID(),
        nullptr,
        pluginPath,
        nullptr,
        nullptr);

    char* finalSeparator = std::strrchr(pluginPath, pathSeparator);
    if (!finalSeparator)
    {
        LogMessage("AeroPath Traffic: Could not resolve plugin path");
        return 0;
    }

    *finalSeparator = '\0';

    finalSeparator = std::strrchr(pluginPath, pathSeparator);
    if (!finalSeparator)
    {
        LogMessage("AeroPath Traffic: Could not resolve plugin root folder");
        return 0;
    }

    *(finalSeparator + 1) = '\0';

    std::string resourcePath = pluginPath;
    resourcePath += "Resources";

    const char* result = XPMPMultiplayerInit(
        "AeroPath Traffic",
        resourcePath.c_str(),
        PreferencesCallback,
        "A319");

    if (result[0])
    {
        LogMessage(
            "AeroPath Traffic: XPMP2 initialization failed: %s",
            result);

        return 0;
    }

    result = XPMPLoadCSLPackage(resourcePath.c_str());
    if (result[0])
    {
        LogMessage(
            "AeroPath Traffic: CSL loading reported: %s",
            result);
    }

    result = XPMPMultiplayerEnable(RequestAIAgain);
    if (result[0])
    {
        LogMessage(
            "AeroPath Traffic: AI/TCAS control unavailable: %s",
            result);
    }

    XPMPRegisterPlaneNotifierFunc(PlaneNotifier, nullptr);

    CreateAircraft();
    UpdateMenuCheckmarks();

    LogMessage(
        "AeroPath Traffic: Enabled using Resources folder '%s'",
        resourcePath.c_str());

    return 1;
}

PLUGIN_API void XPluginDisable()
{
    RemoveAircraft();

    XPMPMultiplayerDisable();
    XPMPUnregisterPlaneNotifierFunc(PlaneNotifier, nullptr);
    XPMPMultiplayerCleanup();

    LogMessage("AeroPath Traffic: Disabled");
}

PLUGIN_API void XPluginReceiveMessage(
    XPLMPluginID,
    long message,
    void*)
{
    if (message == XPLM_MSG_RELEASE_PLANES)
    {
        XPMPMultiplayerDisable();
        UpdateMenuCheckmarks();

        LogMessage(
            "AeroPath Traffic: Released AI/TCAS control at another plugin's request");
    }
}
