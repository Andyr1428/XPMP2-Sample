/// @file       XPMP2-Sample.cpp
/// @brief      AeroPath Traffic - external traffic feed development build
/// @details    Renders one AeroPath-controlled aircraft through XPMP2 and reads
///             its state from Resources/AeroPathTraffic.txt.
///
///             Supported feed modes:
///             - relative: forward/right/up offsets from the user's aircraft
///             - world:    absolute latitude/longitude/altitude coordinates
///
///             This is the first local bridge between AeroPath data and the
///             X-Plane multiplayer renderer.
///
/// @copyright  Based on the XPMP2-Sample project:
///             Copyright (c) 2020 Birger Hoppe
///             Used under the MIT License.

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <cmath>
#include <string>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <ctime>
#include <sys/stat.h>

// X-Plane SDK
#include "XPLMDataAccess.h"
#include "XPLMUtilities.h"
#include "XPLMPlugin.h"
#include "XPLMMenus.h"
#include "XPLMGraphics.h"
#include "XPLMProcessing.h"

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
    constexpr float TRAFFIC_POLL_INTERVAL_SECONDS = 0.25f;
    constexpr double TRAFFIC_FEED_TIMEOUT_SECONDS = 15.0;

    XPLMMenuID gMenu = nullptr;

    bool gMasterEnabled = true;
    bool gAircraftVisible = true;
    bool gFeedFilePreviouslyMissing = false;
    bool gFeedFilePreviouslyStale = false;

    XPLMDataRef gLocalX = nullptr;
    XPLMDataRef gLocalY = nullptr;
    XPLMDataRef gLocalZ = nullptr;
    XPLMDataRef gHeading = nullptr;

    std::string gTrafficFilePath;

    struct LocalPosition
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct TrafficState
    {
        bool feedEnabled = true;
        bool worldMode = false;
        bool onGround = false;

        std::string icaoType = "A319";
        std::string airline = "BAW";
        std::string livery;
        std::string callsign = "AEROPATH TEST";

        // Relative mode
        float forwardMetres = 300.0f;
        float rightMetres = 80.0f;
        float upMetres = 40.0f;
        float headingOffsetDegrees = 0.0f;

        // World mode
        double latitude = 0.0;
        double longitude = 0.0;
        double altitudeFeet = 0.0;
        float headingDegrees = 0.0f;

        // Shared attitude and aircraft configuration
        float pitchDegrees = 0.0f;
        float rollDegrees = 0.0f;
        float gearRatio = 0.0f;
        float flapRatio = 0.0f;
        float thrustRatio = 0.45f;

        bool taxiLight = false;
        bool landingLight = false;
        bool beaconLight = true;
        bool strobeLight = true;
        bool navLight = true;
    };

    TrafficState gTrafficState;

    enum MenuItem
    {
        MENU_TRAFFIC_ENABLED = 0,
        MENU_AIRCRAFT_VISIBLE,
        MENU_RELOAD_FEED,
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

    std::string Trim(const std::string& value)
    {
        const auto first = std::find_if_not(
            value.begin(),
            value.end(),
            [](unsigned char character)
            {
                return std::isspace(character) != 0;
            });

        if (first == value.end())
            return {};

        const auto last = std::find_if_not(
            value.rbegin(),
            value.rend(),
            [](unsigned char character)
            {
                return std::isspace(character) != 0;
            }).base();

        return std::string(first, last);
    }

    std::string ToLower(std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char character)
            {
                return static_cast<char>(std::tolower(character));
            });

        return value;
    }

    bool ParseBool(const std::string& value, const bool currentValue)
    {
        const std::string normalised = ToLower(Trim(value));

        if (normalised == "1" ||
            normalised == "true" ||
            normalised == "yes" ||
            normalised == "on")
        {
            return true;
        }

        if (normalised == "0" ||
            normalised == "false" ||
            normalised == "no" ||
            normalised == "off")
        {
            return false;
        }

        return currentValue;
    }

    float ParseFloat(const std::string& value, const float currentValue)
    {
        try
        {
            std::size_t parsedCharacters = 0;
            const float parsedValue = std::stof(Trim(value), &parsedCharacters);

            if (parsedCharacters == 0)
                return currentValue;

            return parsedValue;
        }
        catch (...)
        {
            return currentValue;
        }
    }

    double ParseDouble(const std::string& value, const double currentValue)
    {
        try
        {
            std::size_t parsedCharacters = 0;
            const double parsedValue = std::stod(Trim(value), &parsedCharacters);

            if (parsedCharacters == 0)
                return currentValue;

            return parsedValue;
        }
        catch (...)
        {
            return currentValue;
        }
    }

    double DegreesToRadians(const double degrees)
    {
        return degrees * PI / 180.0;
    }

    float NormaliseHeading(float headingDegrees)
    {
        headingDegrees = std::fmod(headingDegrees, 360.0f);

        if (headingDegrees < 0.0f)
            headingDegrees += 360.0f;

        return headingDegrees;
    }

    char* SafeCopy(
        char* destination,
        const char* source,
        const std::size_t destinationSize)
    {
        if (!destination || destinationSize == 0)
            return destination;

        std::strncpy(destination, source ? source : "", destinationSize);
        destination[destinationSize - 1] = '\0';
        return destination;
    }

    LocalPosition CalculateRelativePosition(const TrafficState& state)
    {
        LocalPosition position
        {
            XPLMGetDatad(gLocalX),
            XPLMGetDatad(gLocalY),
            XPLMGetDatad(gLocalZ)
        };

        const double headingRadians =
            DegreesToRadians(XPLMGetDataf(gHeading));

        // Forward vector in X-Plane's local coordinate system.
        position.x +=
            std::sin(headingRadians) * state.forwardMetres;

        position.z -=
            std::cos(headingRadians) * state.forwardMetres;

        // Right vector, perpendicular to the forward vector.
        position.x +=
            std::cos(headingRadians) * state.rightMetres;

        position.z +=
            std::sin(headingRadians) * state.rightMetres;

        position.y += state.upMetres;
        return position;
    }

    bool IsTrafficFileFresh()
    {
#ifdef _WIN32
        struct _stat64 fileInformation = {};

        if (_stat64(
                gTrafficFilePath.c_str(),
                &fileInformation) != 0)
        {
            return false;
        }
#else
        struct stat fileInformation = {};

        if (stat(
                gTrafficFilePath.c_str(),
                &fileInformation) != 0)
        {
            return false;
        }
#endif

        const std::time_t currentTime = std::time(nullptr);
        const double ageSeconds =
            std::difftime(
                currentTime,
                fileInformation.st_mtime);

        return ageSeconds <= TRAFFIC_FEED_TIMEOUT_SECONDS;
    }

    int PreferencesCallback(
        const char*,
        const char* item,
        int defaultValue)
    {
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
        bClampToGround = true;
        ApplyIdentity(gTrafficState);
    }

    void ApplyIdentity(const TrafficState& state)
    {
        label = state.callsign;

        colLabel[0] = 0.0f;
        colLabel[1] = 1.0f;
        colLabel[2] = 0.0f;

        acRadar.code = 1200;
        acRadar.mode = xpmpTransponderMode_ModeC;

        SafeCopy(
            acInfoTexts.icaoAcType,
            state.icaoType.c_str(),
            sizeof(acInfoTexts.icaoAcType));

        SafeCopy(
            acInfoTexts.icaoAirline,
            state.airline.c_str(),
            sizeof(acInfoTexts.icaoAirline));

        SafeCopy(
            acInfoTexts.flightNum,
            state.callsign.c_str(),
            sizeof(acInfoTexts.flightNum));

        SafeCopy(
            acInfoTexts.tailNum,
            "AP-REMOTE",
            sizeof(acInfoTexts.tailNum));
    }

    void UpdatePosition(float, int) override
    {
        const TrafficState state = gTrafficState;

        if (state.worldMode)
        {
            SetLocation(
                state.latitude,
                state.longitude,
                state.altitudeFeet,
                state.onGround);

            SetHeading(NormaliseHeading(state.headingDegrees));
        }
        else
        {
            const LocalPosition localPosition =
                CalculateRelativePosition(state);

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

            SetLocation(
                latitude,
                longitude,
                elevationMetres / M_per_FT,
                state.onGround);

            SetHeading(
                NormaliseHeading(
                    XPLMGetDataf(gHeading) +
                    state.headingOffsetDegrees));
        }

        SetPitch(state.pitchDegrees);
        SetRoll(state.rollDegrees);

        SetGearRatio(
            std::clamp(state.gearRatio, 0.0f, 1.0f));

        SetFlapRatio(
            std::clamp(state.flapRatio, 0.0f, 1.0f));

        SetSpoilerRatio(0.0f);
        SetSpeedbrakeRatio(0.0f);
        SetSlatRatio(0.0f);

        SetThrustRatio(
            std::clamp(state.thrustRatio, 0.0f, 1.0f));

        SetLightsTaxi(state.taxiLight);
        SetLightsLanding(state.landingLight);
        SetLightsBeacon(state.beaconLight);
        SetLightsStrobe(state.strobeLight);
        SetLightsNav(state.navLight);

        SetTireDeflection(0.0f);
        SetTireRotAngle(0.0f);
        SetTireRotRpm(0.0f);

        SetEngineRotRpm(1, 1800.0f);
        SetEngineRotRpm(2, 1800.0f);

        SetThrustReversRatio(0.0f);
        SetReversDeployRatio(0.0f);
        SetTouchDown(state.onGround);
    }
};

namespace
{
    AeroPathAircraft* gRemoteAircraft = nullptr;

    bool IsAircraftCreated()
    {
        return gRemoteAircraft != nullptr;
    }

    bool ShouldRenderTraffic()
    {
        return
            gMasterEnabled &&
            gTrafficState.feedEnabled;
    }

    void UpdateMenuCheckmarks()
    {
        if (!gMenu)
            return;

        XPLMCheckMenuItem(
            gMenu,
            MENU_TRAFFIC_ENABLED,
            gMasterEnabled
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_AIRCRAFT_VISIBLE,
            gAircraftVisible
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_AI_CONTROL,
            XPMPHasControlOfAIAircraft()
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);
    }

    void CreateAircraft()
    {
        if (!ShouldRenderTraffic() || gRemoteAircraft)
            return;

        try
        {
            gRemoteAircraft = new AeroPathAircraft(
                gTrafficState.icaoType,
                gTrafficState.airline,
                gTrafficState.livery,
                0xAE0001);

            gRemoteAircraft->SetVisible(gAircraftVisible);

            const XPMP2::CSLModelInfo_t modelInfo =
                gRemoteAircraft->GetModelInfo();

            LogMessage(
                "AeroPath Traffic: Created %s/%s callsign '%s' using CSL model '%s'",
                gTrafficState.icaoType.c_str(),
                gTrafficState.airline.c_str(),
                gTrafficState.callsign.c_str(),
                modelInfo.modelName.c_str());
        }
        catch (const XPMP2::XPMP2Error& exception)
        {
            LogMessage(
                "AeroPath Traffic: Could not create remote aircraft: %s",
                exception.what());

            delete gRemoteAircraft;
            gRemoteAircraft = nullptr;
        }

        UpdateMenuCheckmarks();
    }

    void RemoveAircraft()
    {
        if (!gRemoteAircraft)
            return;

        delete gRemoteAircraft;
        gRemoteAircraft = nullptr;

        LogMessage("AeroPath Traffic: Remote aircraft removed");
        UpdateMenuCheckmarks();
    }

    void ApplyAircraftIdentity(
        const TrafficState& previousState,
        const TrafficState& newState)
    {
        if (!gRemoteAircraft)
            return;

        const bool modelChanged =
            previousState.icaoType != newState.icaoType ||
            previousState.airline != newState.airline ||
            previousState.livery != newState.livery;

        if (modelChanged)
        {
            gRemoteAircraft->ChangeModel(
                newState.icaoType,
                newState.airline,
                newState.livery);

            LogMessage(
                "AeroPath Traffic: Model request changed to %s/%s",
                newState.icaoType.c_str(),
                newState.airline.c_str());
        }

        if (modelChanged ||
            previousState.callsign != newState.callsign)
        {
            gRemoteAircraft->ApplyIdentity(newState);
        }
    }

    bool ReadTrafficFile(TrafficState& parsedState)
    {
        std::ifstream input(gTrafficFilePath);

        if (!input.is_open())
        {
            if (!gFeedFilePreviouslyMissing)
            {
                LogMessage(
                    "AeroPath Traffic: Feed file not found: %s",
                    gTrafficFilePath.c_str());

                gFeedFilePreviouslyMissing = true;
            }

            return false;
        }

        if (gFeedFilePreviouslyMissing)
        {
            LogMessage(
                "AeroPath Traffic: Feed file is available again");

            gFeedFilePreviouslyMissing = false;
        }

        if (!IsTrafficFileFresh())
        {
            if (!gFeedFilePreviouslyStale)
            {
                LogMessage(
                    "AeroPath Traffic: Feed is stale; removing aircraft until AeroPath resumes updates");

                gFeedFilePreviouslyStale = true;
            }

            return false;
        }

        if (gFeedFilePreviouslyStale)
        {
            LogMessage(
                "AeroPath Traffic: Live feed updates resumed");

            gFeedFilePreviouslyStale = false;
        }

        parsedState = gTrafficState;

        std::string line;
        while (std::getline(input, line))
        {
            line = Trim(line);

            if (line.empty() ||
                line[0] == '#' ||
                line[0] == ';')
            {
                continue;
            }

            const std::size_t equalsPosition = line.find('=');
            if (equalsPosition == std::string::npos)
                continue;

            const std::string key =
                ToLower(Trim(line.substr(0, equalsPosition)));

            const std::string value =
                Trim(line.substr(equalsPosition + 1));

            if (key == "enabled")
                parsedState.feedEnabled =
                    ParseBool(value, parsedState.feedEnabled);

            else if (key == "mode")
                parsedState.worldMode =
                    ToLower(value) == "world";

            else if (key == "on_ground")
                parsedState.onGround =
                    ParseBool(value, parsedState.onGround);

            else if (key == "icao")
                parsedState.icaoType = value;

            else if (key == "airline")
                parsedState.airline = value;

            else if (key == "livery")
                parsedState.livery = value;

            else if (key == "callsign")
                parsedState.callsign = value;

            else if (key == "forward_m")
                parsedState.forwardMetres =
                    ParseFloat(value, parsedState.forwardMetres);

            else if (key == "right_m")
                parsedState.rightMetres =
                    ParseFloat(value, parsedState.rightMetres);

            else if (key == "up_m")
                parsedState.upMetres =
                    ParseFloat(value, parsedState.upMetres);

            else if (key == "heading_offset_deg")
                parsedState.headingOffsetDegrees =
                    ParseFloat(
                        value,
                        parsedState.headingOffsetDegrees);

            else if (key == "latitude")
                parsedState.latitude =
                    ParseDouble(value, parsedState.latitude);

            else if (key == "longitude")
                parsedState.longitude =
                    ParseDouble(value, parsedState.longitude);

            else if (key == "altitude_ft")
                parsedState.altitudeFeet =
                    ParseDouble(value, parsedState.altitudeFeet);

            else if (key == "heading")
                parsedState.headingDegrees =
                    ParseFloat(value, parsedState.headingDegrees);

            else if (key == "pitch")
                parsedState.pitchDegrees =
                    ParseFloat(value, parsedState.pitchDegrees);

            else if (key == "roll")
                parsedState.rollDegrees =
                    ParseFloat(value, parsedState.rollDegrees);

            else if (key == "gear")
                parsedState.gearRatio =
                    ParseFloat(value, parsedState.gearRatio);

            else if (key == "flaps")
                parsedState.flapRatio =
                    ParseFloat(value, parsedState.flapRatio);

            else if (key == "thrust")
                parsedState.thrustRatio =
                    ParseFloat(value, parsedState.thrustRatio);

            else if (key == "taxi_light")
                parsedState.taxiLight =
                    ParseBool(value, parsedState.taxiLight);

            else if (key == "landing_light")
                parsedState.landingLight =
                    ParseBool(value, parsedState.landingLight);

            else if (key == "beacon_light")
                parsedState.beaconLight =
                    ParseBool(value, parsedState.beaconLight);

            else if (key == "strobe_light")
                parsedState.strobeLight =
                    ParseBool(value, parsedState.strobeLight);

            else if (key == "nav_light")
                parsedState.navLight =
                    ParseBool(value, parsedState.navLight);
        }

        if (parsedState.icaoType.empty())
            parsedState.icaoType = "A319";

        if (parsedState.callsign.empty())
            parsedState.callsign = "AEROPATH";

        return true;
    }

    void ReloadTrafficFeed()
    {
        TrafficState parsedState;

        if (!ReadTrafficFile(parsedState))
        {
            RemoveAircraft();
            return;
        }

        const TrafficState previousState = gTrafficState;
        gTrafficState = parsedState;

        if (ShouldRenderTraffic())
        {
            CreateAircraft();
            ApplyAircraftIdentity(previousState, gTrafficState);

            if (gRemoteAircraft)
                gRemoteAircraft->SetVisible(gAircraftVisible);
        }
        else
        {
            RemoveAircraft();
        }

        UpdateMenuCheckmarks();
    }

    float TrafficFileLoop(
        float,
        float,
        int,
        void*)
    {
        ReloadTrafficFeed();
        return TRAFFIC_POLL_INTERVAL_SECONDS;
    }

    void SetAircraftVisibility()
    {
        gAircraftVisible = !gAircraftVisible;

        if (gRemoteAircraft)
            gRemoteAircraft->SetVisible(gAircraftVisible);

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
            static_cast<MenuItem>(
                reinterpret_cast<unsigned long long>(
                    itemReference));

        switch (selectedItem)
        {
            case MENU_TRAFFIC_ENABLED:
                gMasterEnabled = !gMasterEnabled;

                if (ShouldRenderTraffic())
                    CreateAircraft();
                else
                    RemoveAircraft();

                break;

            case MENU_AIRCRAFT_VISIBLE:
                SetAircraftVisibility();
                break;

            case MENU_RELOAD_FEED:
                ReloadTrafficFeed();
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
        XPMP2::Aircraft* aircraft =
            XPMP2::AcFindByID(planeId);

        if (!aircraft)
            return;

        const char* notificationText =
            notification == xpmp_PlaneNotification_Created
                ? "created"
                : notification ==
                    xpmp_PlaneNotification_ModelChanged
                    ? "model changed"
                    : "destroyed";

        LogMessage(
            "AeroPath Traffic: Aircraft '%s', type %s, model %s - %s",
            aircraft->label.c_str(),
            aircraft->acIcaoType.c_str(),
            aircraft->GetModelName().c_str(),
            notificationText);
    }
}

PLUGIN_API int XPluginStart(
    char* outName,
    char* outSignature,
    char* outDescription)
{
    std::strcpy(outName, "AeroPath Traffic");
    std::strcpy(
        outSignature,
        "uk.co.aeropath.traffic");

    std::strcpy(
        outDescription,
        "AeroPath multiplayer aircraft rendering for X-Plane");

    XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);

    const int menuSlot =
        XPLMAppendMenuItem(
            XPLMFindPluginsMenu(),
            "AeroPath Traffic",
            nullptr,
            0);

    gMenu =
        XPLMCreateMenu(
            "AeroPath Traffic",
            XPLMFindPluginsMenu(),
            menuSlot,
            MenuCallback,
            nullptr);

    XPLMAppendMenuItem(
        gMenu,
        "Traffic Enabled",
        reinterpret_cast<void*>(
            MENU_TRAFFIC_ENABLED),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Aircraft Visible",
        reinterpret_cast<void*>(
            MENU_AIRCRAFT_VISIBLE),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Reload Traffic File",
        reinterpret_cast<void*>(
            MENU_RELOAD_FEED),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Toggle AI/TCAS Control",
        reinterpret_cast<void*>(
            MENU_AI_CONTROL),
        0);

#ifdef DEBUG
    XPLMAppendMenuItem(
        gMenu,
        "Reload Plugins",
        reinterpret_cast<void*>(
            MENU_RELOAD_PLUGINS),
        0);
#endif

    gLocalX =
        XPLMFindDataRef(
            "sim/flightmodel/position/local_x");

    gLocalY =
        XPLMFindDataRef(
            "sim/flightmodel/position/local_y");

    gLocalZ =
        XPLMFindDataRef(
            "sim/flightmodel/position/local_z");

    gHeading =
        XPLMFindDataRef(
            "sim/flightmodel/position/psi");

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
    const char pathSeparator =
        XPLMGetDirectorySeparator()[0];

    char pluginPath[1024] = {};

    XPLMGetPluginInfo(
        XPLMGetMyID(),
        nullptr,
        pluginPath,
        nullptr,
        nullptr);

    char* finalSeparator =
        std::strrchr(
            pluginPath,
            pathSeparator);

    if (!finalSeparator)
    {
        LogMessage(
            "AeroPath Traffic: Could not resolve plugin path");

        return 0;
    }

    *finalSeparator = '\0';

    finalSeparator =
        std::strrchr(
            pluginPath,
            pathSeparator);

    if (!finalSeparator)
    {
        LogMessage(
            "AeroPath Traffic: Could not resolve plugin root folder");

        return 0;
    }

    *(finalSeparator + 1) = '\0';

    std::string resourcePath = pluginPath;
    resourcePath += "Resources";

    gTrafficFilePath = resourcePath;
    gTrafficFilePath += pathSeparator;
    gTrafficFilePath += "AeroPathTraffic.txt";

    const char* result =
        XPMPMultiplayerInit(
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

    result =
        XPMPLoadCSLPackage(
            resourcePath.c_str());

    if (result[0])
    {
        LogMessage(
            "AeroPath Traffic: CSL loading reported: %s",
            result);
    }

    result =
        XPMPMultiplayerEnable(
            RequestAIAgain);

    if (result[0])
    {
        LogMessage(
            "AeroPath Traffic: AI/TCAS control unavailable: %s",
            result);
    }

    XPMPRegisterPlaneNotifierFunc(
        PlaneNotifier,
        nullptr);

    ReloadTrafficFeed();

    XPLMRegisterFlightLoopCallback(
        TrafficFileLoop,
        TRAFFIC_POLL_INTERVAL_SECONDS,
        nullptr);

    UpdateMenuCheckmarks();

    LogMessage(
        "AeroPath Traffic: Enabled; reading feed '%s'",
        gTrafficFilePath.c_str());

    return 1;
}

PLUGIN_API void XPluginDisable()
{
    XPLMUnregisterFlightLoopCallback(
        TrafficFileLoop,
        nullptr);

    RemoveAircraft();

    XPMPMultiplayerDisable();

    XPMPUnregisterPlaneNotifierFunc(
        PlaneNotifier,
        nullptr);

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
