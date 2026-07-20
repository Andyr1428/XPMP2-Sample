/// @file       XPMP2-Sample.cpp
/// @brief      AeroPath Traffic - multi-aircraft external traffic feed
/// @details    Renders multiple AeroPath-controlled aircraft through XPMP2 and
///             reads their states from Resources/AeroPathTraffic.txt.
///
///             Feed version 2 uses repeated [aircraft] blocks. The original
///             single-aircraft key/value feed remains supported so existing
///             development installations continue to work while the AeroPath
///             desktop writer is upgraded.
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
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>
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
    constexpr std::size_t MAX_REMOTE_AIRCRAFT = 100;

    XPLMMenuID gMenu = nullptr;

    bool gMasterEnabled = true;
    bool gAircraftVisible = true;
    bool gFeedEnabled = true;
    bool gFeedFilePreviouslyMissing = false;
    bool gFeedFilePreviouslyStale = false;

    XPLMDataRef gLocalX = nullptr;
    XPLMDataRef gLocalY = nullptr;
    XPLMDataRef gLocalZ = nullptr;
    XPLMDataRef gHeading = nullptr;

    std::string gTrafficFilePath;
    std::size_t gLastReportedTrafficCount = static_cast<std::size_t>(-1);

    struct LocalPosition
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct TrafficState
    {
        bool feedEnabled = true;
        bool worldMode = true;
        bool onGround = false;

        std::string uniqueId;
        std::string icaoType = "C172";
        std::string airline;
        std::string livery;
        std::string callsign = "AEROPATH";

        // Relative mode remains available for local development.
        float forwardMetres = 300.0f;
        float rightMetres = 80.0f;
        float upMetres = 40.0f;
        float headingOffsetDegrees = 0.0f;

        // World mode.
        double latitude = 0.0;
        double longitude = 0.0;
        double altitudeFeet = 0.0;
        float headingDegrees = 0.0f;

        // Shared attitude and aircraft configuration.
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

    struct ParsedTrafficFeed
    {
        bool enabled = true;
        std::unordered_map<std::string, TrafficState> aircraft;
    };

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

    std::string ToUpper(std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char character)
            {
                return static_cast<char>(std::toupper(character));
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

            if (parsedCharacters == 0 || !std::isfinite(parsedValue))
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

            if (parsedCharacters == 0 || !std::isfinite(parsedValue))
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

    bool IsValidWorldPosition(const TrafficState& state)
    {
        if (!std::isfinite(state.latitude) ||
            !std::isfinite(state.longitude) ||
            !std::isfinite(state.altitudeFeet))
        {
            return false;
        }

        if (state.latitude < -90.0 || state.latitude > 90.0 ||
            state.longitude < -180.0 || state.longitude > 180.0)
        {
            return false;
        }

        return
            std::abs(state.latitude) >= 0.0001 ||
            std::abs(state.longitude) >= 0.0001;
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

        position.x +=
            std::sin(headingRadians) * state.forwardMetres;

        position.z -=
            std::cos(headingRadians) * state.forwardMetres;

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

        return
            ageSeconds >= -5.0 &&
            ageSeconds <= TRAFFIC_FEED_TIMEOUT_SECONDS;
    }

    std::string BuildTrafficKey(
        const TrafficState& state,
        const std::size_t fallbackIndex)
    {
        std::string key = Trim(state.uniqueId);

        if (key.empty())
            key = Trim(state.callsign);

        if (key.empty())
            key = "AEROPATH-" + std::to_string(fallbackIndex + 1);

        return ToUpper(key);
    }

    XPMPPlaneID BuildModeSId(const std::string& trafficKey)
    {
        std::uint32_t hash = 2166136261u;

        for (const unsigned char character : trafficKey)
        {
            hash ^= character;
            hash *= 16777619u;
        }

        // Use an AeroPath-owned development range within the 24-bit Mode-S
        // value. This is only an internal XPMP identifier.
        std::uint32_t modeSId =
            0xA00000u |
            (hash & 0x0FFFFFu);

        if (modeSId == 0)
            modeSId = 0xA00001u;

        return static_cast<XPMPPlaneID>(modeSId);
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
        const TrafficState& initialState,
        XPMPPlaneID modeSId)
        : Aircraft(
            initialState.icaoType,
            initialState.airline,
            initialState.livery,
            modeSId),
          state_(initialState)
    {
        bClampToGround = true;
        ApplyIdentity(state_);
    }

    const TrafficState& State() const
    {
        return state_;
    }

    void ApplyState(const TrafficState& nextState)
    {
        const bool modelChanged =
            state_.icaoType != nextState.icaoType ||
            state_.airline != nextState.airline ||
            state_.livery != nextState.livery;

        const bool identityChanged =
            modelChanged ||
            state_.callsign != nextState.callsign ||
            state_.uniqueId != nextState.uniqueId;

        state_ = nextState;

        if (modelChanged)
        {
            ChangeModel(
                state_.icaoType,
                state_.airline,
                state_.livery);
        }

        if (identityChanged)
            ApplyIdentity(state_);
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

        const std::string tailNumber =
            state.uniqueId.empty()
                ? "AP-REMOTE"
                : state.uniqueId;

        SafeCopy(
            acInfoTexts.tailNum,
            tailNumber.c_str(),
            sizeof(acInfoTexts.tailNum));
    }

    void UpdatePosition(float, int) override
    {
        const TrafficState state = state_;

        if (state.worldMode)
        {
            SetLocation(
                state.latitude,
                state.longitude,
                state.altitudeFeet,
                state.onGround);

            SetHeading(
                NormaliseHeading(state.headingDegrees));
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

private:
    TrafficState state_;
};

namespace
{
    std::unordered_map<
        std::string,
        std::unique_ptr<AeroPathAircraft>> gRemoteAircraft;

    bool ShouldRenderTraffic()
    {
        return
            gMasterEnabled &&
            gFeedEnabled;
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

    void RemoveAircraft(const std::string& key)
    {
        const auto iterator =
            gRemoteAircraft.find(key);

        if (iterator == gRemoteAircraft.end())
            return;

        const std::string callsign =
            iterator->second
                ? iterator->second->State().callsign
                : key;

        gRemoteAircraft.erase(iterator);

        LogMessage(
            "AeroPath Traffic: Removed remote aircraft '%s'",
            callsign.c_str());
    }

    void RemoveAllAircraft()
    {
        if (gRemoteAircraft.empty())
            return;

        const std::size_t removedCount =
            gRemoteAircraft.size();

        gRemoteAircraft.clear();

        LogMessage(
            "AeroPath Traffic: Removed %zu remote aircraft",
            removedCount);

        gLastReportedTrafficCount = 0;
        UpdateMenuCheckmarks();
    }

    void CreateAircraft(
        const std::string& key,
        const TrafficState& state)
    {
        if (!ShouldRenderTraffic() ||
            gRemoteAircraft.find(key) != gRemoteAircraft.end())
        {
            return;
        }

        try
        {
            auto aircraft =
                std::make_unique<AeroPathAircraft>(
                    state,
                    BuildModeSId(key));

            aircraft->SetVisible(gAircraftVisible);

            const XPMP2::CSLModelInfo_t modelInfo =
                aircraft->GetModelInfo();

            LogMessage(
                "AeroPath Traffic: Created %s/%s callsign '%s' using CSL model '%s'",
                state.icaoType.c_str(),
                state.airline.c_str(),
                state.callsign.c_str(),
                modelInfo.modelName.c_str());

            gRemoteAircraft.emplace(
                key,
                std::move(aircraft));
        }
        catch (const XPMP2::XPMP2Error& exception)
        {
            LogMessage(
                "AeroPath Traffic: Could not create remote aircraft '%s': %s",
                state.callsign.c_str(),
                exception.what());
        }
        catch (const std::exception& exception)
        {
            LogMessage(
                "AeroPath Traffic: Could not create remote aircraft '%s': %s",
                state.callsign.c_str(),
                exception.what());
        }
    }

    void ApplyStateValue(
        TrafficState& state,
        const std::string& key,
        const std::string& value)
    {
        if (key == "enabled")
            state.feedEnabled =
                ParseBool(value, state.feedEnabled);

        else if (key == "id" ||
                 key == "pilot_id" ||
                 key == "traffic_id")
            state.uniqueId = value;

        else if (key == "mode")
            state.worldMode =
                ToLower(value) == "world";

        else if (key == "on_ground")
            state.onGround =
                ParseBool(value, state.onGround);

        else if (key == "icao")
            state.icaoType = ToUpper(value);

        else if (key == "airline")
            state.airline = ToUpper(value);

        else if (key == "livery")
            state.livery = value;

        else if (key == "callsign")
            state.callsign = value;

        else if (key == "forward_m")
            state.forwardMetres =
                ParseFloat(value, state.forwardMetres);

        else if (key == "right_m")
            state.rightMetres =
                ParseFloat(value, state.rightMetres);

        else if (key == "up_m")
            state.upMetres =
                ParseFloat(value, state.upMetres);

        else if (key == "heading_offset_deg")
            state.headingOffsetDegrees =
                ParseFloat(
                    value,
                    state.headingOffsetDegrees);

        else if (key == "latitude")
            state.latitude =
                ParseDouble(value, state.latitude);

        else if (key == "longitude")
            state.longitude =
                ParseDouble(value, state.longitude);

        else if (key == "altitude_ft")
            state.altitudeFeet =
                ParseDouble(value, state.altitudeFeet);

        else if (key == "heading")
            state.headingDegrees =
                ParseFloat(value, state.headingDegrees);

        else if (key == "pitch")
            state.pitchDegrees =
                ParseFloat(value, state.pitchDegrees);

        else if (key == "roll")
            state.rollDegrees =
                ParseFloat(value, state.rollDegrees);

        else if (key == "gear")
            state.gearRatio =
                ParseFloat(value, state.gearRatio);

        else if (key == "flaps")
            state.flapRatio =
                ParseFloat(value, state.flapRatio);

        else if (key == "thrust")
            state.thrustRatio =
                ParseFloat(value, state.thrustRatio);

        else if (key == "taxi_light")
            state.taxiLight =
                ParseBool(value, state.taxiLight);

        else if (key == "landing_light")
            state.landingLight =
                ParseBool(value, state.landingLight);

        else if (key == "beacon_light")
            state.beaconLight =
                ParseBool(value, state.beaconLight);

        else if (key == "strobe_light")
            state.strobeLight =
                ParseBool(value, state.strobeLight);

        else if (key == "nav_light")
            state.navLight =
                ParseBool(value, state.navLight);
    }

    void AddParsedAircraft(
        ParsedTrafficFeed& parsedFeed,
        TrafficState state,
        const std::size_t fallbackIndex)
    {
        if (!state.feedEnabled)
            return;

        state.uniqueId = Trim(state.uniqueId);
        state.callsign = Trim(state.callsign);
        state.icaoType = ToUpper(Trim(state.icaoType));
        state.airline = ToUpper(Trim(state.airline));
        state.livery = Trim(state.livery);

        if (state.icaoType.empty())
            state.icaoType = "C172";

        if (state.callsign.empty())
            state.callsign =
                state.uniqueId.empty()
                    ? "AEROPATH"
                    : state.uniqueId;

        const std::string key =
            BuildTrafficKey(state, fallbackIndex);

        if (state.uniqueId.empty())
            state.uniqueId = key;

        if (state.worldMode &&
            !IsValidWorldPosition(state))
        {
            LogMessage(
                "AeroPath Traffic: Ignoring '%s' because its world position is invalid",
                state.callsign.c_str());

            return;
        }

        parsedFeed.aircraft[key] = state;
    }

    bool ReadTrafficFile(ParsedTrafficFeed& parsedFeed)
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

        parsedFeed = ParsedTrafficFeed{};

        bool insideAircraftBlock = false;
        bool currentBlockHasValues = false;
        bool legacyStateHasValues = false;

        TrafficState currentBlockState;
        TrafficState legacyState;

        std::size_t parsedIndex = 0;
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

            const std::string lowerLine =
                ToLower(line);

            if (lowerLine == "[aircraft]")
            {
                if (insideAircraftBlock &&
                    currentBlockHasValues &&
                    parsedFeed.aircraft.size() < MAX_REMOTE_AIRCRAFT)
                {
                    AddParsedAircraft(
                        parsedFeed,
                        currentBlockState,
                        parsedIndex++);
                }

                insideAircraftBlock = true;
                currentBlockHasValues = false;
                currentBlockState = TrafficState{};
                currentBlockState.worldMode = true;
                continue;
            }

            if (lowerLine == "[/aircraft]")
            {
                if (insideAircraftBlock &&
                    currentBlockHasValues &&
                    parsedFeed.aircraft.size() < MAX_REMOTE_AIRCRAFT)
                {
                    AddParsedAircraft(
                        parsedFeed,
                        currentBlockState,
                        parsedIndex++);
                }

                insideAircraftBlock = false;
                currentBlockHasValues = false;
                currentBlockState = TrafficState{};
                continue;
            }

            const std::size_t equalsPosition =
                line.find('=');

            if (equalsPosition == std::string::npos)
                continue;

            const std::string key =
                ToLower(
                    Trim(
                        line.substr(
                            0,
                            equalsPosition)));

            const std::string value =
                Trim(
                    line.substr(
                        equalsPosition + 1));

            if (!insideAircraftBlock &&
                key == "enabled")
            {
                parsedFeed.enabled =
                    ParseBool(
                        value,
                        parsedFeed.enabled);

                continue;
            }

            if (!insideAircraftBlock &&
                (key == "version" ||
                 key == "count" ||
                 key == "generated_at"))
            {
                continue;
            }

            if (insideAircraftBlock)
            {
                ApplyStateValue(
                    currentBlockState,
                    key,
                    value);

                currentBlockHasValues = true;
            }
            else
            {
                // Backward-compatible single-aircraft feed.
                ApplyStateValue(
                    legacyState,
                    key,
                    value);

                legacyStateHasValues = true;
            }
        }

        if (insideAircraftBlock &&
            currentBlockHasValues &&
            parsedFeed.aircraft.size() < MAX_REMOTE_AIRCRAFT)
        {
            AddParsedAircraft(
                parsedFeed,
                currentBlockState,
                parsedIndex++);
        }

        if (legacyStateHasValues &&
            parsedFeed.aircraft.empty())
        {
            AddParsedAircraft(
                parsedFeed,
                legacyState,
                parsedIndex);
        }

        return true;
    }

    void SynchroniseAircraft(
        const ParsedTrafficFeed& parsedFeed)
    {
        gFeedEnabled = parsedFeed.enabled;

        if (!ShouldRenderTraffic())
        {
            RemoveAllAircraft();
            return;
        }

        std::unordered_set<std::string> desiredKeys;

        for (const auto& entry : parsedFeed.aircraft)
        {
            desiredKeys.insert(entry.first);
        }

        std::vector<std::string> keysToRemove;

        for (const auto& entry : gRemoteAircraft)
        {
            if (desiredKeys.find(entry.first) ==
                desiredKeys.end())
            {
                keysToRemove.push_back(entry.first);
            }
        }

        for (const std::string& key : keysToRemove)
            RemoveAircraft(key);

        for (const auto& entry : parsedFeed.aircraft)
        {
            const std::string& key = entry.first;
            const TrafficState& state = entry.second;

            const auto existing =
                gRemoteAircraft.find(key);

            if (existing == gRemoteAircraft.end())
            {
                CreateAircraft(key, state);
                continue;
            }

            try
            {
                const TrafficState previousState =
                    existing->second->State();

                existing->second->ApplyState(state);
                existing->second->SetVisible(gAircraftVisible);

                const bool modelChanged =
                    previousState.icaoType != state.icaoType ||
                    previousState.airline != state.airline ||
                    previousState.livery != state.livery;

                if (modelChanged)
                {
                    LogMessage(
                        "AeroPath Traffic: Model request for '%s' changed to %s/%s",
                        state.callsign.c_str(),
                        state.icaoType.c_str(),
                        state.airline.c_str());
                }
            }
            catch (const XPMP2::XPMP2Error& exception)
            {
                LogMessage(
                    "AeroPath Traffic: Could not update '%s': %s",
                    state.callsign.c_str(),
                    exception.what());
            }
            catch (const std::exception& exception)
            {
                LogMessage(
                    "AeroPath Traffic: Could not update '%s': %s",
                    state.callsign.c_str(),
                    exception.what());
            }
        }

        if (gLastReportedTrafficCount !=
            gRemoteAircraft.size())
        {
            gLastReportedTrafficCount =
                gRemoteAircraft.size();

            LogMessage(
                "AeroPath Traffic: %zu remote aircraft active",
                gLastReportedTrafficCount);
        }

        UpdateMenuCheckmarks();
    }

    void ReloadTrafficFeed()
    {
        ParsedTrafficFeed parsedFeed;

        if (!ReadTrafficFile(parsedFeed))
        {
            gFeedEnabled = false;
            RemoveAllAircraft();
            return;
        }

        SynchroniseAircraft(parsedFeed);
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

        for (auto& entry : gRemoteAircraft)
        {
            if (entry.second)
                entry.second->SetVisible(gAircraftVisible);
        }

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

                if (gMasterEnabled)
                    ReloadTrafficFeed();
                else
                    RemoveAllAircraft();

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
    RemoveAllAircraft();

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
            "C172");

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

    gFeedEnabled = true;
    gLastReportedTrafficCount = static_cast<std::size_t>(-1);

    ReloadTrafficFeed();

    XPLMRegisterFlightLoopCallback(
        TrafficFileLoop,
        TRAFFIC_POLL_INTERVAL_SECONDS,
        nullptr);

    UpdateMenuCheckmarks();

    LogMessage(
        "AeroPath Traffic: Enabled; reading multi-aircraft feed '%s'",
        gTrafficFilePath.c_str());

    return 1;
}

PLUGIN_API void XPluginDisable()
{
    XPLMUnregisterFlightLoopCallback(
        TrafficFileLoop,
        nullptr);

    RemoveAllAircraft();

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
