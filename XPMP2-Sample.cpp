/// @file       XPMP2-Sample.cpp
/// @brief      AeroPath Traffic - multi-aircraft external traffic feed
/// @details    Renders multiple AeroPath-controlled aircraft through XPMP2 and
///             reads their states from Resources/AeroPathTraffic.txt.
///
///             Feed version 5 uses repeated [aircraft] blocks, explicit aircraft/operator/livery identity,
///             motion data and the server's last-seen timestamp. The original
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
#include <filesystem>
#include <sstream>
#include <system_error>
#include <sys/stat.h>

#ifdef _WIN32
    #include <windows.h>
    #include <shobjidl.h>
    #include <shellapi.h>
#endif

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
#include "AeroPathTaxiGuidance.h"

#if !XPLM300
    #error This plugin requires X-Plane SDK 3.0 or newer
#endif

using namespace XPMP2;

namespace
{
    constexpr double PI = 3.141592653589793238462643383279502884;
    constexpr float TRAFFIC_POLL_INTERVAL_SECONDS = 0.10f;
    constexpr double TRAFFIC_FEED_TIMEOUT_SECONDS = 15.0;
    constexpr double TRAFFIC_FILE_GRACE_SECONDS = 2.0;
    constexpr double MIN_INTERPOLATION_SECONDS = 0.10;
    constexpr double DEFAULT_INTERPOLATION_SECONDS = 0.24;
    constexpr double MAX_INTERPOLATION_SECONDS = 0.80;
    constexpr double INTERPOLATION_BUFFER_FACTOR = 0.90;
    constexpr double MAX_EXTRAPOLATION_SECONDS = 1.75;
    constexpr double EARTH_RADIUS_METRES = 6371000.0;
    constexpr double KNOTS_TO_METRES_PER_SECOND = 0.5144444444444445;
    constexpr double MAX_GROUND_TRACK_SPEED_METRES_PER_SECOND = 90.0;
    constexpr double MAX_AIRBORNE_TRACK_SPEED_METRES_PER_SECOND = 430.0;
    constexpr double VELOCITY_BLEND_FACTOR = 0.45;
    constexpr double TIMING_LOG_INTERVAL_SECONDS = 10.0;
    constexpr std::size_t MAX_REMOTE_AIRCRAFT = 100;
    constexpr float DEFAULT_TRAFFIC_LABEL_DISTANCE_NM = 100.0f;


    XPLMMenuID gMenu = nullptr;

    bool gMasterEnabled = true;
    bool gAircraftVisible = true;
    bool gTrafficLabelsEnabled = true;
    float gTrafficLabelDistanceNm =
        DEFAULT_TRAFFIC_LABEL_DISTANCE_NM;
    bool gFeedEnabled = true;
    bool gFeedFilePreviouslyMissing = false;
    bool gFeedFilePreviouslyStale = false;
    double gFeedUnavailableSinceSeconds = -1.0;


    XPLMDataRef gLocalX = nullptr;
    XPLMDataRef gLocalY = nullptr;
    XPLMDataRef gLocalZ = nullptr;
    XPLMDataRef gHeading = nullptr;

    std::string gTrafficFilePath;
    std::string gResourcePath;
    std::string gCslConfigPath;
    std::string gBundledCslPath;
    std::string gAeroPathModelsPath;
    std::string gConfiguredExternalCslPath;
    std::string gLoadedCslPath;
    std::size_t gLastReportedTrafficCount = static_cast<std::size_t>(-1);
    std::string gLastProcessedFeedGeneration;
    double gLastNewFeedSeconds = -1.0;
    double gLastFeedIntervalSeconds = 0.0;
    double gLastTimingLogSeconds = 0.0;
    std::uint64_t gXpmpPositionUpdateCount = 0;
    std::uint64_t gLastXpmpPositionUpdateCount = 0;

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
        std::string airlineName;
        std::string livery;
        std::string registration;
        std::string flightNumber;
        std::string callsign = "AEROPATH";

        bool labelConfigured = false;
        std::string labelText;
        float labelRed = 1.0f;
        float labelGreen = 1.0f;
        float labelBlue = 1.0f;

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
        double groundSpeedKnots = 0.0;
        double verticalSpeedFpm = 0.0;
        std::string lastSeenAt;

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
        std::string generatedAt;
        std::unordered_map<std::string, TrafficState> aircraft;
    };

    enum MenuItem
    {
        MENU_TRAFFIC_ENABLED = 0,
        MENU_AIRCRAFT_VISIBLE,
        MENU_TRAFFIC_LABELS_OFF,
        MENU_TRAFFIC_LABELS_20_NM,
        MENU_TRAFFIC_LABELS_50_NM,
        MENU_TRAFFIC_LABELS_100_NM,
        MENU_CSL_LOCATE_FOLDER,
        MENU_CSL_USE_BUNDLED,
        MENU_CSL_VALIDATE_FOLDER,
        MENU_CSL_OPEN_FOLDER,
        MENU_RELOAD_FEED,
        MENU_TAXI_ROUTE_LIGHTS,
        MENU_TAXI_REBUILD_ROUTE,
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

    struct CslValidationResult
    {
        bool valid = false;
        std::size_t packageCount = 0;
        std::string message;
    };

    std::string JoinPath(
        const std::string& first,
        const std::string& second)
    {
        if (first.empty())
            return second;

        return (
            std::filesystem::u8path(first) /
            std::filesystem::u8path(second))
            .lexically_normal()
            .u8string();
    }

    CslValidationResult ValidateCslFolder(
        const std::string& folderPath)
    {
        CslValidationResult result;
        const std::string cleanPath = Trim(folderPath);

        if (cleanPath.empty())
        {
            result.message = "No CSL folder has been selected.";
            return result;
        }

        const std::filesystem::path root =
            std::filesystem::u8path(cleanPath);

        std::error_code error;

        if (!std::filesystem::exists(root, error) ||
            error ||
            !std::filesystem::is_directory(root, error) ||
            error)
        {
            result.message =
                "The selected folder does not exist or cannot be read.";
            return result;
        }

        std::filesystem::recursive_directory_iterator iterator(
            root,
            std::filesystem::directory_options::skip_permission_denied,
            error);

        const std::filesystem::recursive_directory_iterator end;

        while (!error && iterator != end)
        {
            const std::filesystem::directory_entry& entry = *iterator;
            std::error_code entryError;

            if (entry.is_regular_file(entryError) &&
                !entryError &&
                ToLower(entry.path().filename().u8string()) ==
                    "xsb_aircraft.txt")
            {
                ++result.packageCount;
            }

            iterator.increment(error);

            if (error == std::errc::permission_denied)
                error.clear();
        }

        if (result.packageCount == 0)
        {
            result.message =
                "No xsb_aircraft.txt CSL package definitions were found in this folder or its subfolders.";
            return result;
        }

        result.valid = true;
        result.message =
            std::to_string(result.packageCount) +
            (result.packageCount == 1
                ? " CSL package was found."
                : " CSL packages were found.");

        return result;
    }

    void LoadCslConfiguration()
    {
        gConfiguredExternalCslPath.clear();

        if (gCslConfigPath.empty())
            return;

        std::ifstream input(gCslConfigPath);

        if (!input.is_open())
            return;

        bool insideCslSection = false;
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

            if (line.front() == '[' &&
                line.back() == ']')
            {
                insideCslSection =
                    ToLower(
                        Trim(
                            line.substr(
                                1,
                                line.size() - 2))) ==
                    "csl";

                continue;
            }

            if (!insideCslSection)
                continue;

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

            if (key != "path")
                continue;

            gConfiguredExternalCslPath =
                Trim(
                    line.substr(
                        equalsPosition + 1));

            break;
        }
    }

    bool SaveCslConfiguration(
        const std::string& externalPath)
    {
        if (gCslConfigPath.empty())
            return false;

        std::ofstream output(
            gCslConfigPath,
            std::ios::out | std::ios::trunc);

        if (!output.is_open())
            return false;

        output << "[CSL]\n";
        output << "Path=" << Trim(externalPath) << "\n";
        output << "UseBundledFallback=true\n";
        output.flush();

        return output.good();
    }

#ifdef _WIN32
    std::wstring Utf8ToWide(const std::string& value)
    {
        if (value.empty())
            return {};

        const int requiredCharacters =
            MultiByteToWideChar(
                CP_UTF8,
                MB_ERR_INVALID_CHARS,
                value.c_str(),
                -1,
                nullptr,
                0);

        if (requiredCharacters <= 0)
            return std::wstring(value.begin(), value.end());

        std::wstring result(
            static_cast<std::size_t>(requiredCharacters),
            L'\0');

        MultiByteToWideChar(
            CP_UTF8,
            MB_ERR_INVALID_CHARS,
            value.c_str(),
            -1,
            result.data(),
            requiredCharacters);

        if (!result.empty() && result.back() == L'\0')
            result.pop_back();

        return result;
    }

    std::string WideToUtf8(const std::wstring& value)
    {
        if (value.empty())
            return {};

        const int requiredBytes =
            WideCharToMultiByte(
                CP_UTF8,
                0,
                value.c_str(),
                -1,
                nullptr,
                0,
                nullptr,
                nullptr);

        if (requiredBytes <= 0)
            return std::string(value.begin(), value.end());

        std::string result(
            static_cast<std::size_t>(requiredBytes),
            '\0');

        WideCharToMultiByte(
            CP_UTF8,
            0,
            value.c_str(),
            -1,
            result.data(),
            requiredBytes,
            nullptr,
            nullptr);

        if (!result.empty() && result.back() == '\0')
            result.pop_back();

        return result;
    }

    void ShowCslMessage(
        const std::string& title,
        const std::string& message,
        const bool isError = false)
    {
        MessageBoxW(
            nullptr,
            Utf8ToWide(message).c_str(),
            Utf8ToWide(title).c_str(),
            MB_OK |
                (isError
                    ? MB_ICONERROR
                    : MB_ICONINFORMATION) |
                MB_SETFOREGROUND);
    }

    std::string SelectCslFolder(
        const std::string& initialFolder)
    {
        const HRESULT initialiseResult =
            CoInitializeEx(
                nullptr,
                COINIT_APARTMENTTHREADED |
                    COINIT_DISABLE_OLE1DDE);

        const bool uninitialiseCom =
            SUCCEEDED(initialiseResult);

        if (FAILED(initialiseResult) &&
            initialiseResult != RPC_E_CHANGED_MODE)
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "Windows could not initialise the CSL folder browser.",
                true);

            return {};
        }

        IFileOpenDialog* dialog = nullptr;

        HRESULT result =
            CoCreateInstance(
                CLSID_FileOpenDialog,
                nullptr,
                CLSCTX_INPROC_SERVER,
                IID_PPV_ARGS(&dialog));

        if (FAILED(result) || !dialog)
        {
            if (uninitialiseCom)
                CoUninitialize();

            ShowCslMessage(
                "AeroPath Traffic",
                "Windows could not open the CSL folder browser.",
                true);

            return {};
        }

        DWORD options = 0;
        dialog->GetOptions(&options);
        dialog->SetOptions(
            options |
            FOS_PICKFOLDERS |
            FOS_FORCEFILESYSTEM |
            FOS_PATHMUSTEXIST);
        dialog->SetTitle(
            L"Locate your CSL model library");

        if (!Trim(initialFolder).empty())
        {
            IShellItem* initialItem = nullptr;
            const std::wstring initialFolderWide =
                Utf8ToWide(initialFolder);

            if (SUCCEEDED(
                    SHCreateItemFromParsingName(
                        initialFolderWide.c_str(),
                        nullptr,
                        IID_PPV_ARGS(&initialItem))) &&
                initialItem)
            {
                dialog->SetFolder(initialItem);
                initialItem->Release();
            }
        }

        std::string selectedFolder;
        result = dialog->Show(nullptr);

        if (SUCCEEDED(result))
        {
            IShellItem* selectedItem = nullptr;

            if (SUCCEEDED(dialog->GetResult(&selectedItem)) &&
                selectedItem)
            {
                PWSTR selectedPath = nullptr;

                if (SUCCEEDED(
                        selectedItem->GetDisplayName(
                            SIGDN_FILESYSPATH,
                            &selectedPath)) &&
                    selectedPath)
                {
                    selectedFolder =
                        WideToUtf8(selectedPath);

                    CoTaskMemFree(selectedPath);
                }

                selectedItem->Release();
            }
        }

        dialog->Release();

        if (uninitialiseCom)
            CoUninitialize();

        return selectedFolder;
    }

    bool OpenFolderInExplorer(
        const std::string& folderPath)
    {
        if (Trim(folderPath).empty())
            return false;

        const HINSTANCE result =
            ShellExecuteW(
                nullptr,
                L"open",
                Utf8ToWide(folderPath).c_str(),
                nullptr,
                nullptr,
                SW_SHOWNORMAL);

        return reinterpret_cast<std::intptr_t>(result) > 32;
    }
#else
    void ShowCslMessage(
        const std::string& title,
        const std::string& message,
        const bool = false)
    {
        LogMessage(
            "%s: %s",
            title.c_str(),
            message.c_str());
    }

    std::string SelectCslFolder(const std::string&)
    {
        ShowCslMessage(
            "AeroPath Traffic",
            "The in-plugin CSL folder browser is currently available on Windows only.",
            true);

        return {};
    }

    bool OpenFolderInExplorer(const std::string&)
    {
        return false;
    }
#endif

    std::string ConfiguredCslFolder()
    {
        return gConfiguredExternalCslPath.empty()
            ? gBundledCslPath
            : gConfiguredExternalCslPath;
    }

    void LocateCslFolder()
    {
        if (gResourcePath.empty())
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "AeroPath Traffic must be enabled before a CSL folder can be selected.",
                true);

            return;
        }

        std::string initialFolder =
            ConfiguredCslFolder();

        if (!ValidateCslFolder(initialFolder).valid)
            initialFolder = gResourcePath;

        const std::string selectedFolder =
            SelectCslFolder(initialFolder);

        if (selectedFolder.empty())
            return;

        const CslValidationResult validation =
            ValidateCslFolder(selectedFolder);

        if (!validation.valid)
        {
            ShowCslMessage(
                "Invalid CSL folder",
                validation.message,
                true);

            LogMessage(
                "AeroPath Traffic: Rejected CSL folder '%s': %s",
                selectedFolder.c_str(),
                validation.message.c_str());

            return;
        }

        if (!SaveCslConfiguration(selectedFolder))
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "The selected CSL folder is valid, but AeroPath could not save AeroPathTraffic.ini.",
                true);

            return;
        }

        gConfiguredExternalCslPath = selectedFolder;

        std::ostringstream message;
        message << "Selected CSL library:\n\n"
                << selectedFolder << "\n\n"
                << validation.packageCount
                << (validation.packageCount == 1
                    ? " package was found."
                    : " packages were found.")
                << "\n\nRestart X-Plane, or disable and re-enable AeroPath Traffic, to load this library.";

        ShowCslMessage(
            "AeroPath Traffic CSL library",
            message.str());

        LogMessage(
            "AeroPath Traffic: External CSL folder selected '%s' (%zu packages); reload required",
            selectedFolder.c_str(),
            validation.packageCount);
    }

    void UseBundledCslFolder()
    {
        if (gResourcePath.empty())
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "AeroPath Traffic must be enabled before the CSL setting can be changed.",
                true);

            return;
        }

        if (!SaveCslConfiguration(""))
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "AeroPath could not save the bundled CSL selection to AeroPathTraffic.ini.",
                true);

            return;
        }

        gConfiguredExternalCslPath.clear();

        const CslValidationResult validation =
            ValidateCslFolder(gBundledCslPath);

        std::ostringstream message;
        message << "AeroPath's bundled CSL library has been selected.";

        if (validation.valid)
        {
            message << "\n\n"
                    << validation.packageCount
                    << (validation.packageCount == 1
                        ? " package was found."
                        : " packages were found.");
        }

        message << "\n\nRestart X-Plane, or disable and re-enable AeroPath Traffic, to apply the change.";

        ShowCslMessage(
            "AeroPath Traffic CSL library",
            message.str());

        LogMessage(
            "AeroPath Traffic: Bundled CSL library selected; reload required");
    }

    void ValidateConfiguredCslFolder()
    {
        const std::string selectedFolder =
            ConfiguredCslFolder();

        const CslValidationResult validation =
            ValidateCslFolder(selectedFolder);

        std::ostringstream message;
        message << "Selected folder:\n\n"
                << (selectedFolder.empty()
                    ? "No folder selected"
                    : selectedFolder)
                << "\n\n"
                << validation.message;

        ShowCslMessage(
            validation.valid
                ? "CSL folder is valid"
                : "CSL folder is invalid",
            message.str(),
            !validation.valid);

        LogMessage(
            "AeroPath Traffic: CSL validation for '%s': %s",
            selectedFolder.c_str(),
            validation.message.c_str());
    }

    void OpenConfiguredCslFolder()
    {
        const std::string selectedFolder =
            ConfiguredCslFolder();

        const CslValidationResult validation =
            ValidateCslFolder(selectedFolder);

        if (!validation.valid)
        {
            ShowCslMessage(
                "AeroPath Traffic",
                validation.message,
                true);

            return;
        }

        if (!OpenFolderInExplorer(selectedFolder))
        {
            ShowCslMessage(
                "AeroPath Traffic",
                "The selected CSL folder could not be opened.",
                true);
        }
    }

    std::string ResolveCslFolderForStartup()
    {
        LoadCslConfiguration();

        if (!gConfiguredExternalCslPath.empty())
        {
            const CslValidationResult externalValidation =
                ValidateCslFolder(
                    gConfiguredExternalCslPath);

            if (externalValidation.valid)
            {
                LogMessage(
                    "AeroPath Traffic: Loading external CSL library '%s' (%zu packages)",
                    gConfiguredExternalCslPath.c_str(),
                    externalValidation.packageCount);

                return gConfiguredExternalCslPath;
            }

            LogMessage(
                "AeroPath Traffic: External CSL library '%s' is unavailable: %s; using bundled fallback",
                gConfiguredExternalCslPath.c_str(),
                externalValidation.message.c_str());
        }

        const CslValidationResult bundledValidation =
            ValidateCslFolder(gBundledCslPath);

        if (bundledValidation.valid)
        {
            LogMessage(
                "AeroPath Traffic: Loading bundled CSL library '%s' (%zu packages)",
                gBundledCslPath.c_str(),
                bundledValidation.packageCount);

            return gBundledCslPath;
        }

        const CslValidationResult legacyValidation =
            ValidateCslFolder(gResourcePath);

        const CslValidationResult aeroPathValidation =
            ValidateCslFolder(gAeroPathModelsPath);

        if (legacyValidation.valid)
        {
            /*
             * Resources now contains AeroPathModels, which is loaded separately.
             * If every xsb_aircraft.txt found under Resources belongs to that
             * standalone catalogue, do not load Resources again or XPMP2 will
             * see the same models twice.
             */
            if (aeroPathValidation.valid &&
                legacyValidation.packageCount <=
                    aeroPathValidation.packageCount)
            {
                LogMessage(
                    "AeroPath Traffic: No separate legacy CSL packages found under Resources; standalone AeroPathModels already loaded");

                return {};
            }

            LogMessage(
                "AeroPath Traffic: Bundled CSL subfolder was not found; loading legacy Resources root '%s' (%zu packages)",
                gResourcePath.c_str(),
                legacyValidation.packageCount);

            return gResourcePath;
        }

        LogMessage(
            "AeroPath Traffic: No external, bundled or legacy CSL fallback packages were found; standalone AeroPathModels remain available");

        return {};
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

    void ApplyTrafficLabelSettings()
    {
        XPMPEnableAircraftLabels(
            gTrafficLabelsEnabled);

        if (gTrafficLabelsEnabled)
        {
            /*
             * Keep labels visible at the selected AeroPath range even when
             * X-Plane reports reduced weather visibility. Labels are still
             * drawn only for aircraft inside the current camera view.
             */
            XPMPSetAircraftLabelDist(
                gTrafficLabelDistanceNm,
                false);

            LogMessage(
                "AeroPath Traffic: Aircraft labels enabled to %.0f NM",
                static_cast<double>(
                    gTrafficLabelDistanceNm));
        }
        else
        {
            LogMessage(
                "AeroPath Traffic: Aircraft labels disabled");
        }
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

        interpolationFrom_ = initialState;
        interpolationTo_ = initialState;

        const double nowSeconds = XPLMGetElapsedTime();
        interpolationStartSeconds_ = nowSeconds;
        lastSnapshotSeconds_ = nowSeconds;
        lastAcceptedSnapshot_ = initialState;
        hasAcceptedSnapshot_ = true;

        EnforceDeterministicModelPolicy();
        ApplyIdentity(state_);
    }

    const TrafficState& State() const
    {
        return state_;
    }

    void ApplyState(const TrafficState& nextState)
    {
        TrafficState acceptedState = nextState;

        const bool olderServerSnapshot =
            hasAcceptedSnapshot_ &&
            !Trim(nextState.lastSeenAt).empty() &&
            !Trim(lastAcceptedSnapshot_.lastSeenAt).empty() &&
            Trim(nextState.lastSeenAt) <
                Trim(lastAcceptedSnapshot_.lastSeenAt);

        if (olderServerSnapshot)
        {
            /*
             * Never move an aircraft back to a server snapshot older than the
             * last one already accepted. Identity, labels and lights can still
             * update, but the motion fields stay on the newest state.
             */
            CopyMotionState(
                state_,
                acceptedState);
        }

        const bool modelChanged =
            state_.icaoType != acceptedState.icaoType ||
            state_.airline != acceptedState.airline ||
            state_.livery != acceptedState.livery ||
            state_.registration != acceptedState.registration;

        const bool identityChanged =
            modelChanged ||
            state_.airlineName != acceptedState.airlineName ||
            state_.registration != acceptedState.registration ||
            state_.flightNumber != acceptedState.flightNumber ||
            state_.callsign != acceptedState.callsign ||
            state_.uniqueId != acceptedState.uniqueId ||
            state_.labelConfigured != acceptedState.labelConfigured ||
            state_.labelText != acceptedState.labelText ||
            state_.labelRed != acceptedState.labelRed ||
            state_.labelGreen != acceptedState.labelGreen ||
            state_.labelBlue != acceptedState.labelBlue;

        const double nowSeconds = XPLMGetElapsedTime();
        const bool motionChanged =
            HasMotionChanged(
                interpolationTo_,
                acceptedState);

        if (motionChanged)
        {
            const TrafficState currentRenderedState =
                BuildInterpolatedState(nowSeconds);

            double observedIntervalSeconds =
                nowSeconds - lastSnapshotSeconds_;

            if (!std::isfinite(observedIntervalSeconds) ||
                observedIntervalSeconds <
                    MIN_INTERPOLATION_SECONDS)
            {
                observedIntervalSeconds =
                    DEFAULT_INTERPOLATION_SECONDS;
            }

            bool teleportDetected = false;
            bool directionReversed = false;

            if (!olderServerSnapshot &&
                acceptedState.worldMode &&
                currentRenderedState.worldMode &&
                IsValidWorldPosition(acceptedState) &&
                IsValidWorldPosition(currentRenderedState))
            {
                UpdateMotionEstimate(
                    acceptedState,
                    observedIntervalSeconds,
                    directionReversed,
                    teleportDetected);

                if (!teleportDetected &&
                    hasVelocityEstimate_ &&
                    !directionReversed)
                {
                    double correctionNorthMetres = 0.0;
                    double correctionEastMetres = 0.0;

                    CalculateWorldDeltaMetres(
                        currentRenderedState,
                        acceptedState,
                        correctionNorthMetres,
                        correctionEastMetres);

                    const double horizontalSpeed =
                        std::hypot(
                            velocityNorthMetresPerSecond_,
                            velocityEastMetresPerSecond_);

                    if (horizontalSpeed > 0.5)
                    {
                        const double alongTrackCorrectionMetres =
                            (
                                correctionNorthMetres *
                                    velocityNorthMetresPerSecond_ +
                                correctionEastMetres *
                                    velocityEastMetresPerSecond_
                            ) /
                            horizontalSpeed;

                        const double backwardToleranceMetres =
                            std::max(
                                acceptedState.onGround
                                    ? 2.5
                                    : 12.0,
                                horizontalSpeed * 0.30);

                        if (alongTrackCorrectionMetres <
                            -backwardToleranceMetres)
                        {
                            /*
                             * A delayed packet can arrive behind the position
                             * already being rendered. Do not reverse the model;
                             * retain the current position and continue using the
                             * newest velocity estimate until the feed catches up.
                             */
                            acceptedState.latitude =
                                currentRenderedState.latitude;
                            acceptedState.longitude =
                                currentRenderedState.longitude;
                            acceptedState.altitudeFeet =
                                currentRenderedState.altitudeFeet;
                        }
                    }
                }
            }

            if (teleportDetected)
            {
                interpolationFrom_ = acceptedState;
                interpolationTo_ = acceptedState;
                interpolationStartSeconds_ = nowSeconds;
                interpolationDurationSeconds_ = 0.0;
            }
            else
            {
                interpolationFrom_ = currentRenderedState;
                interpolationTo_ = acceptedState;
                interpolationStartSeconds_ = nowSeconds;
                interpolationDurationSeconds_ = std::clamp(
                    observedIntervalSeconds *
                        INTERPOLATION_BUFFER_FACTOR,
                    MIN_INTERPOLATION_SECONDS,
                    MAX_INTERPOLATION_SECONDS);
            }

            lastSnapshotSeconds_ = nowSeconds;

            if (!olderServerSnapshot)
            {
                lastAcceptedSnapshot_ = nextState;
                hasAcceptedSnapshot_ = true;
            }
        }
        else
        {
            // Keep non-positional state, such as lights, current immediately.
            interpolationTo_ = acceptedState;

            if (!olderServerSnapshot &&
                (!hasAcceptedSnapshot_ ||
                 Trim(nextState.lastSeenAt) >=
                    Trim(lastAcceptedSnapshot_.lastSeenAt)))
            {
                lastAcceptedSnapshot_ = nextState;
                hasAcceptedSnapshot_ = true;
            }
        }

        state_ = acceptedState;

        if (modelChanged)
        {
            ChangeModel(
                state_.icaoType,
                state_.airline,
                state_.livery);

            EnforceDeterministicModelPolicy();
        }

        if (identityChanged)
            ApplyIdentity(state_);
    }

    void ApplyIdentity(const TrafficState& state)
    {
        label =
            state.labelConfigured
                ? state.labelText
                : state.callsign;

        colLabel[0] =
            std::clamp(
                state.labelRed,
                0.0f,
                1.0f);
        colLabel[1] =
            std::clamp(
                state.labelGreen,
                0.0f,
                1.0f);
        colLabel[2] =
            std::clamp(
                state.labelBlue,
                0.0f,
                1.0f);

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
            acInfoTexts.airline,
            state.airlineName.c_str(),
            sizeof(acInfoTexts.airline));

        const std::string flightNumber =
            state.flightNumber.empty()
                ? state.callsign
                : state.flightNumber;

        SafeCopy(
            acInfoTexts.flightNum,
            flightNumber.c_str(),
            sizeof(acInfoTexts.flightNum));

        const std::string tailNumber =
            state.registration.empty()
                ? (state.uniqueId.empty()
                    ? "AP-REMOTE"
                    : state.uniqueId)
                : state.registration;

        SafeCopy(
            acInfoTexts.tailNum,
            tailNumber.c_str(),
            sizeof(acInfoTexts.tailNum));
    }

    void EnforceDeterministicModelPolicy()
    {
        constexpr int MAXIMUM_RELATED_FAMILY_QUALITY = 15;

        const auto sameCode = [](
            const std::string& first,
            const std::string& second)
        {
            return ToUpper(Trim(first)) ==
                   ToUpper(Trim(second));
        };

        const auto modelSupportsLivery = [&sameCode](
            const CSLModelInfo_t& modelInfo,
            const std::string& requestedLivery)
        {
            if (Trim(requestedLivery).empty())
                return false;

            for (const CSLModelInfo_t::MatchCrit_t& criterion :
                 modelInfo.vecMatchCrit)
            {
                if (sameCode(
                        criterion.livery,
                        requestedLivery))
                {
                    return true;
                }
            }

            return false;
        };

        const auto modelSupportsAirline = [&sameCode](
            const CSLModelInfo_t& modelInfo,
            const std::string& requestedAirline)
        {
            if (Trim(requestedAirline).empty())
                return false;

            for (const CSLModelInfo_t::MatchCrit_t& criterion :
                 modelInfo.vecMatchCrit)
            {
                if (sameCode(
                        criterion.icaoAirline,
                        requestedAirline))
                {
                    return true;
                }
            }

            return false;
        };

        const auto modelHasAnyAirline = [](
            const CSLModelInfo_t& modelInfo)
        {
            for (const CSLModelInfo_t::MatchCrit_t& criterion :
                 modelInfo.vecMatchCrit)
            {
                if (!Trim(criterion.icaoAirline).empty())
                    return true;
            }

            return false;
        };

        const auto modelAirlineForLog = [&sameCode](
            const CSLModelInfo_t& modelInfo,
            const std::string& requestedAirline)
        {
            for (const CSLModelInfo_t::MatchCrit_t& criterion :
                 modelInfo.vecMatchCrit)
            {
                if (!Trim(requestedAirline).empty() &&
                    sameCode(
                        criterion.icaoAirline,
                        requestedAirline))
                {
                    return ToUpper(
                        Trim(criterion.icaoAirline));
                }
            }

            for (const CSLModelInfo_t::MatchCrit_t& criterion :
                 modelInfo.vecMatchCrit)
            {
                const std::string airline =
                    ToUpper(Trim(criterion.icaoAirline));

                if (!airline.empty())
                    return airline;
            }

            return std::string{};
        };

        const bool airlineRequested =
            !Trim(state_.airline).empty();

        /*
         * AeroPath standalone multiplayer liveries use the remote registration
         * as XPMP2's special-livery key. Try an exact registration match first.
         * If no such AeroPath model exists, restore the normal airline/livery
         * request and continue through the existing deterministic CSL policy.
         */
        const std::string registrationMatch =
            ToUpper(Trim(state_.registration));

        if (!registrationMatch.empty())
        {
            const int registrationQuality =
                ChangeModel(
                    state_.icaoType,
                    "",
                    registrationMatch);

            const CSLModelInfo_t registrationModel =
                GetModelInfo();

            if (sameCode(
                    registrationModel.icaoType,
                    state_.icaoType) &&
                modelSupportsLivery(
                    registrationModel,
                    registrationMatch))
            {
                SetRender(true);

                LogMessage(
                    "AeroPath Traffic: Exact registration livery accepted for '%s': %s/%s -> '%s' (quality %d)",
                    state_.callsign.c_str(),
                    state_.icaoType.c_str(),
                    registrationMatch.c_str(),
                    registrationModel.modelName.c_str(),
                    registrationQuality);

                return;
            }

            ChangeModel(
                state_.icaoType,
                state_.airline,
                state_.livery);
        }

        CSLModelInfo_t modelInfo = GetModelInfo();
        int selectedMatchQuality = GetMatchQuality();

        const auto acceptCurrentModel =
            [&sameCode,
             &modelSupportsAirline,
             &modelHasAnyAirline,
             &airlineRequested,
             MAXIMUM_RELATED_FAMILY_QUALITY,
             this](
                const CSLModelInfo_t& candidate,
                const int quality)
        {
            const bool exactAircraft =
                sameCode(
                    candidate.icaoType,
                    state_.icaoType);

            const bool acceptableShape =
                exactAircraft ||
                (quality >= 0 &&
                 quality <= MAXIMUM_RELATED_FAMILY_QUALITY);

            if (!acceptableShape)
                return false;

            if (airlineRequested)
            {
                /*
                 * With an airline requested, only accept that airline or a
                 * genuinely generic model. Never accept another operator just
                 * because the aircraft type happens to be exact.
                 */
                return
                    modelSupportsAirline(
                        candidate,
                        state_.airline) ||
                    !modelHasAnyAirline(candidate);
            }

            /*
             * Without an airline identity, a generic CSL model is safer than a
             * random airline paint. Suppress operator-specific models until
             * AeroPath can resolve the operator.
             */
            return !modelHasAnyAirline(candidate);
        };

        std::string selectedModelAirline =
            modelAirlineForLog(
                modelInfo,
                state_.airline);

        if (acceptCurrentModel(
                modelInfo,
                selectedMatchQuality))
        {
            SetRender(true);

            LogMessage(
                "AeroPath Traffic: Strict match accepted for '%s': requested %s/%s/%s -> %s/%s using '%s' (quality %d)",
                state_.callsign.c_str(),
                state_.icaoType.c_str(),
                state_.airline.c_str(),
                state_.livery.c_str(),
                modelInfo.icaoType.c_str(),
                selectedModelAirline.c_str(),
                modelInfo.modelName.c_str(),
                selectedMatchQuality);

            return;
        }

        /*
         * An exact aircraft type is always safer than suppressing the traffic
         * completely. If the requested operator is unavailable, retain the
         * exact type even when XPMP2 selected another airline paint.
         */
        if (sameCode(
                modelInfo.icaoType,
                state_.icaoType))
        {
            SetRender(true);

            LogMessage(
                "AeroPath Traffic: Exact-type fallback accepted for '%s': requested %s/%s/%s -> %s/%s using '%s' (quality %d)",
                state_.callsign.c_str(),
                state_.icaoType.c_str(),
                state_.airline.c_str(),
                state_.livery.c_str(),
                modelInfo.icaoType.c_str(),
                selectedModelAirline.empty()
                    ? "generic"
                    : selectedModelAirline.c_str(),
                modelInfo.modelName.c_str(),
                selectedMatchQuality);

            return;
        }

        /*
         * Remove any descriptive livery text and retry the exact airline. CSL
         * packages primarily identify operators using the three-letter ICAO
         * airline code.
         */
        if (airlineRequested)
        {
            selectedMatchQuality = ChangeModel(
                state_.icaoType,
                state_.airline,
                "");

            modelInfo = GetModelInfo();
            selectedModelAirline =
                modelAirlineForLog(
                    modelInfo,
                    state_.airline);

            if (acceptCurrentModel(
                    modelInfo,
                    selectedMatchQuality))
            {
                SetRender(true);

                LogMessage(
                    "AeroPath Traffic: Airline retry accepted for '%s': requested %s/%s -> %s/%s using '%s' (quality %d)",
                    state_.callsign.c_str(),
                    state_.icaoType.c_str(),
                    state_.airline.c_str(),
                    modelInfo.icaoType.c_str(),
                    selectedModelAirline.c_str(),
                    modelInfo.modelName.c_str(),
                    selectedMatchQuality);

                return;
            }

            if (sameCode(
                    modelInfo.icaoType,
                    state_.icaoType))
            {
                SetRender(true);

                LogMessage(
                    "AeroPath Traffic: Exact-type airline fallback accepted for '%s': requested %s/%s -> %s/%s using '%s' (quality %d)",
                    state_.callsign.c_str(),
                    state_.icaoType.c_str(),
                    state_.airline.c_str(),
                    modelInfo.icaoType.c_str(),
                    selectedModelAirline.empty()
                        ? "generic"
                        : selectedModelAirline.c_str(),
                    modelInfo.modelName.c_str(),
                    selectedMatchQuality);

                return;
            }
        }

        /*
         * Final fallback is type/family without an operator criterion, but only
         * a generic model may be rendered. This prevents Wave Air, Pacific Air
         * or any other unrelated airline paint from being selected.
         */
        selectedMatchQuality = ChangeModel(
            state_.icaoType,
            "",
            "");

        modelInfo = GetModelInfo();
        selectedModelAirline =
            modelAirlineForLog(
                modelInfo,
                state_.airline);

        const bool fallbackExactAircraft =
            sameCode(
                modelInfo.icaoType,
                state_.icaoType);

        const bool fallbackAcceptableShape =
            fallbackExactAircraft ||
            (selectedMatchQuality >= 0 &&
             selectedMatchQuality <=
                 MAXIMUM_RELATED_FAMILY_QUALITY);

        const bool fallbackGeneric =
            !modelHasAnyAirline(modelInfo);

        if (fallbackExactAircraft ||
            (fallbackAcceptableShape &&
             fallbackGeneric))
        {
            SetRender(true);

            LogMessage(
                fallbackExactAircraft
                    ? "AeroPath Traffic: Exact-type final fallback accepted for '%s': requested %s/%s -> %s/%s using '%s' (quality %d)"
                    : "AeroPath Traffic: Generic family fallback accepted for '%s': requested %s/%s -> %s/%s using '%s' (quality %d)",
                state_.callsign.c_str(),
                state_.icaoType.c_str(),
                state_.airline.c_str(),
                modelInfo.icaoType.c_str(),
                selectedModelAirline.empty()
                    ? "generic"
                    : selectedModelAirline.c_str(),
                modelInfo.modelName.c_str(),
                selectedMatchQuality);

            return;
        }

        SetRender(false);

        LogMessage(
            "AeroPath Traffic: No safe model for '%s' (%s/%s). Rejected '%s' type %s airline %s quality %d; 3D rendering suppressed",
            state_.callsign.c_str(),
            state_.icaoType.c_str(),
            state_.airline.c_str(),
            modelInfo.modelName.c_str(),
            modelInfo.icaoType.c_str(),
            selectedModelAirline.c_str(),
            selectedMatchQuality);
    }

    void UpdatePosition(float, int) override
    {
        ++gXpmpPositionUpdateCount;

        const TrafficState state =
            BuildInterpolatedState(
                XPLMGetElapsedTime());

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
    static void CopyMotionState(
        const TrafficState& source,
        TrafficState& destination)
    {
        destination.worldMode = source.worldMode;
        destination.onGround = source.onGround;
        destination.forwardMetres = source.forwardMetres;
        destination.rightMetres = source.rightMetres;
        destination.upMetres = source.upMetres;
        destination.headingOffsetDegrees =
            source.headingOffsetDegrees;
        destination.latitude = source.latitude;
        destination.longitude = source.longitude;
        destination.altitudeFeet = source.altitudeFeet;
        destination.headingDegrees = source.headingDegrees;
        destination.groundSpeedKnots =
            source.groundSpeedKnots;
        destination.verticalSpeedFpm =
            source.verticalSpeedFpm;
        destination.lastSeenAt = source.lastSeenAt;
        destination.pitchDegrees = source.pitchDegrees;
        destination.rollDegrees = source.rollDegrees;
        destination.gearRatio = source.gearRatio;
        destination.flapRatio = source.flapRatio;
        destination.thrustRatio = source.thrustRatio;
    }

    static bool HasMotionChanged(
        const TrafficState& previousState,
        const TrafficState& nextState)
    {
        constexpr double POSITION_EPSILON = 0.0000001;
        constexpr double ALTITUDE_EPSILON_FEET = 0.05;
        constexpr double SPEED_EPSILON_KNOTS = 0.05;
        constexpr double VERTICAL_SPEED_EPSILON_FPM = 1.0;
        constexpr float ANGLE_EPSILON_DEGREES = 0.05f;
        constexpr float RATIO_EPSILON = 0.001f;

        return
            previousState.worldMode != nextState.worldMode ||
            std::abs(previousState.latitude - nextState.latitude) >
                POSITION_EPSILON ||
            std::abs(previousState.longitude - nextState.longitude) >
                POSITION_EPSILON ||
            std::abs(previousState.altitudeFeet - nextState.altitudeFeet) >
                ALTITUDE_EPSILON_FEET ||
            std::abs(
                previousState.groundSpeedKnots -
                nextState.groundSpeedKnots) >
                SPEED_EPSILON_KNOTS ||
            std::abs(
                previousState.verticalSpeedFpm -
                nextState.verticalSpeedFpm) >
                VERTICAL_SPEED_EPSILON_FPM ||
            std::abs(previousState.headingDegrees - nextState.headingDegrees) >
                ANGLE_EPSILON_DEGREES ||
            std::abs(previousState.pitchDegrees - nextState.pitchDegrees) >
                ANGLE_EPSILON_DEGREES ||
            std::abs(previousState.rollDegrees - nextState.rollDegrees) >
                ANGLE_EPSILON_DEGREES ||
            std::abs(previousState.gearRatio - nextState.gearRatio) >
                RATIO_EPSILON ||
            std::abs(previousState.flapRatio - nextState.flapRatio) >
                RATIO_EPSILON ||
            std::abs(previousState.thrustRatio - nextState.thrustRatio) >
                RATIO_EPSILON;
    }

    static void CalculateWorldDeltaMetres(
        const TrafficState& fromState,
        const TrafficState& toState,
        double& northMetres,
        double& eastMetres)
    {
        const double fromLatitudeRadians =
            DegreesToRadians(fromState.latitude);

        const double toLatitudeRadians =
            DegreesToRadians(toState.latitude);

        const double latitudeDifferenceRadians =
            toLatitudeRadians -
            fromLatitudeRadians;

        const double longitudeDifferenceRadians =
            DegreesToRadians(
                toState.longitude -
                fromState.longitude);

        const double averageLatitudeRadians =
            (fromLatitudeRadians +
             toLatitudeRadians) * 0.5;

        northMetres =
            latitudeDifferenceRadians *
            EARTH_RADIUS_METRES;

        eastMetres =
            longitudeDifferenceRadians *
            EARTH_RADIUS_METRES *
            std::cos(averageLatitudeRadians);
    }

    static double TeleportThresholdMetres(
        const TrafficState& state)
    {
        return state.onGround
            ? 1500.0
            : 10000.0;
    }

    void ResetMotionEstimate()
    {
        velocityNorthMetresPerSecond_ = 0.0;
        velocityEastMetresPerSecond_ = 0.0;
        velocityVerticalFeetPerSecond_ = 0.0;
        hasVelocityEstimate_ = false;
    }

    void UpdateMotionEstimate(
        const TrafficState& nextState,
        const double observedIntervalSeconds,
        bool& directionReversed,
        bool& teleportDetected)
    {
        directionReversed = false;
        teleportDetected = false;

        if (!hasAcceptedSnapshot_ ||
            !lastAcceptedSnapshot_.worldMode ||
            !nextState.worldMode ||
            !IsValidWorldPosition(lastAcceptedSnapshot_) ||
            !IsValidWorldPosition(nextState))
        {
            ResetMotionEstimate();
            return;
        }

        double northMetres = 0.0;
        double eastMetres = 0.0;

        CalculateWorldDeltaMetres(
            lastAcceptedSnapshot_,
            nextState,
            northMetres,
            eastMetres);

        const double horizontalDistanceMetres =
            std::hypot(
                northMetres,
                eastMetres);

        if (horizontalDistanceMetres >
            TeleportThresholdMetres(nextState))
        {
            teleportDetected = true;
            ResetMotionEstimate();
            return;
        }

        const double safeIntervalSeconds =
            std::clamp(
                observedIntervalSeconds,
                MIN_INTERPOLATION_SECONDS,
                5.0);

        const double observedNorthMetresPerSecond =
            northMetres /
            safeIntervalSeconds;

        const double observedEastMetresPerSecond =
            eastMetres /
            safeIntervalSeconds;

        const double observedVerticalFeetPerSecond =
            (
                nextState.altitudeFeet -
                lastAcceptedSnapshot_.altitudeFeet
            ) /
            safeIntervalSeconds;

        const double observedHorizontalSpeed =
            std::hypot(
                observedNorthMetresPerSecond,
                observedEastMetresPerSecond);

        const double maximumReasonableSpeed =
            nextState.onGround
                ? MAX_GROUND_TRACK_SPEED_METRES_PER_SECOND
                : MAX_AIRBORNE_TRACK_SPEED_METRES_PER_SECOND;

        if (!std::isfinite(observedHorizontalSpeed) ||
            observedHorizontalSpeed >
                maximumReasonableSpeed)
        {
            return;
        }

        const double reportedSpeedMetresPerSecond =
            std::clamp(
                nextState.groundSpeedKnots *
                    KNOTS_TO_METRES_PER_SECOND,
                0.0,
                maximumReasonableSpeed);

        if (reportedSpeedMetresPerSecond < 0.5 &&
            horizontalDistanceMetres < 2.0)
        {
            velocityNorthMetresPerSecond_ = 0.0;
            velocityEastMetresPerSecond_ = 0.0;
            velocityVerticalFeetPerSecond_ =
                nextState.onGround
                    ? 0.0
                    : nextState.verticalSpeedFpm / 60.0;
            hasVelocityEstimate_ = false;
            return;
        }

        double candidateNorthMetresPerSecond =
            observedNorthMetresPerSecond;

        double candidateEastMetresPerSecond =
            observedEastMetresPerSecond;

        if (observedHorizontalSpeed > 0.25 &&
            reportedSpeedMetresPerSecond > 0.25)
        {
            const double blendedSpeed =
                observedHorizontalSpeed * 0.65 +
                reportedSpeedMetresPerSecond * 0.35;

            candidateNorthMetresPerSecond =
                observedNorthMetresPerSecond /
                observedHorizontalSpeed *
                blendedSpeed;

            candidateEastMetresPerSecond =
                observedEastMetresPerSecond /
                observedHorizontalSpeed *
                blendedSpeed;
        }
        else if (observedHorizontalSpeed <= 0.25 &&
                 reportedSpeedMetresPerSecond > 0.25 &&
                 !nextState.onGround)
        {
            const double headingRadians =
                DegreesToRadians(
                    nextState.headingDegrees);

            candidateNorthMetresPerSecond =
                std::cos(headingRadians) *
                reportedSpeedMetresPerSecond;

            candidateEastMetresPerSecond =
                std::sin(headingRadians) *
                reportedSpeedMetresPerSecond;
        }

        const double candidateHorizontalSpeed =
            std::hypot(
                candidateNorthMetresPerSecond,
                candidateEastMetresPerSecond);

        if (hasVelocityEstimate_)
        {
            const double existingHorizontalSpeed =
                std::hypot(
                    velocityNorthMetresPerSecond_,
                    velocityEastMetresPerSecond_);

            const double directionDotProduct =
                velocityNorthMetresPerSecond_ *
                    candidateNorthMetresPerSecond +
                velocityEastMetresPerSecond_ *
                    candidateEastMetresPerSecond;

            directionReversed =
                existingHorizontalSpeed > 0.75 &&
                candidateHorizontalSpeed > 0.75 &&
                directionDotProduct <
                    -0.20 *
                    existingHorizontalSpeed *
                    candidateHorizontalSpeed;

            if (directionReversed)
            {
                velocityNorthMetresPerSecond_ =
                    candidateNorthMetresPerSecond;
                velocityEastMetresPerSecond_ =
                    candidateEastMetresPerSecond;
            }
            else
            {
                velocityNorthMetresPerSecond_ =
                    velocityNorthMetresPerSecond_ *
                        (1.0 - VELOCITY_BLEND_FACTOR) +
                    candidateNorthMetresPerSecond *
                        VELOCITY_BLEND_FACTOR;

                velocityEastMetresPerSecond_ =
                    velocityEastMetresPerSecond_ *
                        (1.0 - VELOCITY_BLEND_FACTOR) +
                    candidateEastMetresPerSecond *
                        VELOCITY_BLEND_FACTOR;
            }

            velocityVerticalFeetPerSecond_ =
                velocityVerticalFeetPerSecond_ *
                    (1.0 - VELOCITY_BLEND_FACTOR) +
                observedVerticalFeetPerSecond *
                    VELOCITY_BLEND_FACTOR;
        }
        else if (candidateHorizontalSpeed > 0.25)
        {
            velocityNorthMetresPerSecond_ =
                candidateNorthMetresPerSecond;
            velocityEastMetresPerSecond_ =
                candidateEastMetresPerSecond;
            velocityVerticalFeetPerSecond_ =
                observedVerticalFeetPerSecond;
            hasVelocityEstimate_ = true;
        }

        if (!nextState.onGround &&
            std::abs(nextState.verticalSpeedFpm) > 10.0)
        {
            velocityVerticalFeetPerSecond_ =
                velocityVerticalFeetPerSecond_ * 0.65 +
                (nextState.verticalSpeedFpm / 60.0) * 0.35;
        }
        else if (nextState.onGround)
        {
            velocityVerticalFeetPerSecond_ = 0.0;
        }
    }

    static void AdvanceWorldState(
        TrafficState& state,
        const double northMetresPerSecond,
        const double eastMetresPerSecond,
        const double verticalFeetPerSecond,
        const double seconds)
    {
        if (seconds <= 0.0 ||
            !state.worldMode ||
            !IsValidWorldPosition(state))
        {
            return;
        }

        const double northMetres =
            northMetresPerSecond *
            seconds;

        const double eastMetres =
            eastMetresPerSecond *
            seconds;

        const double latitudeRadians =
            DegreesToRadians(state.latitude);

        state.latitude +=
            northMetres /
            EARTH_RADIUS_METRES *
            180.0 /
            PI;

        const double longitudeScale =
            EARTH_RADIUS_METRES *
            std::max(
                0.01,
                std::abs(
                    std::cos(latitudeRadians)));

        state.longitude +=
            eastMetres /
            longitudeScale *
            180.0 /
            PI;

        if (!state.onGround)
        {
            state.altitudeFeet +=
                verticalFeetPerSecond *
                seconds;
        }
    }

    static double InterpolateDouble(
        const double fromValue,
        const double toValue,
        const double progress)
    {
        return fromValue +
            (toValue - fromValue) * progress;
    }

    static float InterpolateFloat(
        const float fromValue,
        const float toValue,
        const double progress)
    {
        return static_cast<float>(
            fromValue +
            (toValue - fromValue) * progress);
    }

    static float InterpolateHeading(
        const float fromHeading,
        const float toHeading,
        const double progress)
    {
        float difference =
            NormaliseHeading(toHeading) -
            NormaliseHeading(fromHeading);

        if (difference > 180.0f)
            difference -= 360.0f;
        else if (difference < -180.0f)
            difference += 360.0f;

        return NormaliseHeading(
            fromHeading +
            static_cast<float>(difference * progress));
    }

    TrafficState BuildInterpolatedState(
        const double nowSeconds) const
    {
        TrafficState renderedState = state_;

        if (interpolationDurationSeconds_ <= 0.0 ||
            interpolationFrom_.worldMode !=
                interpolationTo_.worldMode)
        {
            renderedState = interpolationTo_;

            if (hasVelocityEstimate_ &&
                interpolationTo_.worldMode)
            {
                const double extrapolationSeconds =
                    std::clamp(
                        nowSeconds -
                            interpolationStartSeconds_,
                        0.0,
                        MAX_EXTRAPOLATION_SECONDS);

                AdvanceWorldState(
                    renderedState,
                    velocityNorthMetresPerSecond_,
                    velocityEastMetresPerSecond_,
                    velocityVerticalFeetPerSecond_,
                    extrapolationSeconds);
            }

            return renderedState;
        }

        const double rawProgress =
            (nowSeconds - interpolationStartSeconds_) /
            interpolationDurationSeconds_;

        const double progress =
            std::clamp(
                rawProgress,
                0.0,
                1.0);

        renderedState.latitude = InterpolateDouble(
            interpolationFrom_.latitude,
            interpolationTo_.latitude,
            progress);

        renderedState.longitude = InterpolateDouble(
            interpolationFrom_.longitude,
            interpolationTo_.longitude,
            progress);

        renderedState.altitudeFeet = InterpolateDouble(
            interpolationFrom_.altitudeFeet,
            interpolationTo_.altitudeFeet,
            progress);

        renderedState.groundSpeedKnots = InterpolateDouble(
            interpolationFrom_.groundSpeedKnots,
            interpolationTo_.groundSpeedKnots,
            progress);

        renderedState.verticalSpeedFpm = InterpolateDouble(
            interpolationFrom_.verticalSpeedFpm,
            interpolationTo_.verticalSpeedFpm,
            progress);

        renderedState.headingDegrees = InterpolateHeading(
            interpolationFrom_.headingDegrees,
            interpolationTo_.headingDegrees,
            progress);

        renderedState.pitchDegrees = InterpolateFloat(
            interpolationFrom_.pitchDegrees,
            interpolationTo_.pitchDegrees,
            progress);

        renderedState.rollDegrees = InterpolateFloat(
            interpolationFrom_.rollDegrees,
            interpolationTo_.rollDegrees,
            progress);

        renderedState.gearRatio = InterpolateFloat(
            interpolationFrom_.gearRatio,
            interpolationTo_.gearRatio,
            progress);

        renderedState.flapRatio = InterpolateFloat(
            interpolationFrom_.flapRatio,
            interpolationTo_.flapRatio,
            progress);

        renderedState.thrustRatio = InterpolateFloat(
            interpolationFrom_.thrustRatio,
            interpolationTo_.thrustRatio,
            progress);

        if (rawProgress > 1.0 &&
            hasVelocityEstimate_ &&
            interpolationTo_.worldMode)
        {
            const double extrapolationSeconds =
                std::clamp(
                    nowSeconds -
                        (
                            interpolationStartSeconds_ +
                            interpolationDurationSeconds_
                        ),
                    0.0,
                    MAX_EXTRAPOLATION_SECONDS);

            AdvanceWorldState(
                renderedState,
                velocityNorthMetresPerSecond_,
                velocityEastMetresPerSecond_,
                velocityVerticalFeetPerSecond_,
                extrapolationSeconds);
        }

        return renderedState;
    }

    TrafficState state_;
    TrafficState interpolationFrom_;
    TrafficState interpolationTo_;
    TrafficState lastAcceptedSnapshot_;

    bool hasAcceptedSnapshot_ = false;
    bool hasVelocityEstimate_ = false;

    double velocityNorthMetresPerSecond_ = 0.0;
    double velocityEastMetresPerSecond_ = 0.0;
    double velocityVerticalFeetPerSecond_ = 0.0;

    double interpolationStartSeconds_ = 0.0;
    double interpolationDurationSeconds_ = 0.0;
    double lastSnapshotSeconds_ = 0.0;
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
            MENU_TRAFFIC_LABELS_OFF,
            !gTrafficLabelsEnabled
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_TRAFFIC_LABELS_20_NM,
            gTrafficLabelsEnabled &&
            std::abs(
                gTrafficLabelDistanceNm -
                20.0f) < 0.1f
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_TRAFFIC_LABELS_50_NM,
            gTrafficLabelsEnabled &&
            std::abs(
                gTrafficLabelDistanceNm -
                50.0f) < 0.1f
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_TRAFFIC_LABELS_100_NM,
            gTrafficLabelsEnabled &&
            std::abs(
                gTrafficLabelDistanceNm -
                100.0f) < 0.1f
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_CSL_USE_BUNDLED,
            gConfiguredExternalCslPath.empty()
                ? xplm_Menu_Checked
                : xplm_Menu_Unchecked);

        XPLMCheckMenuItem(
            gMenu,
            MENU_TAXI_ROUTE_LIGHTS,
            AeroPathTaxiGuidance::IsManualVisible()
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

        else if (key == "airline_name")
            state.airlineName = value;

        else if (key == "livery")
            state.livery = ToUpper(value);

        else if (key == "registration")
            state.registration = ToUpper(value);

        else if (key == "flight_number")
            state.flightNumber = ToUpper(value);

        else if (key == "callsign")
            state.callsign = ToUpper(value);

        else if (key == "label" ||
                 key == "label_text")
        {
            state.labelConfigured = true;
            state.labelText = value;
        }

        else if (key == "label_red")
            state.labelRed =
                std::clamp(
                    ParseFloat(value, state.labelRed),
                    0.0f,
                    1.0f);

        else if (key == "label_green")
            state.labelGreen =
                std::clamp(
                    ParseFloat(value, state.labelGreen),
                    0.0f,
                    1.0f);

        else if (key == "label_blue")
            state.labelBlue =
                std::clamp(
                    ParseFloat(value, state.labelBlue),
                    0.0f,
                    1.0f);

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

        else if (key == "ground_speed_kt" ||
                 key == "ground_speed_knots")
            state.groundSpeedKnots =
                std::clamp(
                    ParseDouble(
                        value,
                        state.groundSpeedKnots),
                    0.0,
                    1500.0);

        else if (key == "vertical_speed_fpm")
            state.verticalSpeedFpm =
                std::clamp(
                    ParseDouble(
                        value,
                        state.verticalSpeedFpm),
                    -15000.0,
                    15000.0);

        else if (key == "last_seen_at")
            state.lastSeenAt = Trim(value);

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
        state.airlineName = Trim(state.airlineName);
        state.livery = ToUpper(Trim(state.livery));
        state.registration = ToUpper(Trim(state.registration));
        state.flightNumber = ToUpper(Trim(state.flightNumber));
        state.labelText = Trim(state.labelText);
        state.lastSeenAt = Trim(state.lastSeenAt);
        state.groundSpeedKnots =
            std::clamp(
                state.groundSpeedKnots,
                0.0,
                1500.0);
        state.verticalSpeedFpm =
            std::clamp(
                state.verticalSpeedFpm,
                -15000.0,
                15000.0);
        state.labelRed =
            std::clamp(state.labelRed, 0.0f, 1.0f);
        state.labelGreen =
            std::clamp(state.labelGreen, 0.0f, 1.0f);
        state.labelBlue =
            std::clamp(state.labelBlue, 0.0f, 1.0f);

        if (state.icaoType.empty())
            state.icaoType = "C172";

        if (state.flightNumber.empty())
            state.flightNumber = state.callsign;

        if (state.callsign.empty())
            state.callsign =
                !state.flightNumber.empty()
                    ? state.flightNumber
                    : state.uniqueId.empty()
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
                key == "generated_at")
            {
                parsedFeed.generatedAt = value;
                continue;
            }

            if (!insideAircraftBlock &&
                (key == "version" ||
                 key == "count"))
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
                    previousState.livery != state.livery ||
                    previousState.registration != state.registration;

                if (modelChanged)
                {
                    LogMessage(
                        "AeroPath Traffic: Model request for '%s' changed to %s/%s/%s registration %s",
                        state.callsign.c_str(),
                        state.icaoType.c_str(),
                        state.airline.c_str(),
                        state.livery.c_str(),
                        state.registration.c_str());
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
            const double nowSeconds = XPLMGetElapsedTime();

            if (gFeedUnavailableSinceSeconds < 0.0)
                gFeedUnavailableSinceSeconds = nowSeconds;

            const double unavailableSeconds =
                nowSeconds - gFeedUnavailableSinceSeconds;

            /*
             * AeroPath Desktop replaces the feed atomically. On Windows there
             * can be a very short interval where the old file has gone and the
             * new file is not visible yet. Retain existing aircraft through
             * that transient gap instead of destroying and recreating them.
             */
            if (!gRemoteAircraft.empty() &&
                unavailableSeconds < TRAFFIC_FILE_GRACE_SECONDS)
            {
                return;
            }

            gFeedEnabled = false;
            RemoveAllAircraft();
            return;
        }

        if (gFeedUnavailableSinceSeconds >= 0.0)
        {
            const double unavailableSeconds =
                XPLMGetElapsedTime() -
                gFeedUnavailableSinceSeconds;

            if (!gRemoteAircraft.empty() &&
                unavailableSeconds < TRAFFIC_FILE_GRACE_SECONDS)
            {
                LogMessage(
                    "AeroPath Traffic: Feed recovered after %.2f s; existing aircraft retained",
                    unavailableSeconds);
            }

            gFeedUnavailableSinceSeconds = -1.0;
        }

        const double nowSeconds =
            XPLMGetElapsedTime();

        if (!parsedFeed.generatedAt.empty() &&
            parsedFeed.generatedAt !=
                gLastProcessedFeedGeneration)
        {
            if (gLastNewFeedSeconds >= 0.0)
            {
                gLastFeedIntervalSeconds =
                    nowSeconds -
                    gLastNewFeedSeconds;
            }

            gLastNewFeedSeconds = nowSeconds;
            gLastProcessedFeedGeneration =
                parsedFeed.generatedAt;
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
        AeroPathTaxiGuidance::Update();
        UpdateMenuCheckmarks();

        const double nowSeconds =
            XPLMGetElapsedTime();

        if (nowSeconds -
                gLastTimingLogSeconds >=
            TIMING_LOG_INTERVAL_SECONDS)
        {
            const double elapsedSeconds =
                gLastTimingLogSeconds > 0.0
                    ? nowSeconds -
                      gLastTimingLogSeconds
                    : TIMING_LOG_INTERVAL_SECONDS;

            const std::uint64_t positionUpdates =
                gXpmpPositionUpdateCount -
                gLastXpmpPositionUpdateCount;

            const double totalUpdatesPerSecond =
                elapsedSeconds > 0.0
                    ? static_cast<double>(
                          positionUpdates) /
                      elapsedSeconds
                    : 0.0;

            const double updatesPerAircraftPerSecond =
                !gRemoteAircraft.empty()
                    ? totalUpdatesPerSecond /
                      static_cast<double>(
                          gRemoteAircraft.size())
                    : 0.0;

            LogMessage(
                "AeroPath Traffic timing: feed interval %.0f ms; poll interval %.0f ms; XPMP2 position updates %.1f/s per aircraft; active aircraft %zu",
                gLastFeedIntervalSeconds * 1000.0,
                static_cast<double>(
                    TRAFFIC_POLL_INTERVAL_SECONDS) *
                    1000.0,
                updatesPerAircraftPerSecond,
                gRemoteAircraft.size());

            gLastTimingLogSeconds = nowSeconds;
            gLastXpmpPositionUpdateCount =
                gXpmpPositionUpdateCount;
        }

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

            case MENU_TRAFFIC_LABELS_OFF:
                gTrafficLabelsEnabled = false;
                ApplyTrafficLabelSettings();
                break;

            case MENU_TRAFFIC_LABELS_20_NM:
                gTrafficLabelsEnabled = true;
                gTrafficLabelDistanceNm = 20.0f;
                ApplyTrafficLabelSettings();
                break;

            case MENU_TRAFFIC_LABELS_50_NM:
                gTrafficLabelsEnabled = true;
                gTrafficLabelDistanceNm = 50.0f;
                ApplyTrafficLabelSettings();
                break;

            case MENU_TRAFFIC_LABELS_100_NM:
                gTrafficLabelsEnabled = true;
                gTrafficLabelDistanceNm = 100.0f;
                ApplyTrafficLabelSettings();
                break;

            case MENU_CSL_LOCATE_FOLDER:
                LocateCslFolder();
                break;

            case MENU_CSL_USE_BUNDLED:
                UseBundledCslFolder();
                break;

            case MENU_CSL_VALIDATE_FOLDER:
                ValidateConfiguredCslFolder();
                break;

            case MENU_CSL_OPEN_FOLDER:
                OpenConfiguredCslFolder();
                break;

            case MENU_RELOAD_FEED:
                ReloadTrafficFeed();
                break;

            case MENU_TAXI_ROUTE_LIGHTS:
                AeroPathTaxiGuidance::ToggleVisible();
                break;

            case MENU_TAXI_REBUILD_ROUTE:
                AeroPathTaxiGuidance::RebuildRoute();
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
        "Traffic Labels Off",
        reinterpret_cast<void*>(
            MENU_TRAFFIC_LABELS_OFF),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Traffic Labels 20 NM",
        reinterpret_cast<void*>(
            MENU_TRAFFIC_LABELS_20_NM),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Traffic Labels 50 NM",
        reinterpret_cast<void*>(
            MENU_TRAFFIC_LABELS_50_NM),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Traffic Labels 100 NM",
        reinterpret_cast<void*>(
            MENU_TRAFFIC_LABELS_100_NM),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Locate CSL Folder...",
        reinterpret_cast<void*>(
            MENU_CSL_LOCATE_FOLDER),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Use AeroPath Bundled CSL",
        reinterpret_cast<void*>(
            MENU_CSL_USE_BUNDLED),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Validate Selected CSL Folder",
        reinterpret_cast<void*>(
            MENU_CSL_VALIDATE_FOLDER),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Open Selected CSL Folder",
        reinterpret_cast<void*>(
            MENU_CSL_OPEN_FOLDER),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Reload Traffic File",
        reinterpret_cast<void*>(
            MENU_RELOAD_FEED),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Manual Taxi Route Lights",
        reinterpret_cast<void*>(MENU_TAXI_ROUTE_LIGHTS),
        0);

    XPLMAppendMenuItem(
        gMenu,
        "Rebuild Taxi Route",
        reinterpret_cast<void*>(MENU_TAXI_REBUILD_ROUTE),
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
    AeroPathTaxiGuidance::Shutdown();
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

    gResourcePath = resourcePath;
    gCslConfigPath =
        JoinPath(
            gResourcePath,
            "AeroPathTraffic.ini");
    gBundledCslPath =
        JoinPath(
            gResourcePath,
            "CSL");
    gAeroPathModelsPath =
        JoinPath(
            gResourcePath,
            "AeroPathModels");

    gTrafficFilePath = resourcePath;
    gTrafficFilePath += pathSeparator;
    gTrafficFilePath += "AeroPathTraffic.txt";

    AeroPathTaxiGuidance::Initialise(resourcePath);

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

    /*
     * Load AeroPath's own lightweight multiplayer models independently from
     * the user's external or bundled CSL library. XPMPLoadCSLPackage adds
     * models to XPMP2's catalogue, so the two libraries can coexist.
     */
    const CslValidationResult aeroPathModelValidation =
        ValidateCslFolder(
            gAeroPathModelsPath);

    if (aeroPathModelValidation.valid)
    {
        result =
            XPMPLoadCSLPackage(
                gAeroPathModelsPath.c_str());

        if (result[0])
        {
            LogMessage(
                "AeroPath Traffic: AeroPath model loading reported for '%s': %s",
                gAeroPathModelsPath.c_str(),
                result);
        }
        else
        {
            LogMessage(
                "AeroPath Traffic: AeroPath multiplayer models loaded from '%s' (%zu packages)",
                gAeroPathModelsPath.c_str(),
                aeroPathModelValidation.packageCount);
        }
    }
    else
    {
        LogMessage(
            "AeroPath Traffic: No standalone AeroPath multiplayer models found at '%s': %s",
            gAeroPathModelsPath.c_str(),
            aeroPathModelValidation.message.c_str());
    }

    gLoadedCslPath =
        ResolveCslFolderForStartup();

    if (!gLoadedCslPath.empty() &&
        std::filesystem::u8path(gLoadedCslPath).lexically_normal() !=
            std::filesystem::u8path(gAeroPathModelsPath).lexically_normal())
    {
        result =
            XPMPLoadCSLPackage(
                gLoadedCslPath.c_str());

        if (result[0])
        {
            LogMessage(
                "AeroPath Traffic: CSL loading reported for '%s': %s",
                gLoadedCslPath.c_str(),
                result);
        }
        else
        {
            LogMessage(
                "AeroPath Traffic: CSL fallback library loaded from '%s'",
                gLoadedCslPath.c_str());
        }
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

    ApplyTrafficLabelSettings();

    XPMPRegisterPlaneNotifierFunc(
        PlaneNotifier,
        nullptr);

    gFeedEnabled = true;
    gFeedUnavailableSinceSeconds = -1.0;
    gLastReportedTrafficCount = static_cast<std::size_t>(-1);
    gLastProcessedFeedGeneration.clear();
    gLastNewFeedSeconds = -1.0;
    gLastFeedIntervalSeconds = 0.0;
    gLastTimingLogSeconds = XPLMGetElapsedTime();
    gXpmpPositionUpdateCount = 0;
    gLastXpmpPositionUpdateCount = 0;

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
    AeroPathTaxiGuidance::Shutdown();
    gFeedUnavailableSinceSeconds = -1.0;

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
