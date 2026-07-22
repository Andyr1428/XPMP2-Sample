#if defined(_WIN32) && !defined(NOMINMAX)
#define NOMINMAX
#endif

#include "AeroPathTaxiGuidance.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "XPLMDataAccess.h"
#include "XPLMGraphics.h"
#include "XPLMInstance.h"
#include "XPLMNavigation.h"
#include "XPLMProcessing.h"
#include "XPLMScenery.h"
#include "XPLMUtilities.h"

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

namespace
{
    constexpr double PI = 3.141592653589793238462643383279502884;
    constexpr double EARTH_RADIUS_METRES = 6371000.0;

    constexpr std::size_t LIGHT_COUNT = 40;
    constexpr float LIGHT_FIRST_OFFSET_METRES = 10.0f;
    constexpr float LIGHT_SPACING_METRES = 12.0f;
    constexpr float LIGHT_HEIGHT_OFFSET_METRES = 0.08f;
    constexpr double CONFIG_CHECK_INTERVAL_SECONDS = 1.0;
    constexpr double ROUTE_RETRY_INTERVAL_SECONDS = 5.0;
    constexpr double MAX_START_NODE_DISTANCE_METRES = 1500.0;
    constexpr double OFF_ROUTE_REBUILD_DISTANCE_METRES = 90.0;
    constexpr double ROUTE_COMPLETION_DISTANCE_METRES = 28.0;
    constexpr long long CONFIG_STALE_AFTER_SECONDS = 35;

    struct GeoPoint
    {
        double latitude = 0.0;
        double longitude = 0.0;
    };

    struct LocalPoint
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct TaxiConfig
    {
        bool filePresent = false;
        bool enabled = true;
        std::string airport;
        std::string runway;
        std::string phase;
        long long generatedUnix = 0;
        std::string contextKey;
        std::string rawContent;
    };

    struct TaxiNode
    {
        int id = -1;
        GeoPoint position;
        std::string usage;
        std::string name;
    };

    struct TaxiEdge
    {
        int from = -1;
        int to = -1;
        bool twoWay = true;
        bool runway = false;
        std::string restriction;
        std::string name;
        std::vector<std::string> activeRunways;
    };

    struct RunwayEnd
    {
        std::string identifier;
        GeoPoint position;
    };

    struct AirportData
    {
        std::string identifier;
        std::string sourcePath;
        std::unordered_map<int, TaxiNode> nodes;
        std::vector<TaxiEdge> edges;
        std::vector<RunwayEnd> runwayEnds;
    };

    struct AdjacentNode
    {
        int nodeId = -1;
        double cost = 0.0;
    };

    bool gInitialised = false;
    bool gManualVisible = false;
    bool gAutomaticVisible = false;
    bool gRouteCompleted = false;
    bool gRouteValid = false;
    bool gUsingStraightFallback = false;

    std::string gPluginResourcesPath;
    std::string gConfigPath;
    std::string gTaxiLightObjectRelativePath;
    std::string gLastConfigContent;
    std::string gActiveAirport;
    std::string gActiveRunway;
    std::string gActiveAptPath;

    double gLastConfigCheckSeconds = -1000.0;
    double gLastRouteAttemptSeconds = -1000.0;

    XPLMObjectRef gTaxiLightObject = nullptr;
    XPLMProbeRef gTerrainProbe = nullptr;
    std::vector<XPLMInstanceRef> gLightInstances;

    XPLMDataRef gLocalX = nullptr;
    XPLMDataRef gLocalY = nullptr;
    XPLMDataRef gLocalZ = nullptr;
    XPLMDataRef gHeading = nullptr;

    std::vector<GeoPoint> gRoutePoints;

    bool ShouldRenderGuidance()
    {
        return gManualVisible || gAutomaticVisible;
    }

    bool IsConfigFresh(const TaxiConfig& config)
    {
        // Automatic mode requires the Stage 3 desktop heartbeat. A Stage 2
        // file can still be used through the manual menu option.
        if (config.generatedUnix <= 0)
            return false;

        const long long now = static_cast<long long>(std::time(nullptr));
        if (now <= 0)
            return true;

        const long long age = now - config.generatedUnix;
        return age >= -5 && age <= CONFIG_STALE_AFTER_SECONDS;
    }

    void LogMessage(const char* format, ...)
    {
        char buffer[2048] = {};
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
            value.begin(), value.end(),
            [](unsigned char character) { return std::isspace(character) != 0; });

        if (first == value.end())
            return {};

        const auto last = std::find_if_not(
            value.rbegin(), value.rend(),
            [](unsigned char character) { return std::isspace(character) != 0; }).base();

        return std::string(first, last);
    }

    std::string ToLower(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(),
            [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
        return value;
    }

    std::string ToUpper(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(),
            [](unsigned char character) { return static_cast<char>(std::toupper(character)); });
        return value;
    }

    std::vector<std::string> SplitWhitespace(const std::string& line)
    {
        std::istringstream stream(line);
        std::vector<std::string> values;
        std::string value;
        while (stream >> value)
            values.push_back(value);
        return values;
    }

    std::vector<std::string> Split(const std::string& value, char separator)
    {
        std::vector<std::string> parts;
        std::string current;
        std::istringstream stream(value);
        while (std::getline(stream, current, separator))
            parts.push_back(Trim(current));
        return parts;
    }

    bool ParseBool(const std::string& value, bool fallback)
    {
        const std::string clean = ToLower(Trim(value));
        if (clean == "1" || clean == "true" || clean == "yes" || clean == "on")
            return true;
        if (clean == "0" || clean == "false" || clean == "no" || clean == "off")
            return false;
        return fallback;
    }

    bool ParseDouble(const std::string& value, double& result)
    {
        try
        {
            std::size_t parsed = 0;
            const double number = std::stod(value, &parsed);
            if (parsed == 0 || !std::isfinite(number))
                return false;
            result = number;
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool ParseInt(const std::string& value, int& result)
    {
        try
        {
            std::size_t parsed = 0;
            const int number = std::stoi(value, &parsed);
            if (parsed == 0)
                return false;
            result = number;
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    double DegreesToRadians(double degrees)
    {
        return degrees * PI / 180.0;
    }

    double RadiansToDegrees(double radians)
    {
        return radians * 180.0 / PI;
    }

    double NormaliseHeading(double heading)
    {
        heading = std::fmod(heading, 360.0);
        if (heading < 0.0)
            heading += 360.0;
        return heading;
    }

    std::string NormalisePathForComparison(std::string value)
    {
        std::replace(value.begin(), value.end(), '\\', '/');
        return ToLower(value);
    }

    std::string JoinPath(const std::string& first, const std::string& second)
    {
        if (first.empty())
            return second;
        if (second.empty())
            return first;

        const char separator = XPLMGetDirectorySeparator()[0];
        std::string result = first;
        if (result.back() != '/' && result.back() != '\\')
            result.push_back(separator);

        std::string tail = second;
        while (!tail.empty() && (tail.front() == '/' || tail.front() == '\\'))
            tail.erase(tail.begin());

        std::replace(tail.begin(), tail.end(), '/', separator);
        std::replace(tail.begin(), tail.end(), '\\', separator);
        result += tail;
        return result;
    }

    bool FileExists(const std::string& path)
    {
        std::ifstream stream(path, std::ios::binary);
        return stream.good();
    }

    std::string ReadWholeFile(const std::string& path)
    {
        std::ifstream stream(path, std::ios::binary);
        if (!stream)
            return {};

        std::ostringstream content;
        content << stream.rdbuf();
        return content.str();
    }

    std::string MakePathRelativeToXPlaneRoot(const std::string& absolutePath)
    {
        char systemPathBuffer[2048] = {};
        XPLMGetSystemPath(systemPathBuffer);

        const std::string systemPath = systemPathBuffer;
        const std::string normalisedSystemPath = NormalisePathForComparison(systemPath);
        const std::string normalisedAbsolutePath = NormalisePathForComparison(absolutePath);

        if (!normalisedSystemPath.empty() &&
            normalisedAbsolutePath.rfind(normalisedSystemPath, 0) == 0)
        {
            std::string relativePath = absolutePath.substr(systemPath.size());
            while (!relativePath.empty() &&
                   (relativePath.front() == '\\' || relativePath.front() == '/'))
            {
                relativePath.erase(relativePath.begin());
            }
            return relativePath;
        }

        return {};
    }

    GeoPoint CurrentAircraftGeoPoint()
    {
        GeoPoint point;
        if (!gLocalX || !gLocalY || !gLocalZ)
            return point;

        double elevationMetres = 0.0;
        XPLMLocalToWorld(
            XPLMGetDatad(gLocalX),
            XPLMGetDatad(gLocalY),
            XPLMGetDatad(gLocalZ),
            &point.latitude,
            &point.longitude,
            &elevationMetres);

        return point;
    }

    LocalPoint ToLocal(const GeoPoint& point)
    {
        LocalPoint local;
        XPLMWorldToLocal(
            point.latitude,
            point.longitude,
            0.0,
            &local.x,
            &local.y,
            &local.z);
        return local;
    }

    double DistanceMetres(const GeoPoint& first, const GeoPoint& second)
    {
        const double lat1 = DegreesToRadians(first.latitude);
        const double lat2 = DegreesToRadians(second.latitude);
        const double deltaLat = lat2 - lat1;
        const double deltaLon = DegreesToRadians(second.longitude - first.longitude);

        const double a =
            std::sin(deltaLat / 2.0) * std::sin(deltaLat / 2.0) +
            std::cos(lat1) * std::cos(lat2) *
            std::sin(deltaLon / 2.0) * std::sin(deltaLon / 2.0);

        return EARTH_RADIUS_METRES * 2.0 *
            std::atan2(std::sqrt(a), std::sqrt(std::max(0.0, 1.0 - a)));
    }

    std::string NormaliseRunway(std::string value)
    {
        value = ToUpper(Trim(value));

        const auto removePrefix = [&value](const std::string& prefix)
        {
            if (value.rfind(prefix, 0) == 0)
                value.erase(0, prefix.size());
        };

        removePrefix("RUNWAY");
        removePrefix("RWY");

        std::string clean;
        for (unsigned char character : value)
        {
            if (std::isalnum(character) != 0)
                clean.push_back(static_cast<char>(character));
        }

        std::string digits;
        std::string suffix;
        for (char character : clean)
        {
            if (std::isdigit(static_cast<unsigned char>(character)) != 0 && suffix.empty())
                digits.push_back(character);
            else if (std::isalpha(static_cast<unsigned char>(character)) != 0)
                suffix.push_back(character);
        }

        int runwayNumber = 0;
        if (!digits.empty() && ParseInt(digits, runwayNumber) &&
            runwayNumber >= 1 && runwayNumber <= 36)
        {
            char formatted[8] = {};
            std::snprintf(formatted, sizeof(formatted), "%02d", runwayNumber);
            return std::string(formatted) + suffix;
        }

        return clean;
    }

    bool RunwayTextMatches(const std::string& text, const std::string& runway)
    {
        const std::string target = NormaliseRunway(runway);
        if (target.empty())
            return false;

        std::string current;
        const std::string upper = ToUpper(text);
        for (std::size_t index = 0; index <= upper.size(); ++index)
        {
            const char character = index < upper.size() ? upper[index] : '/';
            if (std::isalnum(static_cast<unsigned char>(character)) != 0)
            {
                current.push_back(character);
                continue;
            }

            if (!current.empty())
            {
                if (NormaliseRunway(current) == target)
                    return true;
                current.clear();
            }
        }

        return false;
    }

    TaxiConfig ReadConfig()
    {
        TaxiConfig config;
        config.rawContent = ReadWholeFile(gConfigPath);
        config.filePresent = !config.rawContent.empty();

        if (!config.filePresent)
            return config;

        std::istringstream stream(config.rawContent);
        std::string line;

        while (std::getline(stream, line))
        {
            line = Trim(line);
            if (line.empty() || line[0] == '#')
                continue;

            const std::size_t separator = line.find('=');
            if (separator == std::string::npos)
                continue;

            const std::string key = ToLower(Trim(line.substr(0, separator)));
            const std::string value = Trim(line.substr(separator + 1));

            if (key == "enabled")
                config.enabled = ParseBool(value, config.enabled);
            else if (key == "airport" || key == "departure_icao")
                config.airport = ToUpper(value);
            else if (key == "runway" || key == "departure_runway")
                config.runway = NormaliseRunway(value);
            else if (key == "phase")
                config.phase = value;
            else if (key == "generated_unix")
            {
                try
                {
                    config.generatedUnix = std::stoll(value);
                }
                catch (...)
                {
                    config.generatedUnix = 0;
                }
            }
        }

        config.contextKey =
            std::string(config.enabled ? "1" : "0") + "|" +
            ToUpper(config.airport) + "|" +
            NormaliseRunway(config.runway) + "|" +
            ToLower(config.phase);

        return config;
    }

    std::string DetectNearestAirport()
    {
        const GeoPoint aircraft = CurrentAircraftGeoPoint();
        float latitude = static_cast<float>(aircraft.latitude);
        float longitude = static_cast<float>(aircraft.longitude);

        const XPLMNavRef airportRef = XPLMFindNavAid(
            nullptr, nullptr, &latitude, &longitude, nullptr, xplm_Nav_Airport);

        if (airportRef == XPLM_NAV_NOT_FOUND)
            return {};

        char identifier[64] = {};
        char name[256] = {};
        char region = 0;
        XPLMGetNavAidInfo(
            airportRef,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            identifier,
            name,
            &region);

        return ToUpper(Trim(identifier));
    }

    std::vector<std::string> BuildAptDataCandidates()
    {
        char systemPathBuffer[2048] = {};
        XPLMGetSystemPath(systemPathBuffer);
        const std::string xPlaneRoot = systemPathBuffer;
        const char separator = XPLMGetDirectorySeparator()[0];

        std::vector<std::string> candidates;
        std::unordered_set<std::string> seen;

        const auto addCandidate = [&](const std::string& path)
        {
            if (!FileExists(path))
                return;

            const std::string comparison = NormalisePathForComparison(path);
            if (seen.insert(comparison).second)
                candidates.push_back(path);
        };

        const std::string sceneryIni = JoinPath(xPlaneRoot, "Custom Scenery/scenery_packs.ini");
        std::ifstream sceneryStream(sceneryIni);
        std::string line;

        while (std::getline(sceneryStream, line))
        {
            line = Trim(line);
            const std::string enabledPrefix = "SCENERY_PACK ";
            const std::string disabledPrefix = "SCENERY_PACK_DISABLED ";

            if (line.rfind(disabledPrefix, 0) == 0 ||
                line.rfind(enabledPrefix, 0) != 0)
            {
                continue;
            }

            std::string packPath = Trim(line.substr(enabledPrefix.size()));
            if (packPath.empty())
                continue;

            std::replace(packPath.begin(), packPath.end(), '/', separator);
            std::replace(packPath.begin(), packPath.end(), '\\', separator);

            const bool absolutePath =
                (packPath.size() > 2 && packPath[1] == ':') ||
                (!packPath.empty() && (packPath.front() == '/' || packPath.front() == '\\'));

            const std::string absolutePack = absolutePath
                ? packPath
                : JoinPath(xPlaneRoot, packPath);

            addCandidate(JoinPath(absolutePack, "Earth nav data/apt.dat"));
        }

        addCandidate(JoinPath(xPlaneRoot,
            "Custom Scenery/Global Airports/Earth nav data/apt.dat"));
        addCandidate(JoinPath(xPlaneRoot,
            "Resources/default scenery/default apt dat/Earth nav data/apt.dat"));

        return candidates;
    }

    bool BlockMatchesAirport(
        const std::vector<std::string>& blockLines,
        const std::string& airportIdentifier)
    {
        const std::string target = ToUpper(Trim(airportIdentifier));
        if (blockLines.empty() || target.empty())
            return false;

        const std::vector<std::string> header = SplitWhitespace(blockLines.front());
        if (header.size() >= 5 && ToUpper(header[4]) == target)
            return true;

        for (const std::string& line : blockLines)
        {
            const std::vector<std::string> fields = SplitWhitespace(line);
            if (fields.size() < 3 || fields[0] != "1302")
                continue;

            const std::string key = ToLower(fields[1]);
            if ((key == "icao_code" || key == "icao_id" ||
                 key == "local_code" || key == "faa_code") &&
                ToUpper(fields[2]) == target)
            {
                return true;
            }
        }

        return false;
    }

    bool FindAirportBlock(
        const std::string& aptPath,
        const std::string& airportIdentifier,
        std::vector<std::string>& blockLines)
    {
        std::ifstream stream(aptPath);
        if (!stream)
            return false;

        std::vector<std::string> currentBlock;
        std::string line;

        const auto processCurrent = [&]() -> bool
        {
            if (BlockMatchesAirport(currentBlock, airportIdentifier))
            {
                blockLines = currentBlock;
                return true;
            }
            return false;
        };

        while (std::getline(stream, line))
        {
            const std::vector<std::string> fields = SplitWhitespace(line);
            if (fields.empty())
                continue;

            const bool airportHeader =
                fields[0] == "1" || fields[0] == "16" || fields[0] == "17";

            if (airportHeader)
            {
                if (processCurrent())
                    return true;
                currentBlock.clear();
            }

            if (!currentBlock.empty() || airportHeader)
                currentBlock.push_back(line);
        }

        return processCurrent();
    }

    bool LoadEffectiveAirportBlock(
        const std::string& airportIdentifier,
        std::vector<std::string>& blockLines,
        std::string& sourcePath)
    {
        const std::vector<std::string> candidates = BuildAptDataCandidates();

        for (const std::string& candidate : candidates)
        {
            if (FindAirportBlock(candidate, airportIdentifier, blockLines))
            {
                sourcePath = candidate;
                return true;
            }
        }

        return false;
    }

    bool ParseAirportData(
        const std::vector<std::string>& lines,
        const std::string& sourcePath,
        AirportData& airport)
    {
        airport = AirportData{};
        airport.sourcePath = sourcePath;
        TaxiEdge* previousEdge = nullptr;

        for (const std::string& line : lines)
        {
            const std::vector<std::string> fields = SplitWhitespace(line);
            if (fields.empty())
                continue;

            if ((fields[0] == "1" || fields[0] == "16" || fields[0] == "17") &&
                fields.size() >= 5)
            {
                airport.identifier = ToUpper(fields[4]);
                previousEdge = nullptr;
                continue;
            }

            if (fields[0] == "1302" && fields.size() >= 3)
            {
                const std::string key = ToLower(fields[1]);
                if (key == "icao_code" || key == "icao_id")
                    airport.identifier = ToUpper(fields[2]);
                previousEdge = nullptr;
                continue;
            }

            if (fields[0] == "100" && fields.size() >= 20)
            {
                double latitude1 = 0.0;
                double longitude1 = 0.0;
                double latitude2 = 0.0;
                double longitude2 = 0.0;

                if (ParseDouble(fields[9], latitude1) &&
                    ParseDouble(fields[10], longitude1) &&
                    ParseDouble(fields[18], latitude2) &&
                    ParseDouble(fields[19], longitude2))
                {
                    airport.runwayEnds.push_back({NormaliseRunway(fields[8]), {latitude1, longitude1}});
                    airport.runwayEnds.push_back({NormaliseRunway(fields[17]), {latitude2, longitude2}});
                }

                previousEdge = nullptr;
                continue;
            }

            if (fields[0] == "1201" && fields.size() >= 5)
            {
                double latitude = 0.0;
                double longitude = 0.0;
                int nodeId = -1;

                if (!ParseDouble(fields[1], latitude) ||
                    !ParseDouble(fields[2], longitude) ||
                    !ParseInt(fields[4], nodeId))
                {
                    previousEdge = nullptr;
                    continue;
                }

                TaxiNode node;
                node.id = nodeId;
                node.position = {latitude, longitude};
                node.usage = fields[3];

                if (fields.size() >= 6)
                {
                    node.name = fields[5];
                    for (std::size_t index = 6; index < fields.size(); ++index)
                        node.name += " " + fields[index];
                }

                airport.nodes[node.id] = node;
                previousEdge = nullptr;
                continue;
            }

            if (fields[0] == "1202" && fields.size() >= 5)
            {
                TaxiEdge edge;
                if (!ParseInt(fields[1], edge.from) || !ParseInt(fields[2], edge.to))
                {
                    previousEdge = nullptr;
                    continue;
                }

                edge.twoWay = ToLower(fields[3]) == "twoway";
                edge.restriction = ToLower(fields[4]);
                edge.runway = edge.restriction == "runway";

                if (fields.size() >= 6)
                {
                    edge.name = fields[5];
                    for (std::size_t index = 6; index < fields.size(); ++index)
                        edge.name += " " + fields[index];
                }

                airport.edges.push_back(edge);
                previousEdge = &airport.edges.back();
                continue;
            }

            if (fields[0] == "1204" && fields.size() >= 3 && previousEdge)
            {
                for (const std::string& runway : Split(fields[2], ','))
                {
                    const std::string normalised = NormaliseRunway(runway);
                    if (!normalised.empty())
                        previousEdge->activeRunways.push_back(normalised);
                }
                continue;
            }

            previousEdge = nullptr;
        }

        return !airport.nodes.empty() && !airport.edges.empty() && !airport.runwayEnds.empty();
    }

    const RunwayEnd* SelectRunwayEnd(
        const AirportData& airport,
        const std::string& requestedRunway,
        const GeoPoint& aircraft)
    {
        const std::string target = NormaliseRunway(requestedRunway);

        if (!target.empty())
        {
            for (const RunwayEnd& runwayEnd : airport.runwayEnds)
            {
                if (NormaliseRunway(runwayEnd.identifier) == target)
                    return &runwayEnd;
            }
        }

        const RunwayEnd* nearest = nullptr;
        double nearestDistance = std::numeric_limits<double>::max();

        for (const RunwayEnd& runwayEnd : airport.runwayEnds)
        {
            const double distance = DistanceMetres(aircraft, runwayEnd.position);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearest = &runwayEnd;
            }
        }

        return nearest;
    }

    int FindNearestNode(
        const AirportData& airport,
        const GeoPoint& point,
        double* distanceOut = nullptr,
        const std::unordered_set<int>* candidates = nullptr)
    {
        int nearestId = -1;
        double nearestDistance = std::numeric_limits<double>::max();

        for (const auto& entry : airport.nodes)
        {
            if (candidates && candidates->find(entry.first) == candidates->end())
                continue;

            const double distance = DistanceMetres(point, entry.second.position);
            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestId = entry.first;
            }
        }

        if (distanceOut)
            *distanceOut = nearestDistance;

        return nearestId;
    }

    bool EdgeTargetsRunway(const TaxiEdge& edge, const std::string& runway)
    {
        if (RunwayTextMatches(edge.name, runway))
            return true;

        const std::string target = NormaliseRunway(runway);
        return std::find(edge.activeRunways.begin(), edge.activeRunways.end(), target) !=
            edge.activeRunways.end();
    }

    bool FindShortestPath(
        const AirportData& airport,
        int startNode,
        int targetNode,
        const std::string& targetRunway,
        std::vector<int>& path)
    {
        std::unordered_map<int, std::vector<AdjacentNode>> adjacency;

        for (const TaxiEdge& edge : airport.edges)
        {
            const auto from = airport.nodes.find(edge.from);
            const auto to = airport.nodes.find(edge.to);
            if (from == airport.nodes.end() || to == airport.nodes.end())
                continue;

            double cost = DistanceMetres(from->second.position, to->second.position);
            const bool targetRunwayEdge = EdgeTargetsRunway(edge, targetRunway);

            if (edge.runway && !targetRunwayEdge)
                cost *= 4.0;
            else if (edge.runway)
                cost *= 1.15;

            adjacency[edge.from].push_back({edge.to, cost});

            if (edge.twoWay)
                adjacency[edge.to].push_back({edge.from, cost});
            else
                adjacency[edge.to].push_back({edge.from, cost * 40.0});
        }

        using QueueValue = std::pair<double, int>;
        std::priority_queue<QueueValue, std::vector<QueueValue>, std::greater<QueueValue>> queue;
        std::unordered_map<int, double> distance;
        std::unordered_map<int, int> previous;

        distance[startNode] = 0.0;
        queue.push({0.0, startNode});

        while (!queue.empty())
        {
            const auto [currentDistance, currentNode] = queue.top();
            queue.pop();

            const auto known = distance.find(currentNode);
            if (known == distance.end() || currentDistance > known->second)
                continue;

            if (currentNode == targetNode)
                break;

            const auto adjacent = adjacency.find(currentNode);
            if (adjacent == adjacency.end())
                continue;

            for (const AdjacentNode& next : adjacent->second)
            {
                const double candidate = currentDistance + next.cost;
                const auto existing = distance.find(next.nodeId);
                if (existing == distance.end() || candidate < existing->second)
                {
                    distance[next.nodeId] = candidate;
                    previous[next.nodeId] = currentNode;
                    queue.push({candidate, next.nodeId});
                }
            }
        }

        if (distance.find(targetNode) == distance.end())
            return false;

        std::vector<int> reversed;
        int current = targetNode;
        reversed.push_back(current);

        while (current != startNode)
        {
            const auto iterator = previous.find(current);
            if (iterator == previous.end())
                return false;
            current = iterator->second;
            reversed.push_back(current);
        }

        path.assign(reversed.rbegin(), reversed.rend());
        return path.size() >= 2;
    }

    double RouteLengthMetres(const std::vector<GeoPoint>& points)
    {
        double length = 0.0;
        for (std::size_t index = 1; index < points.size(); ++index)
            length += DistanceMetres(points[index - 1], points[index]);
        return length;
    }

    void DestroyInstances()
    {
        for (XPLMInstanceRef instance : gLightInstances)
        {
            if (instance)
                XPLMDestroyInstance(instance);
        }
        gLightInstances.clear();
    }

    void DestroyResources()
    {
        DestroyInstances();

        if (gTaxiLightObject)
        {
            XPLMUnloadObject(gTaxiLightObject);
            gTaxiLightObject = nullptr;
        }

        if (gTerrainProbe)
        {
            XPLMDestroyProbe(gTerrainProbe);
            gTerrainProbe = nullptr;
        }
    }

    bool EnsureResourcesLoaded()
    {
        if (gTaxiLightObject && gTerrainProbe && gLightInstances.size() == LIGHT_COUNT)
            return true;

        DestroyResources();

        if (gTaxiLightObjectRelativePath.empty())
        {
            LogMessage("AeroPath Traffic: Taxi-route light object path is unavailable");
            return false;
        }

        gTaxiLightObject = XPLMLoadObject(gTaxiLightObjectRelativePath.c_str());
        if (!gTaxiLightObject)
        {
            LogMessage("AeroPath Traffic: Could not load taxi-route object '%s'",
                gTaxiLightObjectRelativePath.c_str());
            return false;
        }

        gTerrainProbe = XPLMCreateProbe(xplm_ProbeY);
        if (!gTerrainProbe)
        {
            LogMessage("AeroPath Traffic: Could not create taxi-route terrain probe");
            DestroyResources();
            return false;
        }

        const char* instanceDataRefs[] = {nullptr};
        for (std::size_t index = 0; index < LIGHT_COUNT; ++index)
        {
            XPLMInstanceRef instance = XPLMCreateInstance(gTaxiLightObject, instanceDataRefs);
            if (!instance)
            {
                LogMessage("AeroPath Traffic: Could not create taxi-route light %zu", index + 1);
                DestroyResources();
                return false;
            }
            gLightInstances.push_back(instance);
        }

        LogMessage("AeroPath Traffic: Loaded %zu taxi-route light instances", gLightInstances.size());
        return true;
    }

    void HideAllLights()
    {
        float unusedData = 0.0f;
        for (XPLMInstanceRef instance : gLightInstances)
        {
            XPLMDrawInfo_t drawInfo = {};
            drawInfo.structSize = sizeof(drawInfo);
            drawInfo.y = -100000.0f;
            XPLMInstanceSetPosition(instance, &drawInfo, &unusedData);
        }
    }

    void BuildStraightFallback()
    {
        gRoutePoints.clear();
        gRouteValid = false;
        gUsingStraightFallback = true;
    }

    bool BuildTaxiRoute(const TaxiConfig& config)
    {
        const GeoPoint aircraft = CurrentAircraftGeoPoint();
        std::string airportIdentifier = ToUpper(Trim(config.airport));
        if (airportIdentifier.empty())
            airportIdentifier = DetectNearestAirport();

        if (airportIdentifier.empty())
        {
            LogMessage("AeroPath Traffic: Taxi route could not identify the nearest airport");
            BuildStraightFallback();
            return false;
        }

        std::vector<std::string> airportBlock;
        std::string aptPath;
        if (!LoadEffectiveAirportBlock(airportIdentifier, airportBlock, aptPath))
        {
            LogMessage("AeroPath Traffic: No apt.dat airport block found for %s",
                airportIdentifier.c_str());
            BuildStraightFallback();
            return false;
        }

        AirportData airport;
        if (!ParseAirportData(airportBlock, aptPath, airport))
        {
            LogMessage("AeroPath Traffic: %s has no usable aircraft taxi network in '%s'",
                airportIdentifier.c_str(), aptPath.c_str());
            BuildStraightFallback();
            return false;
        }

        const RunwayEnd* runwayEnd = SelectRunwayEnd(airport, config.runway, aircraft);
        if (!runwayEnd)
        {
            LogMessage("AeroPath Traffic: %s has no usable runway ends", airportIdentifier.c_str());
            BuildStraightFallback();
            return false;
        }

        double startDistance = 0.0;
        const int startNode = FindNearestNode(airport, aircraft, &startDistance);
        if (startNode < 0 || startDistance > MAX_START_NODE_DISTANCE_METRES)
        {
            LogMessage("AeroPath Traffic: Nearest %s taxi node is %.0f m away; route rejected",
                airportIdentifier.c_str(), startDistance);
            BuildStraightFallback();
            return false;
        }

        std::unordered_set<int> runwayCandidates;
        for (const TaxiEdge& edge : airport.edges)
        {
            if (EdgeTargetsRunway(edge, runwayEnd->identifier))
            {
                runwayCandidates.insert(edge.from);
                runwayCandidates.insert(edge.to);
            }
        }

        double targetDistance = 0.0;
        int targetNode = FindNearestNode(
            airport,
            runwayEnd->position,
            &targetDistance,
            runwayCandidates.empty() ? nullptr : &runwayCandidates);

        if (targetNode < 0)
        {
            LogMessage("AeroPath Traffic: Could not find a taxi-network entry for runway %s",
                runwayEnd->identifier.c_str());
            BuildStraightFallback();
            return false;
        }

        std::vector<int> pathNodeIds;
        if (!FindShortestPath(
                airport,
                startNode,
                targetNode,
                runwayEnd->identifier,
                pathNodeIds))
        {
            LogMessage("AeroPath Traffic: No connected taxi path from the aircraft to %s runway %s",
                airportIdentifier.c_str(), runwayEnd->identifier.c_str());
            BuildStraightFallback();
            return false;
        }

        std::vector<GeoPoint> route;
        route.push_back(aircraft);
        for (int nodeId : pathNodeIds)
        {
            const auto node = airport.nodes.find(nodeId);
            if (node == airport.nodes.end())
                continue;

            if (route.empty() || DistanceMetres(route.back(), node->second.position) > 0.5)
                route.push_back(node->second.position);
        }

        if (route.size() < 2)
        {
            BuildStraightFallback();
            return false;
        }

        gRoutePoints = std::move(route);
        gRouteValid = true;
        gUsingStraightFallback = false;
        gActiveAirport = airportIdentifier;
        gActiveRunway = runwayEnd->identifier;
        gActiveAptPath = aptPath;

        LogMessage(
            "AeroPath Traffic: Taxi route ready %s -> RWY %s: %zu graph nodes, %.0f m, source '%s'",
            gActiveAirport.c_str(),
            gActiveRunway.c_str(),
            pathNodeIds.size(),
            RouteLengthMetres(gRoutePoints),
            gActiveAptPath.c_str());

        return true;
    }

    std::vector<LocalPoint> BuildLocalRoute()
    {
        std::vector<LocalPoint> route;
        route.reserve(gRoutePoints.size());
        for (const GeoPoint& point : gRoutePoints)
            route.push_back(ToLocal(point));
        return route;
    }

    double DistanceSquaredXZ(const LocalPoint& first, const LocalPoint& second)
    {
        const double dx = first.x - second.x;
        const double dz = first.z - second.z;
        return dx * dx + dz * dz;
    }

    struct RouteProjection
    {
        double alongDistance = 0.0;
        double distanceFromRoute = std::numeric_limits<double>::max();
    };

    RouteProjection ProjectOntoRoute(
        const std::vector<LocalPoint>& route,
        const LocalPoint& aircraft)
    {
        RouteProjection best;
        double accumulated = 0.0;

        for (std::size_t index = 1; index < route.size(); ++index)
        {
            const LocalPoint& first = route[index - 1];
            const LocalPoint& second = route[index];
            const double dx = second.x - first.x;
            const double dz = second.z - first.z;
            const double lengthSquared = dx * dx + dz * dz;
            const double length = std::sqrt(lengthSquared);

            if (length < 0.01)
                continue;

            double progress =
                ((aircraft.x - first.x) * dx + (aircraft.z - first.z) * dz) /
                lengthSquared;
            progress = std::clamp(progress, 0.0, 1.0);

            LocalPoint projected;
            projected.x = first.x + dx * progress;
            projected.z = first.z + dz * progress;

            const double distance = std::sqrt(DistanceSquaredXZ(aircraft, projected));
            if (distance < best.distanceFromRoute)
            {
                best.distanceFromRoute = distance;
                best.alongDistance = accumulated + length * progress;
            }

            accumulated += length;
        }

        return best;
    }

    bool PointAtRouteDistance(
        const std::vector<LocalPoint>& route,
        double requestedDistance,
        LocalPoint& point,
        double& heading)
    {
        double accumulated = 0.0;

        for (std::size_t index = 1; index < route.size(); ++index)
        {
            const LocalPoint& first = route[index - 1];
            const LocalPoint& second = route[index];
            const double dx = second.x - first.x;
            const double dz = second.z - first.z;
            const double length = std::sqrt(dx * dx + dz * dz);

            if (length < 0.01)
                continue;

            if (requestedDistance <= accumulated + length)
            {
                const double progress =
                    std::clamp((requestedDistance - accumulated) / length, 0.0, 1.0);
                point.x = first.x + dx * progress;
                point.y = first.y + (second.y - first.y) * progress;
                point.z = first.z + dz * progress;
                heading = NormaliseHeading(RadiansToDegrees(std::atan2(dx, -dz)));
                return true;
            }

            accumulated += length;
        }

        return false;
    }

    double LocalRouteLength(const std::vector<LocalPoint>& route)
    {
        double length = 0.0;
        for (std::size_t index = 1; index < route.size(); ++index)
        {
            const double dx = route[index].x - route[index - 1].x;
            const double dz = route[index].z - route[index - 1].z;
            length += std::sqrt(dx * dx + dz * dz);
        }
        return length;
    }

    void UpdateStraightFallback()
    {
        const double localX = XPLMGetDatad(gLocalX);
        const double localY = XPLMGetDatad(gLocalY);
        const double localZ = XPLMGetDatad(gLocalZ);
        const double headingDegrees = NormaliseHeading(XPLMGetDataf(gHeading));
        const double headingRadians = DegreesToRadians(headingDegrees);
        float unusedData = 0.0f;

        for (std::size_t index = 0; index < gLightInstances.size(); ++index)
        {
            const double distance = LIGHT_FIRST_OFFSET_METRES + index * LIGHT_SPACING_METRES;
            const double candidateX = localX + std::sin(headingRadians) * distance;
            const double candidateZ = localZ - std::cos(headingRadians) * distance;

            XPLMProbeInfo_t probeInfo = {};
            probeInfo.structSize = sizeof(probeInfo);
            const XPLMProbeResult result = XPLMProbeTerrainXYZ(
                gTerrainProbe,
                static_cast<float>(candidateX),
                static_cast<float>(localY),
                static_cast<float>(candidateZ),
                &probeInfo);

            XPLMDrawInfo_t drawInfo = {};
            drawInfo.structSize = sizeof(drawInfo);
            if (result == xplm_ProbeHitTerrain)
            {
                drawInfo.x = probeInfo.locationX;
                drawInfo.y = probeInfo.locationY + LIGHT_HEIGHT_OFFSET_METRES;
                drawInfo.z = probeInfo.locationZ;
            }
            else
            {
                drawInfo.y = -100000.0f;
            }
            drawInfo.heading = static_cast<float>(headingDegrees);
            XPLMInstanceSetPosition(gLightInstances[index], &drawInfo, &unusedData);
        }
    }

    void UpdateRouteLights()
    {
        if (!gRouteValid || gRoutePoints.size() < 2)
        {
            UpdateStraightFallback();
            return;
        }

        const std::vector<LocalPoint> route = BuildLocalRoute();
        if (route.size() < 2)
        {
            UpdateStraightFallback();
            return;
        }

        LocalPoint aircraft;
        aircraft.x = XPLMGetDatad(gLocalX);
        aircraft.y = XPLMGetDatad(gLocalY);
        aircraft.z = XPLMGetDatad(gLocalZ);

        const RouteProjection projection = ProjectOntoRoute(route, aircraft);
        const double remainingDistance =
            std::max(0.0, LocalRouteLength(route) - projection.alongDistance);

        if (gAutomaticVisible && !gManualVisible &&
            remainingDistance <= ROUTE_COMPLETION_DISTANCE_METRES &&
            projection.distanceFromRoute <= 45.0)
        {
            gRouteCompleted = true;
            HideAllLights();
            LogMessage(
                "AeroPath Traffic: Taxi route complete at runway %s; automatic lights hidden",
                gActiveRunway.empty() ? "entry" : gActiveRunway.c_str());
            return;
        }

        if (projection.distanceFromRoute > OFF_ROUTE_REBUILD_DISTANCE_METRES &&
            XPLMGetElapsedTime() - gLastRouteAttemptSeconds > ROUTE_RETRY_INTERVAL_SECONDS)
        {
            AeroPathTaxiGuidance::RebuildRoute();
            return;
        }

        float unusedData = 0.0f;
        const double firstDistance = projection.alongDistance + LIGHT_FIRST_OFFSET_METRES;

        for (std::size_t index = 0; index < gLightInstances.size(); ++index)
        {
            LocalPoint candidate;
            double heading = 0.0;
            const double routeDistance = firstDistance + index * LIGHT_SPACING_METRES;

            XPLMDrawInfo_t drawInfo = {};
            drawInfo.structSize = sizeof(drawInfo);

            if (!PointAtRouteDistance(route, routeDistance, candidate, heading))
            {
                drawInfo.y = -100000.0f;
                XPLMInstanceSetPosition(gLightInstances[index], &drawInfo, &unusedData);
                continue;
            }

            XPLMProbeInfo_t probeInfo = {};
            probeInfo.structSize = sizeof(probeInfo);
            const XPLMProbeResult result = XPLMProbeTerrainXYZ(
                gTerrainProbe,
                static_cast<float>(candidate.x),
                static_cast<float>(aircraft.y),
                static_cast<float>(candidate.z),
                &probeInfo);

            if (result == xplm_ProbeHitTerrain)
            {
                drawInfo.x = probeInfo.locationX;
                drawInfo.y = probeInfo.locationY + LIGHT_HEIGHT_OFFSET_METRES;
                drawInfo.z = probeInfo.locationZ;
                drawInfo.heading = static_cast<float>(heading);
            }
            else
            {
                drawInfo.y = -100000.0f;
            }

            XPLMInstanceSetPosition(gLightInstances[index], &drawInfo, &unusedData);
        }
    }
}

namespace AeroPathTaxiGuidance
{
    bool Initialise(const std::string& pluginResourcesPath)
    {
        Shutdown();

        gPluginResourcesPath = pluginResourcesPath;
        gConfigPath = JoinPath(pluginResourcesPath, "AeroPathTaxiGuidance.txt");

        const std::string objectAbsolutePath = JoinPath(
            JoinPath(pluginResourcesPath, "TaxiGuidance"),
            "AeroPathTaxiLight.obj");

        gTaxiLightObjectRelativePath = MakePathRelativeToXPlaneRoot(objectAbsolutePath);

        gLocalX = XPLMFindDataRef("sim/flightmodel/position/local_x");
        gLocalY = XPLMFindDataRef("sim/flightmodel/position/local_y");
        gLocalZ = XPLMFindDataRef("sim/flightmodel/position/local_z");
        gHeading = XPLMFindDataRef("sim/flightmodel/position/psi");

        gInitialised =
            gLocalX && gLocalY && gLocalZ && gHeading &&
            !gTaxiLightObjectRelativePath.empty();

        if (!gInitialised)
        {
            LogMessage("AeroPath Traffic: Taxi-route guidance initialisation failed");
            return false;
        }

        LogMessage("AeroPath Traffic: Taxi-route guidance ready; config '%s'",
            gConfigPath.c_str());
        return true;
    }

    void Shutdown()
    {
        gManualVisible = false;
        gAutomaticVisible = false;
        gRouteCompleted = false;
        gInitialised = false;
        gRouteValid = false;
        gUsingStraightFallback = false;
        gRoutePoints.clear();
        gLastConfigContent.clear();
        gActiveAirport.clear();
        gActiveRunway.clear();
        gActiveAptPath.clear();
        DestroyResources();
    }

    void SetVisible(bool visible)
    {
        gManualVisible = visible;

        if (!ShouldRenderGuidance())
        {
            HideAllLights();
            LogMessage("AeroPath Traffic: Manual taxi-route lights disabled");
            return;
        }

        if (!gInitialised || !EnsureResourcesLoaded())
        {
            gManualVisible = false;
            return;
        }

        gRouteCompleted = false;
        RebuildRoute();
        Update();

        LogMessage(
            visible
                ? "AeroPath Traffic: Manual taxi-route lights enabled"
                : "AeroPath Traffic: Manual taxi-route lights disabled; automatic guidance remains active");
    }

    bool IsVisible()
    {
        return ShouldRenderGuidance();
    }

    bool IsManualVisible()
    {
        return gManualVisible;
    }

    bool IsAutomaticVisible()
    {
        return gAutomaticVisible;
    }

    void ToggleVisible()
    {
        SetVisible(!gManualVisible);
    }

    void RebuildRoute()
    {
        if (!gInitialised)
            return;

        gLastRouteAttemptSeconds = XPLMGetElapsedTime();
        const TaxiConfig config = ReadConfig();
        gLastConfigContent = config.contextKey;
        gRouteCompleted = false;

        if (config.filePresent && !config.enabled && !gManualVisible)
        {
            gRouteValid = false;
            gUsingStraightFallback = false;
            gRoutePoints.clear();
            HideAllLights();
            return;
        }

        BuildTaxiRoute(config);
    }

    void Update()
    {
        if (!gInitialised)
            return;

        const double now = XPLMGetElapsedTime();
        if (now - gLastConfigCheckSeconds >= CONFIG_CHECK_INTERVAL_SECONDS)
        {
            gLastConfigCheckSeconds = now;
            const TaxiConfig config = ReadConfig();
            const bool automaticRequested =
                config.filePresent &&
                config.enabled &&
                IsConfigFresh(config);

            if (automaticRequested != gAutomaticVisible)
            {
                gAutomaticVisible = automaticRequested;
                gRouteCompleted = false;

                LogMessage(
                    automaticRequested
                        ? "AeroPath Traffic: Automatic taxi guidance activated by AeroPath"
                        : "AeroPath Traffic: Automatic taxi guidance deactivated");

                if (automaticRequested)
                    RebuildRoute();
            }

            if (config.contextKey != gLastConfigContent)
            {
                gLastConfigContent = config.contextKey;
                if (ShouldRenderGuidance())
                    RebuildRoute();
            }

            if (!ShouldRenderGuidance())
            {
                HideAllLights();
                return;
            }
        }

        if (!ShouldRenderGuidance() || gRouteCompleted)
        {
            HideAllLights();
            return;
        }

        if (!EnsureResourcesLoaded())
        {
            gManualVisible = false;
            gAutomaticVisible = false;
            return;
        }

        if (!gRouteValid && !gUsingStraightFallback &&
            now - gLastRouteAttemptSeconds >= ROUTE_RETRY_INTERVAL_SECONDS)
        {
            RebuildRoute();
        }

        UpdateRouteLights();
    }

}
