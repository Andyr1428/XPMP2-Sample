#pragma once

#include <string>

namespace AeroPathTaxiGuidance
{
    bool Initialise(const std::string& pluginResourcesPath);
    void Shutdown();

    void SetVisible(bool visible);
    bool IsVisible();
    bool IsManualVisible();
    bool IsAutomaticVisible();
    void ToggleVisible();

    void RebuildRoute();
    void Update();
}
