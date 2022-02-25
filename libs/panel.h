#pragma once

#include <iosfwd>
#include <string>

#include "givio.h"
#include "givr.h"
#include "imgui/imgui.h"

namespace panel {

extern bool showPanel;
extern ImVec4 clear_color;

// visualization
extern int curveSamples;

// loading
extern bool rereadControlPoints;
extern std::string controlPointsFilePath;

// animation
extern bool play;

// reset
extern bool resetView;

//single step
extern bool single_step;

void updateMenu();

} // namespace panel
