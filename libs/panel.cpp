#include "panel.h"

#include <array>

namespace panel {

// default values
bool showPanel = true;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

// visualization
int curveSamples = 200;

// loading
bool rereadControlPoints = false;
std::string controlPointsFilePath = "./roller_coaster.obj";

// animation
bool play = false;

// reset
bool resetView = false;
bool single_step = false;

void updateMenu() {
  using namespace ImGui;

  giv::io::ImGuiBeginFrame();

  if (showPanel && Begin("panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
    if (BeginMenuBar()) {
      if (BeginMenu("File")) {
        if (MenuItem("Close", "(P)")) {
          showPanel = false;
        }
        // add more if you would like...
        ImGui::EndMenu();
      }
      EndMenuBar();
    }


    Spacing();
    Separator();
    if (Button("Play/Pause")) {
      play = !play;
    }
    
    if (!play) {
      if (Button("Single Step")) {     
        single_step = true;
      }
    }

    Spacing();
    Separator();
    resetView = Button("Reset view");

    Spacing();
    Separator();
    Text("Application average %.3f ms/frame (%.1f FPS)",
         1000.0f / GetIO().Framerate, GetIO().Framerate);

    End();
  }
  giv::io::ImGuiEndFrame();

  
}

} // namespace panel
