#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cstdlib>
#include <iostream>



#include "givio.h"
#include "sim.hpp"
#include "panel.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

int main(void)
{
  namespace givio = giv::io; // perhaps better than giv::io
  givio::GLFWContext glContext;
  glContext.glMajorVesion(4)
      .glMinorVesion(0)
      .glForwardComaptability(true)
      .glCoreProfile()
      .glAntiAliasingSamples(4)
      .matchPrimaryMonitorVideoMode();

  std::cout << givio::glfwVersionString() << '\n';

  //
  // setup window (OpenGL context)
  //
  auto window =
      glContext.makeImGuiWindow(givio::Properties()
                                    .size(givio::dimensions{1000, 1000})
                                    .title("Curve surfing...")
                                    .glslVersionString("#version 330 core"));

  auto view = View(TurnTable(), Perspective());

  view.camera.translate(vec3(0,0,-5));

  auto linestyle = GL_Line(Width(2.), Colour(0.0, 1.0, 0.0));

  PositionList positions;
  LinConstraintList linconstraints;
  AngConstraintList angconstraints;

  float linalpha = .000000001f;
  float angalpha = .00000001f; 
  int nsegs = 50;
  float original_length = 20.f;

  srand((unsigned)time(0));

  float x = -original_length/2;
  float y = 0; 
  positions.emplace_back(vec2(x,y), true);

  for (int n = 1; n < nsegs; n++) {
    x += (original_length/nsegs);
    y = .0001*(rand() - RAND_MAX/2)/float(RAND_MAX);
    positions.emplace_back(vec2(x,y));
  }

  x += (original_length/nsegs);
  y = 0;

  positions.emplace_back(vec2(x,y), true);

  for (auto pos = positions.begin(); pos != positions.end()-1; pos++) {

    linconstraints.emplace_back(pos, pos+1, linalpha);
    if (pos != positions.begin())
     angconstraints.emplace_back(pos-1, pos, pos+1, angalpha);
   }


  std::vector<std::vector<Line>> animation;
  glClearColor(0.f, 0.f, 0.f, 0.f);
  mainloop(std::move(window), [&](float frame_time) {


    bool gravity = false;
    bool bounds = true;
    bool collisions = true;

    if (panel::play){ 
      for (float deltat = 0.f; deltat < frame_time; deltat += .001)
        sim_iteration(positions, linconstraints, angconstraints, .001, gravity, bounds, collisions); //use gravity, don't use bounds

     } else {
      if (panel::single_step) {
        sim_iteration(positions, linconstraints, angconstraints, .001, gravity, bounds, collisions); //use gravity, don't use bounds
        panel::single_step = false;
      }
    }



    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    auto wall = MultiLine();
    for (auto constraint: linconstraints) {

      auto p1 = constraint.pos1;
      auto p2 = constraint.pos2;
      wall.push_back(Line(Point1(p1->position.x, p1->position.y, 0),Point2(p2->position.x, p2->position.y, 0))); 


    }

    auto cellwall = createRenderable(wall, linestyle);
    draw(cellwall, view);
  });
  exit(EXIT_SUCCESS);
}