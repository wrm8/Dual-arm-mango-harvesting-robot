#include "robot_chassis/vision_stop_controller.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using robot_chassis::VisionStopController;

int main()
{
  VisionStopController c;
  VisionStopController::Parameters p;
  p.kp=1; p.ki=0; p.max_align_speed=.12;
  p.center_tolerance=.02; p.settle_time=.2;
  VisionStopController::Input in;
  in.have_observation=true; in.mango_in_range=true;
  assert(!c.active());
  assert(c.update(in,p,false,.5,-1).state==VisionStopController::State::Idle);
  auto out=c.update(in,p,true,.5,-1);
  assert(out.state==VisionStopController::State::Searching && std::abs(out.velocity+.12)<1e-8);
  in.lateral_position=-.16; in.now=.2; in.observation_time=.2;
  out=c.update(in,p,true,.5,-1);
  assert(std::abs(out.velocity+.12)<1e-8); // Qualified mango directly selects 0.12 m/s.
  in.tree_valid=true; in.tree_error=.1; in.now=.3;in.observation_time=.3;
  out=c.update(in,p,true,.5,-1);
  assert(out.state==VisionStopController::State::Aligning && out.velocity>0);
  assert(std::abs(out.velocity-.1)<1e-8); // PI output is not reduced by a distance-based speed ramp.
  in.tree_error=.02;in.now=.4;in.observation_time=.4;
  out=c.update(in,p,true,.5,-1);
  assert(out.state==VisionStopController::State::Settling && out.velocity==0);
  // The tolerance includes its boundary; error need not be exactly zero.
  // No HOLDING while the wheel feedback still says moving.
  in.now=.9;in.observation_time=.9;
  assert(c.update(in,p,true,.5,-1).state==VisionStopController::State::Settling);
  in.stopped=true;in.now=1;in.observation_time=1;
  c.update(in,p,true,.5,-1);
  in.now=1.3;in.observation_time=1.3;
  assert(c.update(in,p,true,.5,-1).state==VisionStopController::State::Holding);
  // Parked state cannot reaccelerate after camera or mango loss, or parameter disable.
  in.now=5;in.have_observation=false;in.tree_error=.3;
  out=c.update(in,p,false,.5,-1);
  assert(out.state==VisionStopController::State::Holding && out.velocity==0);
  in.stopped=false;
  assert(c.update(in,p,false,.5,-1).state==VisionStopController::State::Settling);

  VisionStopController lost;
  in=VisionStopController::Input{};in.have_observation=true;in.mango_in_range=true;
  in.tree_valid=true;in.tree_error=.2;
  lost.update(in,p,true,.5,1);
  in.now=.7;
  assert(lost.update(in,p,true,.5,1).state==VisionStopController::State::Fault);
  in.observation_time=.7;
  assert(lost.update(in,p,true,.5,1).velocity==0);

  VisionStopController no_tree;
  in=VisionStopController::Input{};in.have_observation=true;in.mango_in_range=true;
  assert(std::abs(no_tree.update(in,p,true,.5,1).velocity-.12)<1e-8);
  in.lateral_position=.36;in.now=.1;in.observation_time=.1;
  assert(no_tree.update(in,p,true,.5,1).velocity==0);
  in.now=16;in.observation_time=16;
  assert(no_tree.update(in,p,true,.5,1).state==VisionStopController::State::Fault);
  VisionStopController signed_image;
  auto signed_parameters=p;
  signed_parameters.control_sign=-1;
  in=VisionStopController::Input{};
  in.have_observation=true;in.mango_in_range=true;in.tree_valid=true;
  in.tree_error=.1;  // Trunk is image-right; configured mapping requests negative chassis Y.
  assert(std::abs(signed_image.update(in,signed_parameters,true,.5,1).velocity+.1)<1e-8);
  in.tree_error=-.1;in.frame_sequence=1;in.now=.1;in.observation_time=.1;
  assert(std::abs(signed_image.update(in,signed_parameters,true,.5,1).velocity-.1)<1e-8);
  in.tree_valid=false;in.now=.2;in.observation_time=.2;
  assert(signed_image.update(in,signed_parameters,true,.5,1).velocity==0);
  in.tree_valid=true;in.tree_error=.3;in.now=.3;in.observation_time=.3;
  assert(std::abs(signed_image.update(in,signed_parameters,true,.5,1).velocity+.12)<1e-8);
  in.tree_error=-.3;in.now=.4;in.observation_time=.4;
  assert(std::abs(signed_image.update(in,signed_parameters,true,.05,1).velocity-.05)<1e-8);
  std::cout << "vision image ratio, 0.12 command, direction, tolerance, measured stop and loss checks passed\n";
}
