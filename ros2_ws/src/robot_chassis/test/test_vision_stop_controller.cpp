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
  assert(no_tree.update(in,p,true,.5,1).state==VisionStopController::State::Idle);
  assert(no_tree.rearming());  // Missing tree skips this stop, not the entire route.
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
  // Explicit completion is required; a drifting base cannot release parking.
  in.stopped = false;
  assert(!c.resume_after_harvest(in));
  in.stopped = true;
  in.now = 5.1; in.observation_time = 5.1; in.have_observation = true;
  c.update(in,p,true,.5,-1);
  in.now = 5.4; in.observation_time = 5.4;
  c.update(in,p,true,.5,-1);
  assert(c.resume_after_harvest(in));
  assert(!c.active() && c.rearming());
  in.mango_in_range = true; in.tree_valid = true; in.tree_error = 0.0;
  in.now = 6.5; in.observation_time = 6.5;
  // Elapsed time alone and a still-centered fruit never retrigger.
  assert(c.update(in,p,true,.5,-1).state == VisionStopController::State::Idle);
  in.tree_error = .15;
  c.update(in,p,true,.5,-1);
  in.lateral_position += .20;  // Wrong direction is not route progress.
  assert(c.update(in,p,true,.5,-1).state == VisionStopController::State::Idle);
  in.lateral_position -= .40;
  // An old off-center trunk must not re-enable PI and pull the chassis back.
  in.now = 6.6; in.observation_time = 6.6;
  assert(c.update(in,p,true,.5,-1).state == VisionStopController::State::Idle);
  assert(c.rearming());
  // Adjacent tree, continuously visible mangoes: no empty-scene condition.
  in.tree_error = .01; in.now = 6.6; in.observation_time = 6.6;
  assert(c.update(in,p,true,.5,-1).state == VisionStopController::State::Settling);
  assert(!c.rearming());
  assert(!lost.resume_after_harvest(in));  // A fault is never released by completion.
  // Exercise both route directions and both camera-to-chassis signs. Mangoes
  // remain visible throughout, and the next trunk is acquired before center.
  for (const int direction : {-1, 1}) {
    for (const double sign : {-1.0, 1.0}) {
      VisionStopController two_trees;
      auto params = p;
      params.control_sign = sign;
      params.max_align_speed = .7;
      in = VisionStopController::Input{};
      in.have_observation = in.mango_in_range = in.tree_valid = in.stopped = true;
      two_trees.update(in, params, true, .7, direction);
      in.now = in.observation_time = .3;
      assert(two_trees.update(in, params, true, .7, direction).state == VisionStopController::State::Holding);
      assert(two_trees.resume_after_harvest(in));
      // A centered old trunk must still block departure completion, even
      // after the minimum distance and time have elapsed.
      in.now = in.observation_time = 1.4;
      in.lateral_position = direction * .12;
      out = two_trees.update(in, params, true, .7, direction);
      assert(two_trees.rearming());
      assert(std::abs(out.velocity - direction * .10) < 1e-8);
      // Old trunk behind us: stay on the route instead of reversing back.
      in.tree_error = -direction * sign * .15;
      out = two_trees.update(in, params, true, .7, direction);
      assert(two_trees.rearming());
      // Clearing the old trunk restores cruise speed without requiring the
      // next trunk to be visible, while old-tree suppression stays active.
      assert(std::abs(out.velocity - direction * .7) < 1e-8);
      auto empty_lane = two_trees;
      auto empty_input = in;
      empty_input.tree_valid = empty_input.mango_in_range = false;
      empty_input.now = empty_input.observation_time = 2.0;
      out = empty_lane.update(empty_input, params, true, .7, direction);
      assert(out.state == VisionStopController::State::Idle && empty_lane.rearming());
      assert(std::abs(out.velocity - direction * .7) < 1e-8);
      empty_input.now = empty_input.observation_time = 20.0;
      out = empty_lane.update(empty_input, params, true, .7, direction);
      assert(out.state == VisionStopController::State::Idle);
      assert(std::abs(out.velocity - direction * .7) < 1e-8);
      // Freshness protection remains active after the speed cap is lifted.
      auto stale_lane = empty_lane;
      empty_input.now += params.observation_timeout + .1;
      out = stale_lane.update(empty_input, params, true, .7, direction);
      assert(out.state == VisionStopController::State::Fault && out.velocity == 0);
      // Next tree ahead, outside +/-2%: start centering immediately.
      in.tree_error = direction * sign * .12;
      in.now = in.observation_time = 1.5;
      out = two_trees.update(in, params, true, .7, direction);
      assert(out.state == VisionStopController::State::Aligning);
      assert(!two_trees.rearming() && out.velocity * direction > 0);
      in.tree_error = 0;
      in.now = in.observation_time = 1.6;
      two_trees.update(in, params, true, .7, direction);
      in.now = in.observation_time = 1.9;
      assert(two_trees.update(in, params, true, .7, direction).state == VisionStopController::State::Holding);
    }
  }
  std::cout << "vision stop, explicit resume and adjacent-tree rearming checks passed\n";
}
