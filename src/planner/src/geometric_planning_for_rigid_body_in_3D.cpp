#include <cstdio>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state){
  //cast the abstract state type to the type we expect
  const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

  //extract the first component of the state and cast it to what we expect
  const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

  //extract the second component of the state and cast it to what we expect
  const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

  //check validity of the state defined by the pos & rot

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return (const void*)rot != (const void*)pos;
}

void plan(){
  //construct the state space we are planning in
  // auto space(sr)
}


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world planner package\n");
  return 0;
}
