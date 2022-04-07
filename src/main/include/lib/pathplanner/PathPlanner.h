#pragma once

#include "lib/pathplanner/PathPlannerTrajectory.h"
#include <units/velocity.h>
#include <units/acceleration.h>
#include <string>
#include <vector>

namespace pathplanner{
    class PathPlanner {
        public:
            static double resolution;

            static wpi::json loadConfig(std::string name);

            static pathplanner::PathPlannerTrajectory loadPath(std::string name, units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel, bool reversed);

            static pathplanner::PathPlannerTrajectory loadPath(std::string name, units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel){
                return PathPlanner::loadPath(name, maxVel, maxAccel, false);
            }

        private:
            static pathplanner::PathPlannerTrajectory joinPaths(std::vector<pathplanner::PathPlannerTrajectory> paths);
    };
}