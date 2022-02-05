#include "lib/AutoHelper.h"

#include <frc/Filesystem.h>
#include <wpi/fs.h>
#include <wpi/SmallString.h>

#include <lib/pathplanner/PathPlanner.h>

#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryParameterizer.h>

#include <iostream>


namespace rj {

frc::Trajectory AutoHelper::LoadTrajectory(std::string name, frc::TrajectoryConfig *config) {
    // velocity, accel don't matter
    // but let's use the configured ones anyway
    pathplanner::PathPlannerTrajectory pp_traj = pathplanner::PathPlanner::loadPath(name, config->MaxVelocity(), config->MaxAcceleration());

    std::vector<frc::TrajectoryGenerator::PoseWithCurvature> path;

    for (int ind = 0; ind < pp_traj.numStates(); ind++) {
        auto pp_state = pp_traj.getState(ind);

        frc::Rotation2d heading_diff;
        if (ind == 0) {
            // sample this and next heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_state->pose.Rotation();
        } else if (ind == pp_traj.numStates() - 1) {
            // sample this and last heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_traj.getState(ind-1)->pose.Rotation();
        } else {
            // sample last and next heading
            heading_diff = pp_traj.getState(ind+1)->pose.Rotation() - pp_state->pose.Rotation();
        }

        int curv_sign = 0;

        if (heading_diff.Radians() > 0_rad) {
            curv_sign = 1;
        } else if (heading_diff.Radians() < 0_rad) {
            curv_sign = -1;
        }

        auto pose = pp_state->pose;

        if (config->IsReversed()) {
            pose = pp_state->pose.TransformBy(frc::Transform2d(frc::Translation2d(), frc::Rotation2d(180_deg)));
            curv_sign *= -1;
        }

        path.push_back(frc::TrajectoryGenerator::PoseWithCurvature{pose, pp_state->curvature * curv_sign});
    }

    return frc::TrajectoryParameterizer::TimeParameterizeTrajectory(path, config->Constraints(), config->StartVelocity(), config->EndVelocity(), config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed());
}

}