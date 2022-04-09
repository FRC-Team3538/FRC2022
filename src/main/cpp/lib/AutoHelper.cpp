#include "lib/AutoHelper.h"
#include <frc/trajectory/TrajectoryGenerator.h>      // for TrajectoryGenera...
#include <frc/trajectory/TrajectoryParameterizer.h>  // for TrajectoryParame...
#include <frc/trajectory/Trajectory.h>               // for Trajectory
#include <cstddef>                                   // for size_t
#include <iostream>                                  // for operator<<, endl
#include <vector>                                    // for vector
#include "frc/geometry/Pose2d.h"                     // for Pose2d
#include "frc/geometry/Rotation2d.h"                 // for Rotation2d
#include "frc/trajectory/TrajectoryConfig.h"         // for TrajectoryConfig
#include "units/angle.h"                             // for operator""_rad
#include "units/base.h"                              // for unit_t, operator*
#include "units/curvature.h"                         // for curvature_t
#include "units/time.h"                              // for second_t
#include "lib/pathplanner/PathPlanner.h"
#include "lib/pathplanner/PathPlannerTrajectory.h"
#include "wpi/json.h"

namespace rj
{

    frc::Trajectory AutoHelper::LoadTrajectory(std::string name, frc::TrajectoryConfig *config)
    {        
        // velocity, accel don't matter
        // but let's use the configured ones anyway
        pathplanner::PathPlannerTrajectory pp_traj = pathplanner::PathPlanner::loadPath(name, config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed());

        std::vector<frc::TrajectoryGenerator::PoseWithCurvature> path;
        path.reserve(pp_traj.numStates());

        for (int ind = 0; ind < pp_traj.numStates(); ind++)
        {
            auto pp_state = pp_traj.getState(ind);

            frc::Rotation2d heading_diff;
            if (ind == pp_traj.numStates() - 1)
            {
                // Last point is special, use the previous point instead
                heading_diff = pp_traj.getState(ind)->pose.Rotation() - pp_traj.getState(ind - 1)->pose.Rotation();
            }
            else
            {
                // Find the heading delta towards the next point.
                heading_diff = pp_traj.getState(ind + 1)->pose.Rotation() - pp_state->pose.Rotation();
            }

            int curv_sign = 0;

            if (heading_diff.Radians() > 0_rad)
            {
                curv_sign = 1;
            }
            else if (heading_diff.Radians() < 0_rad)
            {
                curv_sign = -1;
            }

            path.push_back(frc::TrajectoryGenerator::PoseWithCurvature{pp_state->pose, pp_state->curvature * curv_sign});
        }

        frc::Trajectory final_trajectory;

        std::size_t segments = path.size() / 250;
        for (size_t segment = 0; segment < segments; segment++) {
            std::vector<frc::TrajectoryGenerator::PoseWithCurvature> current_path;
            current_path.reserve(251);

            bool invert = segment % 2 == 1;

            for (int i = 0; i < 251; i++) {
                // avoid off-by-one
                if (i == 250 && segment == segments - 1) {
                    continue;
                }
                current_path.push_back(path[i + segment * 250]);
            }

            auto current_traj = frc::TrajectoryParameterizer::TimeParameterizeTrajectory(current_path, config->Constraints(), config->StartVelocity(), config->EndVelocity(), config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed() ^ invert);

            std::cout << "time for segment " << segment << ": " << current_traj.TotalTime().value() << std::endl;

            final_trajectory = final_trajectory + current_traj;
        }

        std::cout << "total time: " << final_trajectory.TotalTime().value() << std::endl;

        return final_trajectory;
    }

    std::vector<frc::Trajectory> AutoHelper::LoadTrajectorySplit(std::string name, frc::TrajectoryConfig *config)
    {
        wpi::json pp_config = pathplanner::PathPlanner::loadConfig(name);

        // velocity, accel don't matter
        // but let's use the configured ones anyway
        pathplanner::PathPlannerTrajectory pp_traj = pathplanner::PathPlanner::loadPath(name, config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed());

        std::vector<frc::TrajectoryGenerator::PoseWithCurvature> path;
        path.reserve(pp_traj.numStates());

        for (int ind = 0; ind < pp_traj.numStates(); ind++)
        {
            auto pp_state = pp_traj.getState(ind);

            frc::Rotation2d heading_diff;
            if (ind == pp_traj.numStates() - 1)
            {
                // Last point is special, use the previous point instead
                heading_diff = pp_traj.getState(ind)->pose.Rotation() - pp_traj.getState(ind - 1)->pose.Rotation();
            }
            else
            {
                // Find the heading delta towards the next point.
                heading_diff = pp_traj.getState(ind + 1)->pose.Rotation() - pp_state->pose.Rotation();
            }

            int curv_sign = 0;

            if (heading_diff.Radians() > 0_rad)
            {
                curv_sign = 1;
            }
            else if (heading_diff.Radians() < 0_rad)
            {
                curv_sign = -1;
            }

            path.push_back(frc::TrajectoryGenerator::PoseWithCurvature{pp_state->pose, pp_state->curvature * curv_sign});
        }

        std::vector<frc::Trajectory> trajectories;
        trajectories.reserve(path.size() / 250);
        std::size_t current_segment = 0;
        bool invert = false;
        std::vector<frc::TrajectoryGenerator::PoseWithCurvature> current_path;
        current_path.reserve(251);
        
        // handle all but final segment
        for (wpi::json::reference waypoint : pp_config.at("waypoints"))
        {
            current_path.clear();
            bool reverse_here = waypoint.at("isReversal");
            invert = invert ^ reverse_here;

            for (std::size_t i = 250 * current_segment; i < 250 * (current_segment + 1) + 1; i++)
            {
                if (i >= path.size())
                {
                    break;
                }
                current_path.push_back(path[i]);
            }

            // final waypoint
            if (current_path.size() == 0)
            {
                break;
            }
            auto current_traj = frc::TrajectoryParameterizer::TimeParameterizeTrajectory(current_path, config->Constraints(), config->StartVelocity(), config->EndVelocity(), config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed() ^ invert);

            std::cout << "time for segment: " << current_traj.TotalTime().value() << std::endl;

            trajectories.push_back(current_traj);

            current_segment += 1;
        }

        return trajectories;
    }

}