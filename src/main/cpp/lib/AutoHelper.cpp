#include "lib/AutoHelper.h"
#include <frc/trajectory/TrajectoryGenerator.h>      // for TrajectoryGenera...
#include <frc/trajectory/TrajectoryParameterizer.h>  // for TrajectoryParame...
#include <lib/pathplanner/PathPlanner.h>             // for PathPlanner
#include <cstddef>                                   // for size_t
#include <iostream>                                  // for operator<<, endl
#include <vector>                                    // for vector
#include "frc/geometry/Pose2d.h"                     // for Pose2d
#include "frc/geometry/Rotation2d.h"                 // for Rotation2d
#include "frc/trajectory/Trajectory.h"               // for Trajectory
#include "frc/trajectory/TrajectoryConfig.h"         // for TrajectoryConfig
#include "lib/pathplanner/PathPlannerTrajectory.h"   // for PathPlannerTraje...
#include "units/angle.h"                             // for operator""_rad
#include "units/base.h"                              // for unit_t, operator*
#include "units/curvature.h"                         // for curvature_t
#include "units/time.h"                              // for second_t

namespace rj
{

    frc::Trajectory AutoHelper::LoadTrajectory(std::string name, frc::TrajectoryConfig *config)
    {
        // velocity, accel don't matter
        // but let's use the configured ones anyway
        pathplanner::PathPlannerTrajectory pp_traj = pathplanner::PathPlanner::loadPath(name, config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed());

        std::vector<frc::TrajectoryGenerator::PoseWithCurvature> path;

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
            current_path.reserve(250);

            bool invert = segment % 2 == 1;

            for (int i = 0; i < 250; i++) {
                current_path.push_back(path[i + segment * 250]);
            }

            auto current_traj = frc::TrajectoryParameterizer::TimeParameterizeTrajectory(current_path, config->Constraints(), config->StartVelocity(), config->EndVelocity(), config->MaxVelocity(), config->MaxAcceleration(), config->IsReversed() ^ invert);

            std::cout << "time for segment " << segment << ": " << current_traj.TotalTime().value() << std::endl;

            final_trajectory = final_trajectory + current_traj;
        }

        std::cout << "total time: " << final_trajectory.TotalTime().value() << std::endl;
        // for (units::second_t time = 0_s; time < final_trajectory.TotalTime(); time = time + 0.02_s) {
        //     auto reference = final_trajectory.Sample(time);
        //     std::cout << reference.t.value() << ", " << reference.pose.Translation().X().value() << ", " << reference.pose.Translation().Y().value() << ", " << reference.pose.Rotation().Radians().value() << ", " << reference.curvature.value() << ", " << reference.velocity.value() << ", " << reference.acceleration.value() << std::endl;
        // }

        return final_trajectory;
    }

}