#pragma once

#include <frc/geometry/Pose2d.h>         // for Pose2d
#include <frc/geometry/Rotation2d.h>     // for Rotation2d
#include <frc/geometry/Translation2d.h>  // for Translation2d
#include <frc/trajectory/Trajectory.h>   // for Trajectory
#include <units/acceleration.h>          // for meters_per_second_squared_t
#include <units/angular_acceleration.h>  // for radians_per_second_squared_t
#include <units/angular_velocity.h>      // for radians_per_second_t
#include <units/curvature.h>             // for curvature_t
#include <units/length.h>                // for operator""_m, meter_t
#include <units/time.h>                  // for second_t, operator""_s
#include <units/velocity.h>              // for meters_per_second_t, operato...
#include <vector>                        // for vector

namespace pathplanner{
    class PathPlannerTrajectory{
        public:
            class PathPlannerState{
                public:
                    units::second_t time = 0_s;
                    units::meter_t position = 0_m;
                    units::meters_per_second_t velocity = 0_mps;
                    units::meters_per_second_squared_t acceleration = 0_mps_sq;
                    frc::Pose2d pose;
                    units::curvature_t curvature{0.0};
                    units::radians_per_second_t angularVel;
                    units::radians_per_second_squared_t angularAccel;
                    frc::Rotation2d holonomicRotation;
                    PathPlannerState interpolate(PathPlannerState endVal, double t);
                    units::meter_t curveRadius = 0_m;
                    units::meter_t deltaPos = 0_m;
            };

            class Waypoint{
                public:
                    frc::Translation2d anchorPoint;
                    frc::Translation2d prevControl;
                    frc::Translation2d nextControl;
                    units::meters_per_second_t velocityOverride;
                    frc::Rotation2d holonomicRotation;
                    bool isReversal;

                    Waypoint(frc::Translation2d anchorPoint, frc::Translation2d prevControl, frc::Translation2d nextControl, units::meters_per_second_t velocityOverride, frc::Rotation2d holonomicRotation, bool isReversal){
                        this->anchorPoint = anchorPoint;
                        this->prevControl = prevControl;
                        this->nextControl = nextControl;
                        this->velocityOverride = velocityOverride;
                        this->holonomicRotation = holonomicRotation;
                        this->isReversal = isReversal;
                    }
            };
        
        private:
            std::vector<PathPlannerState> states;
            std::vector<PathPlannerState> joinSplines(std::vector<Waypoint> pathPoints, units::meters_per_second_t maxVel, double step);
            void calculateMaxVel(std::vector<PathPlannerState> *states, units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel, bool reversed);
            void calculateVelocity(std::vector<PathPlannerState> *states, std::vector<Waypoint> pathPoints, units::meters_per_second_squared_t maxAccel);
            void recalculateValues(std::vector<PathPlannerState> *states, bool reversed);
            units::meter_t calculateRadius(PathPlannerState s0, PathPlannerState s1, PathPlannerState s2);
        
        public:
            PathPlannerTrajectory(std::vector<Waypoint> waypoints, units::meters_per_second_t maxVelocity, units::meters_per_second_squared_t maxAcceleration, bool reversed);
            PathPlannerTrajectory(std::vector<PathPlannerState> states);
            PathPlannerTrajectory();

            PathPlannerState sample(units::second_t time);
            std::vector<PathPlannerState> *getStates() { return &this->states; }
            int numStates() { return getStates()->size(); }
            PathPlannerState *getState(int i) { return &getStates()->data()[i]; }
            PathPlannerState *getInitialState() { return getState(0); }
            PathPlannerState *getEndState() { return getState(numStates() - 1); }
            units::second_t getTotalTime() { return getEndState()->time; }
            frc::Trajectory asWPILibTrajectory();
    };
}