// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "networktables/NetworkTableEntry.inc"               // for NetworkT...
#include "units/angle.h"                                     // for degree_t
#include "units/angular_velocity.h"                          // for radians_...
#include "units/curvature.h"                                 // for curvature_t
#include "units/length.h"                                    // for meter_t
#include "units/math.h"                                      // for abs
#include "units/velocity.h"                                  // for meters_p...
#include "units/voltage.h"            
#include "subsystems/Drivetrain.hpp"

void Drivetrain::Drive(double fwd, double rot) {
  m_leftGroup.Set(fwd - rot);
  m_rightGroup.Set(fwd + rot);
}

//void Drivetrain::UpdateOdometry() {
//  m_odometry.Update(m_gyro.GetRotation2d(),
//                    units::meter_t(m_leftEncoder.GetDistance()),
//                    units::meter_t(m_rightEncoder.GetDistance()));
//}
