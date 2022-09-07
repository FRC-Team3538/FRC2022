#include <wpi/array.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/velocity.h>
#include <units/math.h>

template <size_t NumModules>
class AlternativeSwerveKinematics {
public:
    AlternativeSwerveKinematics(
        units::meters_per_second_t maxTranslationalVelocity,
        units::radians_per_second_t maxRotationalVelocity,
        units::meters_per_second_t maxModuleVelocity):
        maxTranslationalVelocity(maxTranslationalVelocity),
        maxRotationalVelocity(maxRotationalVelocity),
        maxModuleVelocity(maxModuleVelocity)
    {}
        

    /*
    public void normalizeDrive(SwerveModuleState[] desiredStates, ChassisSpeeds speeds) {
        double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / DriveConstants.kMaxTranslationalVelocity;
        double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / DriveConstants.kMaxRotationalVelocity;
        double k = Math.max(translationalK, rotationalK);

        // Find the how fast the fastest spinning drive motor is spinning                                       
        double realMaxSpeed = 0.0;
        for (SwerveModuleState moduleState : desiredStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }

        double scale = Math.min(k * ModuleConstants.kMaxSpeedMetersPerSecond / realMaxSpeed, 1);
        for (SwerveModuleState moduleState : desiredStates) {
            moduleState.speedMetersPerSecond *= scale;
        }
    }
    */

    void NormalizeWheelSpeeds(
        wpi::array<frc::SwerveModuleState, NumModules>* moduleStates,
        frc::ChassisSpeeds speeds)
    {
        auto translationalK = units::math::hypot(speeds.vx, speeds.vy) / maxTranslationalVelocity;
        auto rotationalK = units::math::abs(speeds.omega) / maxRotationalVelocity;
        auto k = units::math::max(translationalK, rotationalK);

        auto realMaxSpeed = std::max_element(moduleStates.begin(), moduleStates.end(),
                                        [](const auto& a, const auto& b) {
                                            return units::math::abs(a.speed) <
                                                units::math::abs(b.speed);
                                        })
                            ->speed;
        
        auto scale = units::math::min(k * maxModuleVelocity / realMaxSpeed, 1);

        for (auto moduleState : moduleStates) {
            moduleState->speed *= scale;
        }
    }
private:
    units::meters_per_second_t maxTranslationalVelocity;
    units::radians_per_second_t maxRotationalVelocity;
    units::meters_per_second_t maxModuleVelocity;

};
