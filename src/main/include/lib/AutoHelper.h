#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>

namespace rj {
class AutoHelper {
public: 
    static frc::Trajectory LoadTrajectory(std::string name, frc::TrajectoryConfig *config);
};
}