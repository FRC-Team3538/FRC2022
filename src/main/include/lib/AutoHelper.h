#include <frc/trajectory/Trajectory.h>  // for Trajectory
#include <string>                       // for string
namespace frc { class TrajectoryConfig; }

namespace rj {
class AutoHelper {
public: 
    static frc::Trajectory LoadTrajectory(std::string name, frc::TrajectoryConfig *config);
};
}