#include <frc/trajectory/Trajectory.h>  // for Trajectory
#include <string>                       // for string
#include <vector>
namespace frc { class TrajectoryConfig; }

namespace rj {
class AutoHelper {
public: 
    static frc::Trajectory LoadTrajectory(std::string name, frc::TrajectoryConfig *config);
    static std::vector<frc::Trajectory> LoadTrajectorySplit(std::string name, frc::TrajectoryConfig *config);
};
}