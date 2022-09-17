#include <auto/AutoInterface.h>
#include <Robotmap.h>
#include <frc/Timer.h>

class DriveForwardAuto: public AutoInterface {
public:
    DriveForwardAuto(Robotmap &);
    DriveForwardAuto() = delete;
    ~DriveForwardAuto();

    void Init() override;
    void Run() override;
    void UpdateSmartDash() override;

private:
    Robotmap &IO;

    units::meters_per_second_t speed = 0_mps;
    units::degree_t heading = 0_deg;
    units::second_t duration = 0_s;

    frc::Timer timer;
public:
    static std::string GetName();
};