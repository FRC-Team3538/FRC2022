#include "lib/wpi/DataLogManager.h"

#include <ntcore.h>
#include <unordered_map>

#include "lib/wpi/DataLog.h"
#include <frc/DriverStation.h>
#include <hal/DriverStation.h>
#include <atomic>

#include "lib/Logging.h"
#ifdef LOGGER

namespace rj {

// Do not use directly
class JoystickLogSender {
 public:
  void Init(wpi::log::DataLog& log, unsigned int stick, int64_t timestamp);
  void Send(uint64_t timestamp);

 private:
  void AppendButtons(HAL_JoystickButtons buttons, uint64_t timestamp);
  void AppendPOVs(const HAL_JoystickPOVs& povs, uint64_t timestamp);

  unsigned int m_stick;
  HAL_JoystickButtons m_prevButtons;
  HAL_JoystickAxes m_prevAxes;
  HAL_JoystickPOVs m_prevPOVs;
  wpi::log::BooleanArrayLogEntry m_logButtons;
  wpi::log::FloatArrayLogEntry m_logAxes;
  wpi::log::IntegerArrayLogEntry m_logPOVs;
};

// Do not use directly
class DSDataLogSender {
 public:
  void Init(wpi::log::DataLog& log, bool logJoysticks, int64_t timestamp);
  void Send(uint64_t timestamp);

 private:
  std::atomic_bool m_initialized{false};

  HAL_ControlWord m_prevControlWord;
  wpi::log::BooleanLogEntry m_logEnabled;
  wpi::log::BooleanLogEntry m_logAutonomous;
  wpi::log::BooleanLogEntry m_logTest;
  wpi::log::BooleanLogEntry m_logEstop;

  bool m_logJoysticks;
  std::array<JoystickLogSender, frc::DriverStation::kJoystickPorts> m_joysticks;
};

class DataLogUtils
{
private:
  bool NTEntryLogging;
  bool NTConnectionLogging;
  bool JoystickLogging;
  bool DSLogging;

  NT_EntryListener entryListener;
  NT_ConnectionListener connectionListener;

  std::unordered_map<NT_Entry, int> NTEntryToDataLogEntryHandleMap;
  wpi::log::DataLog& dataLog = frc::DataLogManager::GetLog();

  DSDataLogSender DS_Sender;
public:
  DataLogUtils(){};

  void EnableNTEntryLogging();
  void DisableNTEntryLogging();
  bool isNTEntryLogging();

  void EnableNTConnectionLogging();
  void DisableNTConnectionLogging();
  bool isNTConnectionLogging();

  void InitDSLogging(bool logJoystick);
  void LogDSState();
  bool isDSLogging();
};
}

#endif // LOGGER