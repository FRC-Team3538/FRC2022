#include "lib/wpi/DataLogManager.h"

#include <ntcore.h>
#include <unordered_map>

#include "lib/wpi/DataLog.h"

namespace rj {

class DataLogUtils
{
private:
  bool NTEntryLogging;
  bool NTConnectionLogging;
  bool JoystickLogging;

  NT_EntryListener entryListener;
  NT_ConnectionListener connectionListener;

  std::unordered_map<NT_Entry, int> NTEntryToDataLogEntryHandleMap;
  wpi::log::DataLog& dataLog = frc::DataLogManager::GetLog();

public:
  DataLogUtils(){};

  void EnableNTEntryLogging();
  void DisableNTEntryLogging();
  bool isNTEntryLogging();

  void EnableNTConnectionLogging();
  void DisableNTConnectionLogging();
  bool isNTConnectionLogging();

  void EnableJoystickLogging();
  void DisableJoystickLogging();
  bool isJoystickLogging();
};
}