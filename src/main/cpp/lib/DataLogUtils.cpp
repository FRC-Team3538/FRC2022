#include "lib/DataLogUtils.h"

#include "fmt/format.h"
#include <networktables/NetworkTableInstance.h>
#include <wpi/json_serializer.h>
#include <wpi/raw_ostream.h>
#include <hal/DriverStationTypes.h>
#include <wpi/timestamp.h>

static std::string
ConnInfoToJson(bool connected, const nt::ConnectionInfo& info)
{
  std::string str;
  wpi::raw_string_ostream os{ str };
  wpi::json::serializer s{ os, ' ', 0 };
  os << "{\"connected\":" << (connected ? "true" : "false");
  os << ",\"remote_id\":\"";
  s.dump_escaped(info.remote_id, false);
  os << "\",\"remote_ip\":\"";
  s.dump_escaped(info.remote_ip, false);
  os << "\",\"remote_port\":";
  s.dump_integer(static_cast<uint64_t>(info.remote_port));
  os << ",\"protocol_version\":";
  s.dump_integer(static_cast<uint64_t>(info.protocol_version));
  os << "}";
  os.flush();
  return str;
}

static std::string_view
GetStorageTypeStr(NT_Type type)
{
  switch (type) {
    case NT_BOOLEAN:
      return wpi::log::BooleanLogEntry::kDataType;
    case NT_DOUBLE:
      return wpi::log::DoubleLogEntry::kDataType;
    case NT_STRING:
      return wpi::log::StringLogEntry::kDataType;
    case NT_RAW:
      return wpi::log::RawLogEntry::kDataType;
    case NT_BOOLEAN_ARRAY:
      return wpi::log::BooleanArrayLogEntry::kDataType;
    case NT_DOUBLE_ARRAY:
      return wpi::log::DoubleArrayLogEntry::kDataType;
    case NT_STRING_ARRAY:
      return wpi::log::StringArrayLogEntry::kDataType;
    default:
      return {};
  }
}

namespace rj {
void
DataLogUtils::EnableNTEntryLogging()
{
  if (!NTEntryLogging) {
    entryListener = nt::NetworkTableInstance::GetDefault().AddEntryListener(
      "",
      [this](const nt::EntryNotification& event) {
        if ((event.flags & NT_NOTIFY_NEW) != 0) {
          // add entry
          auto datatype = event.value->type();
          auto storageType = GetStorageTypeStr(datatype);
          if (storageType.empty()) {
            return;
          }

          auto id = this->dataLog.Start(fmt::format("NT:{}", event.name),
                                        storageType,
                                        "{\"source\":\"NT\"}",
                                        event.value->time());
          NTEntryToDataLogEntryHandleMap[event.entry] = id;
        }

        if ((event.flags & (NT_NOTIFY_NEW | NT_NOTIFY_UPDATE)) != 0) {
          auto id = NTEntryToDataLogEntryHandleMap[event.entry];
          switch (event.value->type()) {
            case NT_BOOLEAN:
              this->dataLog.AppendBoolean(
                id, event.value->GetBoolean(), event.value->time());
              break;
            case NT_DOUBLE:
              this->dataLog.AppendDouble(
                id, event.value->GetDouble(), event.value->time());
              break;
            case NT_STRING:
              this->dataLog.AppendString(
                id, event.value->GetString(), event.value->time());
              break;
            case NT_RAW: {
              auto val = event.value->GetRaw();
              this->dataLog.AppendRaw(
                id,
                { reinterpret_cast<const uint8_t*>(val.data()), val.size() },
                event.value->time());
              break;
            }
            case NT_BOOLEAN_ARRAY:
              this->dataLog.AppendBooleanArray(
                id, event.value->GetBooleanArray(), event.value->time());
              break;
            case NT_DOUBLE_ARRAY:
              this->dataLog.AppendDoubleArray(
                id, event.value->GetDoubleArray(), event.value->time());
              break;
            case NT_STRING_ARRAY:
              this->dataLog.AppendStringArray(
                id, event.value->GetStringArray(), event.value->time());
              break;
            default:
              break;
          }
        }

        if ((event.flags & NT_NOTIFY_DELETE) != 0) {
          auto id = NTEntryToDataLogEntryHandleMap.extract(event.entry);
          this->dataLog.Finish(id.mapped());
        }
      },
      NT_NOTIFY_IMMEDIATE | NT_NOTIFY_LOCAL | NT_NOTIFY_NEW | NT_NOTIFY_DELETE |
        NT_NOTIFY_UPDATE);

    NTEntryLogging = true;
  }
}

void
DataLogUtils::DisableNTEntryLogging()
{
  if (NTEntryLogging) {
    nt::NetworkTableInstance::GetDefault().RemoveEntryListener(entryListener);

    NTEntryLogging = false;
  }
}

bool
DataLogUtils::isNTEntryLogging()
{
  return NTEntryLogging;
}

void
DataLogUtils::EnableNTConnectionLogging()
{
  if (!NTConnectionLogging) {
    wpi::log::StringLogEntry connectionEntry = wpi::log::StringLogEntry(
      dataLog,
      "NTConnection",
      "{\"schema\":\"NTConnectionInfo\",\"source\":\"NT\"}",
      "json");

    connectionListener =
      nt::NetworkTableInstance::GetDefault().AddConnectionListener(
        [&connectionEntry](const nt::ConnectionNotification& event) {
          connectionEntry.Append(ConnInfoToJson(event.connected, event.conn));
        },
        true);

    NTConnectionLogging = true;
  }
}

void
DataLogUtils::DisableNTConnectionLogging()
{
  if (NTConnectionLogging) {
    nt::NetworkTableInstance::GetDefault().RemoveConnectionListener(
      connectionListener);

    NTConnectionLogging = false;
  }
}

bool
DataLogUtils::isNTConnectionLogging()
{
  return NTConnectionLogging;
}



void DataLogUtils::InitDSLogging(bool logJoystick) {
  if (!DSLogging) {
    DSLogging = true;
    DS_Sender.Init(dataLog, true, wpi::Now());
  }
}

void DataLogUtils::LogDSState() {
  if (DSLogging) {
    DS_Sender.Send(wpi::Now());
  }
}


bool 
DataLogUtils::isDSLogging() 
{ 
  return DSLogging; 
}

void JoystickLogSender::Init(wpi::log::DataLog& log, unsigned int stick,
                             int64_t timestamp) {
  m_stick = stick;

  m_logButtons = wpi::log::BooleanArrayLogEntry{
      log, fmt::format("DS:joystick{}/buttons", stick), timestamp};
  m_logAxes = wpi::log::FloatArrayLogEntry{
      log, fmt::format("DS:joystick{}/axes", stick), timestamp};
  m_logPOVs = wpi::log::IntegerArrayLogEntry{
      log, fmt::format("DS:joystick{}/povs", stick), timestamp};

  HAL_GetJoystickButtons(m_stick, &m_prevButtons);
  HAL_GetJoystickAxes(m_stick, &m_prevAxes);
  HAL_GetJoystickPOVs(m_stick, &m_prevPOVs);
  AppendButtons(m_prevButtons, timestamp);
  m_logAxes.Append(
      wpi::span<const float>{m_prevAxes.axes,
                             static_cast<size_t>(m_prevAxes.count)},
      timestamp);
  AppendPOVs(m_prevPOVs, timestamp);
}

void JoystickLogSender::Send(uint64_t timestamp) {
  HAL_JoystickButtons buttons;
  HAL_GetJoystickButtons(m_stick, &buttons);
  if (buttons.count != m_prevButtons.count ||
      buttons.buttons != m_prevButtons.buttons) {
    AppendButtons(buttons, timestamp);
  }
  m_prevButtons = buttons;

  HAL_JoystickAxes axes;
  HAL_GetJoystickAxes(m_stick, &axes);
  if (axes.count != m_prevAxes.count ||
      std::memcmp(axes.axes, m_prevAxes.axes,
                  sizeof(axes.axes[0]) * axes.count) != 0) {
    m_logAxes.Append(
        wpi::span<const float>{axes.axes, static_cast<size_t>(axes.count)},
        timestamp);
  }
  m_prevAxes = axes;

  HAL_JoystickPOVs povs;
  HAL_GetJoystickPOVs(m_stick, &povs);
  if (povs.count != m_prevPOVs.count ||
      std::memcmp(povs.povs, m_prevPOVs.povs,
                  sizeof(povs.povs[0]) * povs.count) != 0) {
    AppendPOVs(povs, timestamp);
  }
  m_prevPOVs = povs;
}

void JoystickLogSender::AppendButtons(HAL_JoystickButtons buttons,
                                      uint64_t timestamp) {
  uint8_t buttonsArr[32];
  for (unsigned int i = 0; i < buttons.count; ++i) {
    buttonsArr[i] = (buttons.buttons & (1u << i)) != 0;
  }
  m_logButtons.Append(wpi::span<const uint8_t>{buttonsArr, buttons.count},
                      timestamp);
}

void JoystickLogSender::AppendPOVs(const HAL_JoystickPOVs& povs,
                                   uint64_t timestamp) {
  int64_t povsArr[HAL_kMaxJoystickPOVs];
  for (int i = 0; i < povs.count; ++i) {
    povsArr[i] = povs.povs[i];
  }
  m_logPOVs.Append(
      wpi::span<const int64_t>{povsArr, static_cast<size_t>(povs.count)},
      timestamp);
}

void DSDataLogSender::Init(wpi::log::DataLog& log, bool logJoysticks,
                         int64_t timestamp) {
  m_logEnabled = wpi::log::BooleanLogEntry{log, "DS:enabled", timestamp};
  m_logAutonomous = wpi::log::BooleanLogEntry{log, "DS:autonomous", timestamp};
  m_logTest = wpi::log::BooleanLogEntry{log, "DS:test", timestamp};
  m_logEstop = wpi::log::BooleanLogEntry{log, "DS:estop", timestamp};

  // append initial control word values
  HAL_GetControlWord(&m_prevControlWord);
  m_logEnabled.Append(m_prevControlWord.enabled, timestamp);
  m_logAutonomous.Append(m_prevControlWord.autonomous, timestamp);
  m_logTest.Append(m_prevControlWord.test, timestamp);
  m_logEstop.Append(m_prevControlWord.eStop, timestamp);

  m_logJoysticks = logJoysticks;
  if (logJoysticks) {
    unsigned int i = 0;
    for (auto&& joystick : m_joysticks) {
      joystick.Init(log, i++, timestamp);
    }
  }

  m_initialized = true;
}

void DSDataLogSender::Send(uint64_t timestamp) {
  if (!m_initialized) {
    return;
  }

  // append control word value changes
  HAL_ControlWord ctlWord;
  HAL_GetControlWord(&ctlWord);
  if (ctlWord.enabled != m_prevControlWord.enabled) {
    m_logEnabled.Append(ctlWord.enabled, timestamp);
  }
  if (ctlWord.autonomous != m_prevControlWord.autonomous) {
    m_logAutonomous.Append(ctlWord.autonomous, timestamp);
  }
  if (ctlWord.test != m_prevControlWord.test) {
    m_logTest.Append(ctlWord.test, timestamp);
  }
  if (ctlWord.eStop != m_prevControlWord.eStop) {
    m_logEstop.Append(ctlWord.eStop, timestamp);
  }
  m_prevControlWord = ctlWord;

  if (m_logJoysticks) {
    // append joystick value changes
    for (auto&& joystick : m_joysticks) {
      joystick.Send(timestamp);
    }
  }
}

}