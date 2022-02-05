#include "lib/DataLogUtils.h"

#include "fmt/format.h"
#include <networktables/NetworkTableInstance.h>
#include <wpi/json_serializer.h>
#include <wpi/raw_ostream.h>

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

}