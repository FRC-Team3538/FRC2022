// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/wpi/DataLogReader.h"

#include "lib/wpi/DataLog.h"
#include "wpi/Endian.h"
#include "wpi/MathExtras.h"

using namespace wpi::log;

static bool ReadString(wpi::span<const uint8_t>* buf, std::string_view* str) {
  if (buf->size() < 4) {
    *str = {};
    return false;
  }
  uint32_t len = wpi::support::endian::read32le(buf->data());
  if (len > (buf->size() - 4)) {
    *str = {};
    return false;
  }
  *str = {reinterpret_cast<const char*>(buf->data() + 4), len};
  *buf = buf->subspan(len + 4);
  return true;
}

bool DataLogRecord::IsStart() const {
  return m_entry == 0 && m_data.size() >= 20 &&
         wpi::support::endian::read32le(m_data.data()) == impl::kControlStart;
}

bool DataLogRecord::IsFinish() const {
  return m_entry == 0 && m_data.size() == 8 &&
         wpi::support::endian::read32le(m_data.data()) == impl::kControlFinish;
}

bool DataLogRecord::IsSetMetadata() const {
  return m_entry == 0 && m_data.size() >= 12 &&
         wpi::support::endian::read32le(m_data.data()) ==
             impl::kControlSetMetadata;
}

bool DataLogRecord::GetStartData(StartRecordData* out) const {
  if (!IsStart()) {
    return false;
  }
  out->entry = wpi::support::endian::read32le(&m_data[4]);
  auto buf = m_data.subspan(8);
  if (!ReadString(&buf, &out->name)) {
    return false;
  }
  if (!ReadString(&buf, &out->type)) {
    return false;
  }
  if (!ReadString(&buf, &out->metadata)) {
    return false;
  }
  return true;
}

bool DataLogRecord::GetFinishEntry(int* out) const {
  if (!IsFinish()) {
    return false;
  }
  *out = wpi::support::endian::read32le(&m_data[4]);
  return true;
}

bool DataLogRecord::GetSetMetadataData(MetadataRecordData* out) const {
  if (!IsSetMetadata()) {
    return false;
  }
  out->entry = wpi::support::endian::read32le(&m_data[4]);
  auto buf = m_data.subspan(8);
  return ReadString(&buf, &out->metadata);
}

bool DataLogRecord::GetBoolean(bool* value) const {
  if (m_data.size() != 1) {
    return false;
  }
  *value = m_data[0] != 0;
  return true;
}

bool DataLogRecord::GetInteger(int64_t* value) const {
  if (m_data.size() != 8) {
    return false;
  }
  *value = wpi::support::endian::read64le(m_data.data());
  return true;
}

bool DataLogRecord::GetFloat(float* value) const {
  if (m_data.size() != 4) {
    return false;
  }
  *value = wpi::BitsToFloat(wpi::support::endian::read32le(m_data.data()));
  return true;
}

bool DataLogRecord::GetDouble(double* value) const {
  if (m_data.size() != 8) {
    return false;
  }
  *value = wpi::BitsToDouble(wpi::support::endian::read64le(m_data.data()));
  return true;
}

bool DataLogRecord::GetString(std::string_view* value) const {
  *value = {reinterpret_cast<const char*>(m_data.data()), m_data.size()};
  return true;
}

bool DataLogRecord::GetBooleanArray(std::vector<int>* arr) const {
  arr->clear();
  arr->reserve(m_data.size());
  for (auto v : m_data) {
    arr->push_back(v);
  }
  return true;
}

bool DataLogRecord::GetIntegerArray(std::vector<int64_t>* arr) const {
  arr->clear();
  if ((m_data.size() % 8) != 0) {
    return false;
  }
  arr->reserve(m_data.size() / 8);
  for (size_t pos = 0; pos < m_data.size(); pos += 8) {
    arr->push_back(wpi::support::endian::read64le(&m_data[pos]));
  }
  return true;
}

bool DataLogRecord::GetFloatArray(std::vector<float>* arr) const {
  arr->clear();
  if ((m_data.size() % 4) != 0) {
    return false;
  }
  arr->reserve(m_data.size() / 4);
  for (size_t pos = 0; pos < m_data.size(); pos += 4) {
    arr->push_back(
        wpi::BitsToFloat(wpi::support::endian::read32le(&m_data[pos])));
  }
  return true;
}

bool DataLogRecord::GetDoubleArray(std::vector<double>* arr) const {
  arr->clear();
  if ((m_data.size() % 8) != 0) {
    return false;
  }
  arr->reserve(m_data.size() / 8);
  for (size_t pos = 0; pos < m_data.size(); pos += 8) {
    arr->push_back(
        wpi::BitsToDouble(wpi::support::endian::read64le(&m_data[pos])));
  }
  return true;
}

bool DataLogRecord::GetStringArray(std::vector<std::string_view>* arr) const {
  arr->clear();
  if (m_data.size() < 4) {
    return false;
  }
  uint32_t size = wpi::support::endian::read32le(m_data.data());
  // sanity check size
  if (size > ((m_data.size() - 4) / 4)) {
    return false;
  }
  auto buf = m_data.subspan(4);
  arr->reserve(size);
  for (uint32_t i = 0; i < size; ++i) {
    std::string_view str;
    if (!ReadString(&buf, &str)) {
      arr->clear();
      return false;
    }
    arr->push_back(str);
  }
  // any left over?  treat as corrupt
  if (!buf.empty()) {
    arr->clear();
    return false;
  }
  return true;
}

DataLogReader::DataLogReader(std::unique_ptr<MemoryBuffer> buffer)
    : m_buf{std::move(buffer)} {}

bool DataLogReader::IsValid() const {
  if (!m_buf) {
    return false;
  }
  auto buf = m_buf->GetBuffer();
  return buf.size() >= 8 &&
         std::string_view{reinterpret_cast<const char*>(buf.data()), 6} ==
             "WPILOG" &&
         wpi::support::endian::read16le(&buf[6]) >= 0x0100;
}

uint16_t DataLogReader::GetVersion() const {
  if (!m_buf) {
    return 0;
  }
  auto buf = m_buf->GetBuffer();
  if (buf.size() < 8) {
    return 0;
  }
  return wpi::support::endian::read16le(&buf[6]);
}

bool DataLogReader::GetRecord(size_t* pos, DataLogRecord* out) const {
  if (!m_buf) {
    return false;
  }
  auto buf = m_buf->GetBuffer();
  if (*pos >= buf.size()) {
    return false;
  }
  buf = buf.subspan(*pos);
  if (buf.size() < 16) {
    return false;
  }
  int entry = wpi::support::endian::read32le(&buf[0]);
  uint32_t size = wpi::support::endian::read32le(&buf[4]);
  if (size > (buf.size() - 16)) {
    return false;
  }
  int64_t timestamp = wpi::support::endian::read64le(&buf[8]);
  *out = DataLogRecord{entry, timestamp, buf.subspan(16, size)};
  *pos += 16 + size;
  return true;
}

bool DataLogReader::GetNextRecord(size_t* pos) const {
  if (!m_buf) {
    return false;
  }
  auto buf = m_buf->GetBuffer();
  if (buf.size() < (*pos + 16)) {
    return false;
  }
  uint32_t size = wpi::support::endian::read32le(&buf[*pos + 4]);
  if (size >= (buf.size() - *pos - 16)) {  // check this way to avoid overflow
    return false;
  }
  *pos += 16 + size;
  return true;
}