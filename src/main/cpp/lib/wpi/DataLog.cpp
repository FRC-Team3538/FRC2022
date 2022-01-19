// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "lib/wpi/DataLog.h"

#include "wpi/Synchronization.h"

#ifndef _WIN32
#include <unistd.h>
#endif

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>  // NOLINT(build/include_order)

#endif

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "fmt/format.h"
#include "wpi/Endian.h"
#include "wpi/MathExtras.h"
#include "wpi/fs.h"
#include "wpi/timestamp.h"

using namespace wpi::log;

static constexpr size_t kBlockSize = 16 * 1024;
static constexpr size_t kRecordHeaderSize = 16;

static void WriteRecordHeader(uint8_t** buf, int entry, uint64_t timestamp,
                              size_t payloadSize) {
  wpi::support::endian::write32le(*buf, entry);
  *buf += 4;
  wpi::support::endian::write32le(*buf, payloadSize);
  *buf += 4;
  wpi::support::endian::write64le(*buf,
                                  timestamp == 0 ? wpi::Now() : timestamp);
  *buf += 8;
}

class DataLog::Buffer {
 public:
  explicit Buffer(size_t alloc = kBlockSize)
      : m_buf{new uint8_t[alloc]}, m_maxLen{alloc} {}
  ~Buffer() { delete[] m_buf; }

  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  Buffer(Buffer&& oth)
      : m_buf{oth.m_buf}, m_len{oth.m_len}, m_maxLen{oth.m_maxLen} {
    oth.m_buf = nullptr;
    oth.m_len = 0;
    oth.m_maxLen = 0;
  }

  Buffer& operator=(Buffer&& oth) {
    if (m_buf) {
      delete[] m_buf;
    }
    m_buf = oth.m_buf;
    m_len = oth.m_len;
    m_maxLen = oth.m_maxLen;
    oth.m_buf = nullptr;
    oth.m_len = 0;
    oth.m_maxLen = 0;
    return *this;
  }

  uint8_t* Reserve(size_t size) {
    assert(size <= GetRemaining());
    uint8_t* rv = m_buf + m_len;
    m_len += size;
    return rv;
  }

  void Clear() { m_len = 0; }

  size_t GetRemaining() const { return m_maxLen - m_len; }

  wpi::span<uint8_t> GetData() { return {m_buf, m_len}; }
  wpi::span<const uint8_t> GetData() const { return {m_buf, m_len}; }

 private:
  uint8_t* m_buf;
  size_t m_len = 0;
  size_t m_maxLen;
};

DataLog::DataLog(std::string_view dir, std::string_view filename, double period)
    : m_newFilename{filename}, m_thread{[this, dir = std::string{dir}, period] {
        WriterThreadMain(dir, period);
      }} {}

DataLog::~DataLog() {
  {
    std::scoped_lock lock{m_mutex};
    m_active = false;
    m_doFlush = true;
  }
  m_cond.notify_all();
  m_thread.join();
}

void DataLog::SetFilename(std::string_view filename) {
  {
    std::scoped_lock lock{m_mutex};
    m_newFilename = filename;
  }
  m_cond.notify_all();
}

void DataLog::Flush() {
  {
    std::scoped_lock lock{m_mutex};
    m_doFlush = true;
  }
  m_cond.notify_all();
}

void DataLog::Pause() {
  std::scoped_lock lock{m_mutex};
  m_paused = true;
}

void DataLog::Resume() {
  std::scoped_lock lock{m_mutex};
  m_paused = false;
}

static void WriteToFile(fs::file_t f, wpi::span<const uint8_t> data,
                        std::string_view filename) {
  do {
#ifdef _WIN32
    DWORD ret;
    if (!WriteFile(f, data.data(), data.size(), &ret, nullptr)) {
      fmt::print(stderr, "DataLog: Error writing to log file '{}': {}\n",
                 filename, GetLastError());
      break;
    }
#else
    ssize_t ret = ::write(f, data.data(), data.size());
    if (ret < 0) {
      // If it's a recoverable error, swallow it and retry the write
      if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
        continue;
      }

      // Otherwise it's a non-recoverable error; quit trying
      fmt::print(stderr, "DataLog: Error writing to log file '{}': {}\n",
                 filename, std::strerror(errno));
      break;
    }
#endif

    // The write may have written some or all of the data
    data = data.subspan(ret);
  } while (data.size() > 0);
}

void DataLog::WriterThreadMain(std::string_view dir, double period) {
  std::chrono::duration<double> periodTime{period};

  std::error_code ec;
  fs::path dirPath{dir};
  std::string filename;

  {
    std::scoped_lock lock{m_mutex};
    filename = std::move(m_newFilename);
    m_newFilename.clear();
  }

  if (filename.empty()) {
    // build random filename
    static std::random_device dev;
    static std::mt19937 rng(dev());
    std::uniform_int_distribution<int> dist(0, 15);
    const char* v = "0123456789abcdef";
    filename = "wpilog_";
    for (int i = 0; i < 16; i++) {
      filename += v[dist(rng)];
    }
    filename += ".wpilog";
  }

  // open file for append
  auto f = fs::OpenFileForWrite(dirPath / filename, ec, fs::CD_CreateAlways,
                                fs::OF_Append);
  if (ec) {
    fmt::print(stderr, "DataLog: Could not open log file '{}': {}\n", filename,
               ec.message());
  }

  // write header (version 1.0)
  if (f != fs::kInvalidFile) {
    const uint8_t header[] = {'W', 'P', 'I', 'L', 'O', 'G', 0, 1};
    WriteToFile(f, header, filename);
  }

  std::vector<Buffer> toWrite;

  std::unique_lock lock{m_mutex};
  while (m_active) {
    bool doFlush = false;
    auto timeoutTime = std::chrono::steady_clock::now() + periodTime;
    if (m_cond.wait_until(lock, timeoutTime) == std::cv_status::timeout) {
      doFlush = true;
    }

    if (!m_newFilename.empty()) {
      auto newFilename = std::move(m_newFilename);
      m_newFilename.clear();
      lock.unlock();
      // rename
      if (filename != newFilename) {
        fs::rename(dirPath / filename, dirPath / newFilename, ec);
      }
      if (ec) {
        fmt::print(stderr,
                   "DataLog: Could not rename log file from '{}' to '{}': {}\n",
                   filename, newFilename, ec.message());
      }
      filename = std::move(newFilename);
      lock.lock();
    }

    if (doFlush || m_doFlush) {
      // flush to file
      m_doFlush = false;
      if (m_outgoing.empty()) {
        continue;
      }
      // swap outgoing with empty vector
      toWrite.swap(m_outgoing);

      if (f != fs::kInvalidFile) {
        lock.unlock();
        // write buffers to file
        for (auto&& buf : toWrite) {
          WriteToFile(f, buf.GetData(), filename);
        }

        // sync to storage
#if defined(__linux__)
        ::fdatasync(f);
#elif defined(__APPLE__)
        ::fsync(f);
#endif
        lock.lock();
      }

      // release buffers back to free list
      for (auto&& buf : toWrite) {
        buf.Clear();
        m_free.emplace_back(std::move(buf));
      }
      toWrite.resize(0);
    }
  }

  if (f != fs::kInvalidFile) {
    fs::CloseFile(f);
  }
}

// Control records use the following format:
// 4-byte type
// 4-byte entry
// rest of data (depending on type)

int DataLog::Start(std::string_view name, std::string_view type,
                   std::string_view metadata, int64_t timestamp) {
  std::scoped_lock lock{m_mutex};
  auto& entryInfo = m_entries[name];
  if (entryInfo.id == 0) {
    entryInfo.id = ++m_lastId;
  }
  auto& savedCount = m_entryCounts[entryInfo.id];
  ++savedCount;
  if (savedCount > 1) {
    if (entryInfo.type != type) {
      fmt::print(stderr,
                 "DataLog: type mismatch for '{}': was '{}', requested '{}'; "
                 "ignoring\n",
                 name, entryInfo.type, type);
      return 0;
    }
    return entryInfo.id;
  }
  entryInfo.type = type;
  size_t strsize = name.size() + type.size() + metadata.size();
  uint8_t* buf = Reserve(kRecordHeaderSize + 8);
  WriteRecordHeader(&buf, 0, timestamp, 8 + 12 + strsize);
  wpi::support::endian::write32le(buf, impl::kControlStart);
  buf += 4;
  wpi::support::endian::write32le(buf, entryInfo.id);
  AppendStringImpl(name);
  AppendStringImpl(type);
  AppendStringImpl(metadata);

  return entryInfo.id;
}

void DataLog::Finish(int entry, int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  auto& savedCount = m_entryCounts[entry];
  if (savedCount == 0) {
    return;
  }
  --savedCount;
  if (savedCount != 0) {
    return;
  }
  m_entryCounts.erase(entry);
  uint8_t* buf = Reserve(kRecordHeaderSize + 8);
  WriteRecordHeader(&buf, 0, timestamp, 8);
  wpi::support::endian::write32le(buf, impl::kControlFinish);
  buf += 4;
  wpi::support::endian::write32le(buf, entry);
}

void DataLog::SetMetadata(int entry, std::string_view metadata,
                          int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  uint8_t* buf = Reserve(kRecordHeaderSize + 8);
  WriteRecordHeader(&buf, entry, timestamp, 8 + 4 + metadata.size());
  wpi::support::endian::write32le(buf, impl::kControlSetMetadata);
  buf += 4;
  wpi::support::endian::write32le(buf, entry);
  AppendStringImpl(metadata);
}

uint8_t* DataLog::Reserve(size_t size) {
  assert(size <= kBlockSize);
  if (m_outgoing.empty() || size > m_outgoing.back().GetRemaining()) {
    if (m_free.empty()) {
      m_outgoing.emplace_back();
    } else {
      m_outgoing.emplace_back(std::move(m_free.back()));
      m_free.pop_back();
    }
  }
  return m_outgoing.back().Reserve(size);
}

void DataLog::AppendImpl(wpi::span<const uint8_t> data) {
  while (data.size() > kBlockSize) {
    uint8_t* buf = Reserve(kBlockSize);
    std::memcpy(buf, data.data(), kBlockSize);
    data = data.subspan(kBlockSize);
  }
  uint8_t* buf = Reserve(data.size());
  std::memcpy(buf, data.data(), data.size());
}

void DataLog::AppendStringImpl(std::string_view str) {
  uint8_t* buf = Reserve(4);
  wpi::support::endian::write32le(buf, str.size());
  AppendImpl({reinterpret_cast<const uint8_t*>(str.data()), str.size()});
}

void DataLog::AppendRaw(int entry, wpi::span<const uint8_t> data,
                        int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize);
  WriteRecordHeader(&buf, entry, timestamp, data.size());
  AppendImpl(data);
}

void DataLog::AppendRaw2(int entry,
                         wpi::span<const wpi::span<const uint8_t>> data,
                         int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  size_t size = 0;
  for (auto&& chunk : data) {
    size += chunk.size();
  }
  uint8_t* buf = Reserve(kRecordHeaderSize);
  WriteRecordHeader(&buf, entry, timestamp, size);
  for (auto chunk : data) {
    AppendImpl(chunk);
  }
}

void DataLog::AppendBoolean(int entry, bool value, int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 1);
  WriteRecordHeader(&buf, entry, timestamp, 1);
  buf[0] = value ? 1 : 0;
}

void DataLog::AppendInteger(int entry, int64_t value, int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 8);
  WriteRecordHeader(&buf, entry, timestamp, 8);
  wpi::support::endian::write64le(buf, value);
}

void DataLog::AppendFloat(int entry, float value, int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 4);
  WriteRecordHeader(&buf, entry, timestamp, 4);
  if constexpr (wpi::support::endian::system_endianness() ==
                wpi::support::little) {
    std::memcpy(buf, &value, 4);
  } else {
    wpi::support::endian::write32le(buf, wpi::FloatToBits(value));
  }
}

void DataLog::AppendDouble(int entry, double value, int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 8);
  WriteRecordHeader(&buf, entry, timestamp, 8);
  if constexpr (wpi::support::endian::system_endianness() ==
                wpi::support::little) {
    std::memcpy(buf, &value, 8);
  } else {
    wpi::support::endian::write64le(buf, wpi::DoubleToBits(value));
  }
}

void DataLog::AppendString(int entry, std::string_view value,
                           int64_t timestamp) {
  AppendRaw(entry,
            {reinterpret_cast<const uint8_t*>(value.data()), value.size()},
            timestamp);
}

void DataLog::AppendBooleanArray(int entry, wpi::span<const bool> arr,
                                 int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize);
  WriteRecordHeader(&buf, entry, timestamp, arr.size());
  while (arr.size() > kBlockSize) {
    buf = Reserve(kBlockSize);
    for (auto val : arr.subspan(0, kBlockSize)) {
      *buf++ = val ? 1 : 0;
    }
    arr = arr.subspan(kBlockSize);
  }
  buf = Reserve(arr.size());
  for (auto val : arr) {
    *buf++ = val ? 1 : 0;
  }
}

void DataLog::AppendBooleanArray(int entry, wpi::span<const int> arr,
                                 int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize);
  WriteRecordHeader(&buf, entry, timestamp, arr.size());
  while (arr.size() > kBlockSize) {
    buf = Reserve(kBlockSize);
    for (auto val : arr.subspan(0, kBlockSize)) {
      *buf++ = val & 1;
    }
    arr = arr.subspan(kBlockSize);
  }
  buf = Reserve(arr.size());
  for (auto val : arr) {
    *buf++ = val & 1;
  }
}

void DataLog::AppendBooleanArray(int entry, wpi::span<const uint8_t> arr,
                                 int64_t timestamp) {
  AppendRaw(entry, arr, timestamp);
}

void DataLog::AppendIntegerArray(int entry, wpi::span<const int64_t> arr,
                                 int64_t timestamp) {
  if constexpr (wpi::support::endian::system_endianness() ==
                wpi::support::little) {
    AppendRaw(entry,
              {reinterpret_cast<const uint8_t*>(arr.data()), arr.size() * 8},
              timestamp);
  } else {
    if (entry <= 0) {
      return;
    }
    std::scoped_lock lock{m_mutex};
    if (m_paused) {
      return;
    }
    uint8_t* buf = Reserve(kRecordHeaderSize);
    WriteRecordHeader(&buf, entry, timestamp, arr.size() * 8);
    while ((arr.size() * 8) > kBlockSize) {
      buf = Reserve(kBlockSize);
      for (auto val : arr.subspan(0, kBlockSize / 8)) {
        wpi::support::endian::write64le(buf, val);
        buf += 8;
      }
      arr = arr.subspan(kBlockSize / 8);
    }
    buf = Reserve(arr.size() * 8);
    for (auto val : arr) {
      wpi::support::endian::write64le(buf, val);
      buf += 8;
    }
  }
}

void DataLog::AppendFloatArray(int entry, wpi::span<const float> arr,
                               int64_t timestamp) {
  if constexpr (wpi::support::endian::system_endianness() ==
                wpi::support::little) {
    AppendRaw(entry,
              {reinterpret_cast<const uint8_t*>(arr.data()), arr.size() * 4},
              timestamp);
  } else {
    if (entry <= 0) {
      return;
    }
    std::scoped_lock lock{m_mutex};
    if (m_paused) {
      return;
    }
    uint8_t* buf = Reserve(kRecordHeaderSize);
    WriteRecordHeader(&buf, entry, timestamp, arr.size() * 4);
    while ((arr.size() * 4) > kBlockSize) {
      buf = Reserve(kBlockSize);
      for (auto val : arr.subspan(0, kBlockSize / 4)) {
        wpi::support::endian::write32le(buf, wpi::FloatToBits(val));
        buf += 4;
      }
      arr = arr.subspan(kBlockSize / 4);
    }
    buf = Reserve(arr.size() * 4);
    for (auto val : arr) {
      wpi::support::endian::write32le(buf, wpi::FloatToBits(val));
      buf += 4;
    }
  }
}

void DataLog::AppendDoubleArray(int entry, wpi::span<const double> arr,
                                int64_t timestamp) {
  if constexpr (wpi::support::endian::system_endianness() ==
                wpi::support::little) {
    AppendRaw(entry,
              {reinterpret_cast<const uint8_t*>(arr.data()), arr.size() * 8},
              timestamp);
  } else {
    if (entry <= 0) {
      return;
    }
    std::scoped_lock lock{m_mutex};
    if (m_paused) {
      return;
    }
    uint8_t* buf = Reserve(kRecordHeaderSize);
    WriteRecordHeader(&buf, entry, timestamp, arr.size() * 8);
    while ((arr.size() * 8) > kBlockSize) {
      buf = Reserve(kBlockSize);
      for (auto val : arr.subspan(0, kBlockSize / 8)) {
        wpi::support::endian::write64le(buf, wpi::DoubleToBits(val));
        buf += 8;
      }
      arr = arr.subspan(kBlockSize / 8);
    }
    buf = Reserve(arr.size() * 8);
    for (auto val : arr) {
      wpi::support::endian::write64le(buf, wpi::DoubleToBits(val));
      buf += 8;
    }
  }
}

void DataLog::AppendStringArray(int entry, wpi::span<const std::string> arr,
                                int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  // storage: 4-byte array length, each string prefixed by 4-byte length
  // calculate total size
  size_t size = 4;
  for (auto&& str : arr) {
    size += 4 + str.size();
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 4);
  WriteRecordHeader(&buf, entry, timestamp, size);
  wpi::support::endian::write32le(buf, arr.size());
  for (auto&& str : arr) {
    AppendStringImpl(str);
  }
}

void DataLog::AppendStringArray(int entry,
                                wpi::span<const std::string_view> arr,
                                int64_t timestamp) {
  if (entry <= 0) {
    return;
  }
  // storage: 4-byte array length, each string prefixed by 4-byte length
  // calculate total size
  size_t size = 4;
  for (auto&& str : arr) {
    size += 4 + str.size();
  }
  std::scoped_lock lock{m_mutex};
  if (m_paused) {
    return;
  }
  uint8_t* buf = Reserve(kRecordHeaderSize + 4);
  WriteRecordHeader(&buf, entry, timestamp, size);
  wpi::support::endian::write32le(buf, arr.size());
  for (auto sv : arr) {
    AppendStringImpl(sv);
  }
}