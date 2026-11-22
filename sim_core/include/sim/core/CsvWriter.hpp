#pragma once

#include <cstdio>
#include <initializer_list>
#include <string>
#include <vector>

namespace sim {

// Buffered CSV writer for telemetry logs. Rows are formatted into an in-memory
// buffer and flushed in large chunks, so logging at 100 Hz costs almost nothing.
// TODO(phase 0): move file I/O to a background thread for high-rate logging.
class CsvWriter {
public:
    CsvWriter() = default;
    ~CsvWriter();

    CsvWriter(const CsvWriter&) = delete;
    CsvWriter& operator=(const CsvWriter&) = delete;

    bool open(const std::string& path, const std::vector<std::string>& columns);
    bool isOpen() const { return file_ != nullptr; }
    void row(std::initializer_list<double> values);
    void flush();
    void close();

private:
    std::FILE* file_ = nullptr;
    std::size_t columns_ = 0;
    std::string buffer_;
    static constexpr std::size_t kFlushBytes = 1 << 20;
};

}  // namespace sim
