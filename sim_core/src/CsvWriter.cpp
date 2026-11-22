#include "sim/core/CsvWriter.hpp"

#include <stdexcept>

namespace sim {

CsvWriter::~CsvWriter() { close(); }

bool CsvWriter::open(const std::string& path, const std::vector<std::string>& columns) {
    close();
#ifdef _MSC_VER
    if (fopen_s(&file_, path.c_str(), "wb") != 0) {
        file_ = nullptr;
    }
#else
    file_ = std::fopen(path.c_str(), "wb");
#endif
    if (file_ == nullptr) {
        return false;
    }
    columns_ = columns.size();
    buffer_.reserve(kFlushBytes + 4096);
    for (std::size_t i = 0; i < columns.size(); ++i) {
        buffer_ += columns[i];
        buffer_ += (i + 1 < columns.size()) ? ',' : '\n';
    }
    return true;
}

void CsvWriter::row(std::initializer_list<double> values) {
    if (file_ == nullptr) {
        return;
    }
    if (values.size() != columns_) {
        throw std::logic_error("CsvWriter::row column count does not match the header");
    }
    char cell[32];
    std::size_t i = 0;
    for (double v : values) {
        const int n = std::snprintf(cell, sizeof(cell), "%.10g", v);
        buffer_.append(cell, static_cast<std::size_t>(n));
        buffer_ += (++i < columns_) ? ',' : '\n';
    }
    if (buffer_.size() >= kFlushBytes) {
        flush();
    }
}

void CsvWriter::flush() {
    if (file_ != nullptr && !buffer_.empty()) {
        std::fwrite(buffer_.data(), 1, buffer_.size(), file_);
        buffer_.clear();
    }
}

void CsvWriter::close() {
    if (file_ != nullptr) {
        flush();
        std::fclose(file_);
        file_ = nullptr;
    }
}

}  // namespace sim
