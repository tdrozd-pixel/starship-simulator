#include "fsw/Ccsds.hpp"

#include <cmath>
#include <cstring>

namespace fsw::ccsds {

std::size_t writeU8(uint8_t* buf, uint8_t value) {
    buf[0] = value;
    return 1;
}

std::size_t writeU16(uint8_t* buf, uint16_t value) {
    buf[0] = static_cast<uint8_t>(value >> 8);
    buf[1] = static_cast<uint8_t>(value);
    return 2;
}

std::size_t writeU32(uint8_t* buf, uint32_t value) {
    for (int i = 0; i < 4; ++i) {
        buf[i] = static_cast<uint8_t>(value >> (24 - 8 * i));
    }
    return 4;
}

std::size_t writeF32(uint8_t* buf, float value) {
    uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    return writeU32(buf, bits);
}

std::size_t writeF64(uint8_t* buf, double value) {
    uint64_t bits = 0;
    std::memcpy(&bits, &value, sizeof bits);
    for (int i = 0; i < 8; ++i) {
        buf[i] = static_cast<uint8_t>(bits >> (56 - 8 * i));
    }
    return 8;
}

uint8_t readU8(const uint8_t* buf) { return buf[0]; }

uint16_t readU16(const uint8_t* buf) { return static_cast<uint16_t>((buf[0] << 8) | buf[1]); }

uint32_t readU32(const uint8_t* buf) {
    return (static_cast<uint32_t>(buf[0]) << 24) | (static_cast<uint32_t>(buf[1]) << 16) |
           (static_cast<uint32_t>(buf[2]) << 8) | static_cast<uint32_t>(buf[3]);
}

float readF32(const uint8_t* buf) {
    const uint32_t bits = readU32(buf);
    float value = 0.0F;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

double readF64(const uint8_t* buf) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; ++i) {
        bits = (bits << 8) | buf[i];
    }
    double value = 0.0;
    std::memcpy(&value, &bits, sizeof value);
    return value;
}

void writePrimaryHeader(uint8_t* buf, uint16_t apid, uint16_t seqCount, uint16_t dataLength) {
    // version 0, type 0 (TM), secondary header flag 1
    writeU16(buf, static_cast<uint16_t>(0x0800U | (apid & 0x07FFU)));
    writeU16(buf + 2, static_cast<uint16_t>(0xC000U | (seqCount & 0x3FFFU)));  // flags 0b11
    writeU16(buf + 4, static_cast<uint16_t>(dataLength - 1));
}

PrimaryHeader readPrimaryHeader(const uint8_t* buf) {
    const uint16_t w0 = readU16(buf);
    const uint16_t w1 = readU16(buf + 2);
    PrimaryHeader h;
    h.version = static_cast<uint8_t>(w0 >> 13);
    h.type = static_cast<uint8_t>((w0 >> 12) & 1U);
    h.secondaryHeader = ((w0 >> 11) & 1U) != 0;
    h.apid = static_cast<uint16_t>(w0 & 0x07FFU);
    h.sequenceFlags = static_cast<uint8_t>(w1 >> 14);
    h.sequenceCount = static_cast<uint16_t>(w1 & 0x3FFFU);
    h.dataLength = readU16(buf + 4);
    return h;
}

void writeCucTime(uint8_t* buf, double seconds) {
    if (!(seconds > 0.0)) {
        seconds = 0.0;
    }
    double whole = std::floor(seconds);
    uint32_t fine = static_cast<uint32_t>(std::lround((seconds - whole) * 65536.0));
    if (fine >= 65536U) {  // rounded up to the next second
        fine = 0;
        whole += 1.0;
    }
    writeU32(buf, static_cast<uint32_t>(whole));
    writeU16(buf + 4, static_cast<uint16_t>(fine));
}

double readCucTime(const uint8_t* buf) {
    return static_cast<double>(readU32(buf)) + static_cast<double>(readU16(buf + 4)) / 65536.0;
}

}  // namespace fsw::ccsds
