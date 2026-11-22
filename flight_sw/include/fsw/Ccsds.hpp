#pragma once

// CCSDS Space Packet helpers (CCSDS 133.0-B-2), big-endian. See docs/telemetry/gnc_packets.md.

#include <cstddef>
#include <cstdint>

namespace fsw::ccsds {

constexpr std::size_t kPrimaryHeaderSize = 6;    // [bytes]
constexpr std::size_t kSecondaryHeaderSize = 6;  // [bytes] CUC time code
constexpr std::size_t kHeaderSize = kPrimaryHeaderSize + kSecondaryHeaderSize;
constexpr uint16_t kMaxSequenceCount = 16383;

// Writers return the number of bytes written.
std::size_t writeU8(uint8_t* buf, uint8_t value);
std::size_t writeU16(uint8_t* buf, uint16_t value);
std::size_t writeU32(uint8_t* buf, uint32_t value);
std::size_t writeF32(uint8_t* buf, float value);
std::size_t writeF64(uint8_t* buf, double value);

uint8_t readU8(const uint8_t* buf);
uint16_t readU16(const uint8_t* buf);
uint32_t readU32(const uint8_t* buf);
float readF32(const uint8_t* buf);
double readF64(const uint8_t* buf);

// `dataLength` is the packet data field length in bytes (secondary header + user data);
// the header carries dataLength - 1.
void writePrimaryHeader(uint8_t* buf, uint16_t apid, uint16_t seqCount, uint16_t dataLength);

// CUC time: u32 whole seconds then u16 fraction in 1/65536 s. Negative times clamp to 0.
void writeCucTime(uint8_t* buf, double seconds);
double readCucTime(const uint8_t* buf);  // [s]

struct PrimaryHeader {
    uint8_t version = 0;
    uint8_t type = 0;
    bool secondaryHeader = false;
    uint16_t apid = 0;
    uint8_t sequenceFlags = 0;
    uint16_t sequenceCount = 0;
    uint16_t dataLength = 0;  // as carried (bytes - 1)
};
PrimaryHeader readPrimaryHeader(const uint8_t* buf);

}  // namespace fsw::ccsds
