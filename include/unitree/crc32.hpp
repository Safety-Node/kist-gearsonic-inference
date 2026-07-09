#pragma once

#include <cstdint>

namespace kist {

// Unitree SDK message checksum (gear_sonic utils.hpp Crc32Core).
// Used on both directions of the DDS link: appended to outgoing LowCmd
// (firmware rejects commands without it) and validated on incoming LowState.
inline uint32_t crc32_core(const uint32_t* ptr, uint32_t len) {
    uint32_t crc = 0xFFFFFFFF;
    const uint32_t poly = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        uint32_t xbit = 1u << 31;
        uint32_t data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (crc & 0x80000000) {
                crc <<= 1;
                crc ^= poly;
            } else {
                crc <<= 1;
            }
            if (data & xbit) crc ^= poly;
            xbit >>= 1;
        }
    }
    return crc;
}

} // namespace kist
