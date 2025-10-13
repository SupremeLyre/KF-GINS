#include "crc32.h"
extern uint32_t crc32(const void *buf, std::size_t size, uint32_t crc) {
    const uint8_t *p = (const uint8_t *) buf;
    crc              = crc ^ 0xFFFFFFFF;
    while (size--) {
        crc = crc32_table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFF;
}