#pragma once
#include <Arduino.h>
#include "SoftwareWiremy.h"

class AT24C32_EEPROM {
public:
  // 24C32 specifics
  static const uint16_t kSizeBytes = 4096;   // total capacity
  static const uint8_t  kPageSize  = 32;     // page write size

  explicit AT24C32_EEPROM(SoftwareWire &wire, uint8_t i2c_addr = 0x57)
  : _w(wire), _addr(i2c_addr) {}

  // Optional no-op; kept for symmetry with other libs
  bool begin() { return true; }

  // ---- High-level byte ops ----
  bool writeByte(uint16_t mem, uint8_t v) {
    return write(mem, &v, 1);
  }
  bool readByte(uint16_t mem, uint8_t &v) {
    return read(mem, &v, 1);
  }

  // ---- 32-bit helpers (little-endian) ----
  bool writeUint32(uint16_t mem, uint32_t v) {
    uint8_t b[4];
    b[0] = (uint8_t)(v & 0xFF);
    b[1] = (uint8_t)((v >> 8) & 0xFF);
    b[2] = (uint8_t)((v >> 16) & 0xFF);
    b[3] = (uint8_t)((v >> 24) & 0xFF);
    return write(mem, b, 4);
  }
  bool readUint32(uint16_t mem, uint32_t &v) {
    uint8_t b[4];
    if (!read(mem, b, 4)) return false;
    v = (uint32_t)b[0] |
        ((uint32_t)b[1] << 8) |
        ((uint32_t)b[2] << 16) |
        ((uint32_t)b[3] << 24);
    return true;
  }
  bool writeInt32(uint16_t mem, int32_t v) {
    return writeUint32(mem, (uint32_t)v);
  }
  bool readInt32(uint16_t mem, int32_t &v) {
    uint32_t tmp;
    if (!readUint32(mem, tmp)) return false;
    v = (int32_t)tmp;
    return true;
  }

  // ---- Buffer ops ----
  bool write(uint16_t mem, const void *data, size_t len) {
    if (!data) return false;
    if (mem >= kSizeBytes) return false;
    if (len == 0) return true;
    if ((uint32_t)mem + (uint32_t)len > kSizeBytes) return false;

    const uint8_t *p = (const uint8_t *)data;
    while (len > 0) {
      uint8_t pageSpace = (uint8_t)(kPageSize - (mem % kPageSize));
      uint8_t chunk = (len < pageSpace) ? (uint8_t)len : pageSpace;

      _w.beginTransmission(_addr);
      _w.write((uint8_t)(mem >> 8));        // high addr byte
      _w.write((uint8_t)(mem & 0xFF));      // low  addr byte
      for (uint8_t i = 0; i < chunk; i++) {
        _w.write((uint8_t)p[i]);            // avoid char* overload
      }
      if (_w.endTransmission(true) != 0) return false;

      if (!waitWriteComplete()) return false;

      mem += chunk;
      p   += chunk;
      len -= chunk;
    }
    return true;
  }

  bool read(uint16_t mem, void *data, size_t len) {
    if (!data) return false;
    if (mem >= kSizeBytes) return false;
    if (len == 0) return true;
    if ((uint32_t)mem + (uint32_t)len > kSizeBytes) return false;

    uint8_t *p = (uint8_t *)data;

    // Set memory address pointer with repeated start
    _w.beginTransmission(_addr);
    _w.write((uint8_t)(mem >> 8));
    _w.write((uint8_t)(mem & 0xFF));
    if (_w.endTransmission(false) != 0) return false;

    // Read in small chunks (I2C buffer friendly)
    size_t remain = len;
    while (remain > 0) {
      uint8_t want = (remain > 32) ? 32 : (uint8_t)remain; // 32 is safe
      uint8_t got = _w.requestFrom(_addr, want);
      if (got == 0) return false;
      for (uint8_t i = 0; i < got; i++) {
        if (!_w.available()) return false;
        *p++ = (uint8_t)_w.read();
      }
      remain -= got;
    }
    return true;
  }

private:
  SoftwareWire &_w;
  uint8_t _addr;

  bool waitWriteComplete(uint16_t timeout_ms = 20) {
    // Standard ACK polling loop after a page write
    uint32_t start = millis();
    for (;;) {
      _w.beginTransmission(_addr);
      int rc = _w.endTransmission(true);
      if (rc == 0) return true; // device responded -> write done
      if ((uint16_t)(millis() - start) >= timeout_ms) return false;
      // tiny pause to avoid hammering the bus
      delay(1);
    }
  }
};