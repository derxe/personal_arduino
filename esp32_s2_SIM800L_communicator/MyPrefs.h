// PrefBlob.h
#pragma once
#include <Arduino.h>
#include <Preferences.h>


template<typename T>
class PrefBlob {
  static_assert(std::is_trivially_copyable<T>::value,
                "T must be trivially copyable (POD-like).");

public:
  // ctor: optional version tag you bump when the layout changes
  explicit PrefBlob(uint16_t version = 1) : _version(version) {}

  // Open NVS namespace (rw by default). Note: NVS key length <= 15 chars.
  bool begin() {
    bool readOnly = false;
    _begun = _prefs.begin(_nvsNamespace, readOnly);
    return _begun;
  }

  void end() {
    if (_begun) { _prefs.end(); _begun = false; }
  }

  int load(T& out) {
    if (!_begun) return -1;

    size_t len = _prefs.getBytesLength(_key);
    if (len < sizeof(Header) + sizeof(T)) return -2;

    std::unique_ptr<uint8_t[]> buf(new (std::nothrow) uint8_t[len]);
    if (!buf) return -3;

    size_t got = _prefs.getBytes(_key, buf.get(), len);
    if (got != len) return -4;

    const Header* hdr = reinterpret_cast<const Header*>(buf.get());
    if (hdr->magic != MAGIC) return -5;
    if (hdr->size  != sizeof(T)) return -6;         // struct size changed
    if (hdr->version != _version) return -7;         // stored newer than firmware

    const uint8_t* payload = buf.get() + sizeof(Header);
    uint32_t crc = crc32(payload, sizeof(T));
    if (crc != hdr->crc32) return -8;

    // Valid -> copy payload into caller-provided out
    memcpy(&out, payload, sizeof(T));
    return 1;
  }

  int save(const T& in) {
    if (!_begun) return -1;

    Packed p;
    p.h.magic   = MAGIC;
    p.h.version = _version;
    p.h.size    = sizeof(T);
    memcpy(&p.payload, &in, sizeof(T));
    p.h.crc32   = crc32(reinterpret_cast<const uint8_t*>(&p.payload), sizeof(T));

    size_t wrote = _prefs.putBytes(_key, &p, sizeof(Packed));
    if (wrote != sizeof(Packed)) return -2;

    return 1;
  }

  // Remove key from NVS
  bool erase() {
    if (!_begun) return false;
    bool ok = _prefs.remove(_key);
    return ok;
  }


private:
  struct Header {
    uint32_t magic;   // 'PBLB'
    uint16_t version; // you bump this in firmware when you redefine T
    uint16_t size;    // sizeof(T)
    uint32_t crc32;   // CR_namespaceC of payload (T bytes)
  };

  static constexpr uint32_t MAGIC = 0x50424C42; // 'PBLB'

  struct Packed {
    Header h;
    T      payload;
  };

  static uint32_t crc32(const uint8_t* data, size_t len) {
    // Small, tableless CRC32 (IEEE 802.3)
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b) {
        uint32_t mask = -(crc & 1u);
        crc = (crc >> 1) ^ (0xEDB88320u & mask);
      }
    }
    return ~crc;
  }

  Preferences _prefs;
  const char* _nvsNamespace = "app";
  const char* _key = "prefs_blob";
  uint16_t _version;
  bool _begun = false;

  T  _loadedCopy{};
};

