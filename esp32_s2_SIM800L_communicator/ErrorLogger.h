#pragma once


class ErrorLogger {
public:

    // --- Enum of all possible errors ---
    enum ErrorCode {
        ERR_NONE = 0,             // no error
        ERR_SEND_AT_FAIL,      // 1
        ERR_SEND_NO_SIM,       // 2
        ERR_SEND_CSQ_FAIL,     // 3     
        ERR_SEND_REG_FAIL,     // 4 
        ERR_SEND_CIMI_FAIL,    // 5 
        ERR_SEND_GPRS_FAIL,    // 6 
        ERR_SEND_HTTP_FAIL,    // 7 
        ERR_SEND_REPEAT,       // 8 

        ERR_DIR_READ,              // 9         
        ERR_DIR_READ_ONCE,         // 10 
        ERR_WIND_BUF_OVERWRITE,    // 11    
        ERR_WIND_SHORT_BUF_FULL,   // 12         
        ERR_SPEED_SHORT_BUF_FULL,  // 13         
        ERR_DIR_SHORT_BUF_FULL,    // 14     

        // --- Power and reset related errors ---
        ERR_POWERON_RESET,        // 15 normal power-on reset (info)
        ERR_BROWNOUT_RESET,       // 16 brown-out or low-voltage reset
        ERR_PANIC_RESET,          // 17 software panic or abort
        ERR_WDT_RESET,            // watchdog reset (task/int/other)
        ERR_SDIO_RESET,           // reset triggered over SDIO
        ERR_USB_RESET,            // reset by USB peripheral
        ERR_JTAG_RESET,           // reset by JTAG
        ERR_EFUSE_RESET,          // reset due to efuse error
        ERR_PWR_GLITCH_RESET,     // reset due to power glitch detection
        ERR_CPU_LOCKUP_RESET,     // reset due to CPU lockup (double exception)
        ERR_UNEXPECTED_RESET,     // reset reason unexpected / unclassified
        
        // Marker for the total number of errors
        ERR_COUNT_MAX
    };

    // Constructor: Initializes the error array to zero.
    ErrorLogger() {
        // Explicitly initialize the array to all zeros
        for (int i = 0; i < ERR_COUNT_MAX; ++i) {
            _errorCounts[i] = 0;
        }
    }

    // NOTE: begin() and end() are no longer necessary without Preferences.

    // --- Log (increment) an error count ---
    void log(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            _errorCounts[code]++;
        }
    }

    // --- Get number of times an error occurred ---
    int32_t get(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            return _errorCounts[code];
        }
        return -1; // Return 0 for an invalid code
    }

    // --- Print or return all stored error counts ---
    String getAll(bool printToSerial = false) {
        String out;
        for (int code = ERR_NONE; code < ERR_COUNT_MAX; ++code) {
            uint32_t count = _errorCounts[code];
            if (count > 0) {
                String msg = String(code) + ": " +
                             errorToString((ErrorCode)code) +
                             " -> count=" + String(count) + "\r\n";
                out += msg;
            }
        }
        return out;
    }

    String getAllForSend(bool printToSerial = false) {
        String out;
        bool first = true;
        for (int code = 1; code < ERR_COUNT_MAX; ++code) {
            uint32_t count = _errorCounts[code];
            if (count > 0) {
                if(!first) out += ",";
                first = false;
                String msg = String(code) + ":" + String(count);
                out += msg;
            }
        }
        return out;
    }

    // --- Clear a specific error counter ---
    void clear(ErrorCode code) {
        if (code >= 0 && code < ERR_COUNT_MAX) {
            _errorCounts[code] = 0;
        }
    }

    // --- Clear all error counters ---
    void clearAll() {
        // Simply call the constructor's initialization logic
        for (int i = 0; i < ERR_COUNT_MAX; ++i) {
            _errorCounts[i] = 0;
        }
    }

    static const char* errorToString(ErrorCode code) {
        switch (code) {
            // ... (Your errorToString implementation remains the same) ...
            case ERR_NONE:            return "No error";
            case ERR_DIR_READ:        return "Direction read";
            case ERR_SEND_AT_FAIL:    return "AT command failure";
            case ERR_SEND_NO_SIM:     return "No SIM card detected";
            case ERR_SEND_CSQ_FAIL:   return "Signal quality (CSQ) check failed";
            case ERR_SEND_REG_FAIL:   return "Network registration failed";
            case ERR_SEND_CIMI_FAIL:  return "IMSI (CIMI) retrieval failed";
            case ERR_SEND_GPRS_FAIL:  return "GPRS/Data connection failed";
            case ERR_SEND_HTTP_FAIL:  return "HTTP communication failed";
            case ERR_SEND_REPEAT:     return "Send had to be repeated";

            case ERR_POWERON_RESET:   return "Power-on reset (info)";
            case ERR_BROWNOUT_RESET:  return "Brown-out/low-voltage reset";
            case ERR_PANIC_RESET:     return "Software panic/abort reset";
            case ERR_WDT_RESET:       return "Watchdog Timer reset";
            case ERR_SDIO_RESET:      return "SDIO-triggered reset";
            case ERR_USB_RESET:       return "USB-triggered reset";
            case ERR_JTAG_RESET:      return "JTAG-triggered reset";
            case ERR_EFUSE_RESET:     return "EFUSE error reset";
            case ERR_PWR_GLITCH_RESET: return "Power glitch reset";
            case ERR_CPU_LOCKUP_RESET: return "CPU lockup/double exception reset";
            case ERR_UNEXPECTED_RESET: return "Unexpected/unclassified reset";
            case ERR_DIR_READ_ONCE:    return "Read dir error had to be repeated.";

            default:                  return "Unknown ErrorCode";
        }
    }

private:
    // New class member: Static array to hold the counts.
    // The size is determined by the final element in the enum.
    static constexpr int ERROR_COUNT = ERR_COUNT_MAX; // Alternatively, use ERR_COUNT_MAX
    uint16_t _errorCounts[ERROR_COUNT];
};


/*


class ErrorLogger {
public:
    // --- Enum of all possible errors ---
  enum ErrorCode {
      ERR_NONE = 0,             // no error
      ERR_DIR_READ,                   
      ERR_SEND_AT_FAIL,
      ERR_SEND_NO_SIM,
      ERR_SEND_CSQ_FAIL,
      ERR_SEND_REG_FAIL,
      ERR_SEND_CIMI_FAIL,
      ERR_SEND_GPRS_FAIL,
      ERR_SEND_HTTP_FAIL,

      // --- Power and reset related errors ---
      ERR_POWERON_RESET,        // normal power-on reset (info)
      ERR_BROWNOUT_RESET,       // brown-out or low-voltage reset
      ERR_PANIC_RESET,          // software panic or abort
      ERR_WDT_RESET,            // watchdog reset (task/int/other)
      ERR_SDIO_RESET,           // reset triggered over SDIO
      ERR_USB_RESET,            // reset by USB peripheral
      ERR_JTAG_RESET,           // reset by JTAG
      ERR_EFUSE_RESET,          // reset due to efuse error
      ERR_PWR_GLITCH_RESET,     // reset due to power glitch detection
      ERR_CPU_LOCKUP_RESET,     // reset due to CPU lockup (double exception)
      ERR_UNEXPECTED_RESET      // reset reason unexpected / unclassified
    };

    ErrorLogger(const char* ns = "errors")
        : _ns(ns) {}

    void begin() {
        prefs.begin(_ns.c_str(), false);
    }

    void end() {
        prefs.end();
    }

    // --- Log (increment) an error count ---
    void log(ErrorCode code) {
        String key = makeKey(code);
        uint32_t count = prefs.getUInt(key.c_str(), 0);
        prefs.putUInt(key.c_str(), count + 1);
    }

    // --- Get number of times an error occurred ---
    uint32_t get(ErrorCode code) {
        return prefs.getUInt(makeKey(code).c_str(), 0);
    }

    // --- Print or return all stored error counts ---
    String getAll(bool printToSerial = false) {
        String out;
        for (int code = ERR_NONE; code <= ERR_UNEXPECTED_RESET; ++code) {
            uint32_t count = get((ErrorCode)code);
            if (count > 0) {
                String msg = String(code) + ": " +
                             errorToString((ErrorCode)code) +
                             " → count=" + String(count) + "\n";
                out += msg;
                if (printToSerial) Serial.print(msg);
            }
        }
        return out;
    }

    // --- Clear a specific error counter ---
    void clear(ErrorCode code) {
        prefs.remove(makeKey(code).c_str());
    }

    // --- Clear all error counters ---
    void clearAll() {
        prefs.clear();
    }

    static const char* errorToString(ErrorCode code) {
      switch (code) {
        // --- General Errors ---
        case ERR_NONE:            return "No error";
        case ERR_DIR_READ:        return "Directory read failed";
        case ERR_SEND_AT_FAIL:    return "AT command failure";
        case ERR_SEND_NO_SIM:     return "No SIM card detected";
        case ERR_SEND_CSQ_FAIL:   return "Signal quality (CSQ) check failed";
        case ERR_SEND_REG_FAIL:   return "Network registration failed";
        case ERR_SEND_CIMI_FAIL:  return "IMSI (CIMI) retrieval failed";
        case ERR_SEND_GPRS_FAIL:  return "GPRS/Data connection failed";
        case ERR_SEND_HTTP_FAIL:  return "HTTP communication failed";

        // --- Power and Reset Errors ---
        case ERR_POWERON_RESET:   return "Power-on reset (info)";
        case ERR_BROWNOUT_RESET:  return "Brown-out/low-voltage reset";
        case ERR_PANIC_RESET:     return "Software panic/abort reset";
        case ERR_WDT_RESET:       return "Watchdog Timer reset";
        case ERR_SDIO_RESET:      return "SDIO-triggered reset";
        case ERR_USB_RESET:       return "USB-triggered reset";
        case ERR_JTAG_RESET:      return "JTAG-triggered reset";
        case ERR_EFUSE_RESET:     return "EFUSE error reset";
        case ERR_PWR_GLITCH_RESET: return "Power glitch reset";
        case ERR_CPU_LOCKUP_RESET: return "CPU lockup/double exception reset";
        case ERR_UNEXPECTED_RESET: return "Unexpected/unclassified reset";

        // --- Default Case ---
        default:                  return "Unknown ErrorCode";
      }
    }

private:
    Preferences prefs;
    String _ns;

    String makeKey(ErrorCode code) {
        return String((int)code); // Use enum number as key
    }
};
*/