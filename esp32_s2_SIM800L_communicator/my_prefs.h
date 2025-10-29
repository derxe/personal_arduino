#ifndef MY_PREFERENCES_H
#define MY_PREFERENCES_H

#include <Preferences.h>
#include <Arduino.h>

/**
 * @brief Defines the structure for all persistent settings.
 * * NOTE: The ESP32 Preferences library uses key-value pairs, so we 
 * will use this struct as a guide, but save each member separately 
 * for simplicity and robustness.
 */
struct AppSettings {
    // --- Preferences Keys ---
    // A unique identifier to check if settings have been saved before.
    // Changing this number forces the defaults to be loaded on next boot.
    const uint32_t MAGIC_NUMBER = 0xAA55AA55; 
    
    // --- Actual Settings ---
    char deviceName[16] = "ESP-Device"; // Name for Wi-Fi or identification
    int maxMotorSpeed = 1024;           // Max speed value (e.g., for PWM/DAC)
    float temperatureOffset = 1.5f;     // Calibration offset
    bool enableAutoMode = true;         // Flag for automatic operation
};

/**
 * @brief Manages reading and writing application settings to ESP32 NVS.
 */
class PreferenceManager {
public:
    // Singleton pattern to ensure only one instance
    static PreferenceManager& getInstance() {
        static PreferenceManager instance; // Guaranteed to be destroyed and instantiated once
        return instance;
    }

    // Settings structure instance
    AppSettings settings;

    /**
     * @brief Loads settings from NVS, or loads defaults if NVS is empty/invalid.
     */
    void loadPreferences() {
        // Open the preferences namespace
        if (!preferences.begin("app-settings", false)) {
            Serial.println("FATAL: Failed to open Preferences namespace.");
            // Keep going, but settings will be defaults
            return;
        }

        // 1. Check if the magic number exists
        uint32_t magic = preferences.getUInt("magic", 0);

        if (magic == settings.MAGIC_NUMBER) {
            // Settings exist and are valid. Load them.
            Serial.println("Preferences found. Loading stored values...");
            
            // Load primitive types
            settings.maxMotorSpeed = preferences.getInt("maxSpeed", settings.maxMotorSpeed);
            settings.temperatureOffset = preferences.getFloat("tempOffset", settings.temperatureOffset);
            settings.enableAutoMode = preferences.getBool("autoMode", settings.enableAutoMode);

            // Load string/char array
            preferences.getString("devName", settings.deviceName, sizeof(settings.deviceName));
            
        } else {
            // Settings are invalid or missing. Save the current defaults.
            Serial.println("No valid preferences found. Saving defaults...");
            savePreferences();
        }

        // Close the preferences
        preferences.end();
    }

    /**
     * @brief Saves the current settings to NVS.
     */
    void savePreferences() {
        if (!preferences.begin("app-settings", false)) {
            Serial.println("FATAL: Failed to open Preferences namespace for saving.");
            return;
        }

        // 1. Save the magic number first
        preferences.putUInt("magic", settings.MAGIC_NUMBER);
        
        // 2. Save all settings
        preferences.putInt("maxSpeed", settings.maxMotorSpeed);
        preferences.putFloat("tempOffset", settings.temperatureOffset);
        preferences.putBool("autoMode", settings.enableAutoMode);
        preferences.putString("devName", settings.deviceName);

        // Commit and close
        preferences.end();
        Serial.println("Preferences saved successfully.");
    }

private:
    // Make constructor private to enforce Singleton pattern
    PreferenceManager() {} 
    
    // Disallow copy and assignment
    PreferenceManager(const PreferenceManager&) = delete;
    PreferenceManager& operator=(const PreferenceManager&) = delete;

    Preferences preferences;
};

#endif // MY_PREFERENCES_H