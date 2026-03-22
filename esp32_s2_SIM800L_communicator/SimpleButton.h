#pragma once
#include <Arduino.h>

class SimpleButton {
public:
    typedef void (*ClickHandler)();

    SimpleButton() {}

    void begin(uint8_t pin) {
        _pin = pin;
        pinMode(_pin, INPUT_PULLUP);
    }

    void setClickHandler(ClickHandler handler) {
        _handler = handler;
    }

    void loop() {
        bool pressed = digitalRead(_pin) == LOW;

        unsigned long now = millis();

        if (pressed && !_lastPressed) {
            _pressStart = now;
        }

        if (!pressed && _lastPressed) {
            if (now - _pressStart >= 10) {
                if (_handler) {
                    _handler();
                }
            }
        }

        _lastPressed = pressed;
    }

private:
    uint8_t _pin = 0;
    bool _lastPressed = false;
    unsigned long _pressStart = 0;
    ClickHandler _handler = nullptr;
};
