#ifndef BUFFER_H
#define BUFFER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <TMCStepper.h>

constexpr bool useMcp = true;
constexpr bool useMcpInMotorControl = true;
constexpr bool debugEnabled = false;

// Original buffer pins
constexpr uint8_t hall1Pin = PB2;
constexpr uint8_t hall2Pin = PB3;
constexpr uint8_t hall3Pin = PB4;

constexpr uint8_t endstop3Pin = PB7;

constexpr uint8_t keyReversePin = PB10;
constexpr uint8_t keyForwardPin = PB11;
constexpr uint8_t keyForward2Pin = PB12;
constexpr uint8_t keyReverse2Pin = PB13;

constexpr uint8_t ledReversePin = PA4;
constexpr uint8_t ledForwardPin = PA5;

constexpr uint8_t enPin = PA6;
constexpr uint8_t dirMotorPin = PA7;
constexpr uint8_t stepPin = PC13;
constexpr uint8_t uartPin = PB1;

// MCP wiring on your board
constexpr uint8_t mcpSclPin = PB14;
constexpr uint8_t mcpSdaPin = PB15;
constexpr uint8_t mcpIntPin = PA3;
constexpr uint8_t muxResetPin = PA2;

// TMC2209
constexpr uint8_t driverAddress = 0b00;
constexpr float rSense = 0.11f;

// Motion settings
constexpr uint32_t speedNormalRpm = static_cast<uint32_t>(33.0f / 0.22f);
constexpr uint32_t speedButtonRpm = static_cast<uint32_t>(66.0f / 0.22f);
constexpr int32_t moveDivideNum = 64;

constexpr uint32_t _rpmToVactual(uint32_t rpm) {
    return static_cast<uint32_t>(
        (static_cast<float>(rpm) * static_cast<float>(moveDivideNum) * 200.0f) /
        (60.0f * 0.715f)
    );
}

constexpr uint32_t vactualNormal = _rpmToVactual(speedNormalRpm);
constexpr uint32_t vactualButton = _rpmToVactual(speedButtonRpm);
constexpr uint32_t vactualMedium = _rpmToVactual((speedNormalRpm + speedButtonRpm) / 2U);

constexpr uint32_t stopValue = 0;

// Current settings
constexpr uint16_t currentAutoMa = 600;
constexpr uint16_t currentButtonMa = 900;

// Timeout / retry settings
constexpr uint32_t defaultTimeoutMs = 10000;
constexpr uint8_t maxAutoRetries = 6;
constexpr uint32_t autoRetryPauseMs = 400;

// Status LED heartbeat
constexpr uint32_t statusLedPeriodMs = 3000;
constexpr uint32_t statusLedOffPulseMs = 120;

// MCP23017 address
constexpr uint8_t mcpAddress = 0x20;

struct BufferState {
    bool pos1SensorState = false;
    bool pos2SensorState = false;
    bool pos3SensorState = false;
    bool materialSwitchState = false;

    bool localReversePressed = false;
    bool localForwardPressed = false;

    bool panelOverridePressed = false;
    bool panelCancelPressed = false;
    bool panelSpeedIncPressed = false;
    bool panelResetPressed = false;
    bool panelForwardPressed = false;
    bool panelReversePressed = false;
    bool panelSpeedDecPressed = false;
};

enum class MotorState : uint8_t {
    forward = 0,
    stop,
    back
};

void bufferSensorInit();
void bufferMotorInit();
void readSensorState();
void motorControl();

void bufferInit();
void bufferLoop();
void timerItCallback();
void bufferDebug();

extern bool isError;
extern uint32_t frontTime;
extern uint32_t timeoutMs;
extern bool isFront;
extern TMC2209Stepper driver;

#endif