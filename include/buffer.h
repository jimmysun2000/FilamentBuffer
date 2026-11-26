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
// A continuous automatic feed is stopped and restarted every timeoutMs (stall recovery)
constexpr uint32_t defaultTimeoutMs = 10000;
constexpr uint32_t autoRetryPauseMs = 400;

// Runaway feed guard (e.g. bowden tube popped off, so the buffer never fills).
// Net automatic forward feed time is budgeted. A brief stop only refunds it
// slowly, so a slider that flickers out of the feed position cannot reset it;
// a stop of feedFillDwellMs or more is taken as a real fill and clears it.
// Until the buffer has filled once the loading budget applies, which leaves
// room for a long tube. After that refills are short, so the primed budget applies.
// The loading budget is also how far automatic reverse may pull back.
constexpr uint32_t loadFeedBudgetMs = 90000;
constexpr uint32_t primedFeedBudgetMs = 10000;
constexpr uint32_t primedFeedKickMs = 4000;   // restart interval while primed
constexpr uint32_t feedPrimeDwellMs = 1000;   // time out of the feed position before the primed budget applies
constexpr uint32_t feedFillDwellMs = 300;     // time out of the feed position that counts as a real fill
constexpr int32_t feedIdleRefundRate = 2;     // budget ms refunded per ms spent stopped

// Panel latched motion stops by itself after this long without a new press
constexpr uint32_t panelLatchMaxMs = 90000;

// A held local button stops after this long and must be released to work again
constexpr uint32_t localButtonMaxMs = 90000;

// Input filtering: a new state must hold this long before it is acted on
constexpr uint32_t inputDebounceMs = 50;   // runout switch and local buttons
constexpr uint32_t panelDebounceMs = 30;   // panel buttons

// Panel housekeeping
constexpr uint32_t panelSetupRefreshMs = 2000;  // re-send the panel setup, or look for a missing panel
constexpr uint8_t panelMaxReadFailures = 10;    // failed reads in a row before the panel counts as lost

// Driver settings are re-sent this often while the motor is stopped
constexpr uint32_t driverRefreshIntervalMs = 10000;

// Hardware watchdog: resets the MCU if the main loop stops running
constexpr uint32_t watchdogTimeoutUs = 8000000;

// Accepted range for the stored feed timeout
constexpr uint32_t minTimeoutMs = 1000;
constexpr uint32_t maxTimeoutMs = 120000;

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
extern volatile uint32_t frontTime;
extern uint32_t timeoutMs;
extern volatile bool isFront;
extern TMC2209Stepper driver;

#endif