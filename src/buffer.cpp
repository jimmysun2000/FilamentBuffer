#include "buffer.h"

#include <IWatchdog.h>

TMC2209Stepper driver(uartPin, uartPin, rSense, driverAddress);

bool isError = false;
volatile uint32_t frontTime = 0;
uint32_t timeoutMs = defaultTimeoutMs;
volatile bool isFront = false;

static BufferState bufferState = {};
static String serialBuffer;
static HardwareTimer timer6(TIM6);

static uint16_t currentCached = currentAutoMa;
static MotorState motorState = MotorState::stop;

// Auto retry state
static volatile bool autoRetryRequested = false;
static bool autoRetryPauseActive = false;
static uint32_t autoRetryPauseStartMs = 0;
static uint8_t autoRetryCount = 0;

// Auto refresh state
static bool forceAutoRefresh = false;

// Runaway feed guard state
enum class FeedAccounting : uint8_t {
    hold = 0,
    feeding,
    retracting,
    idle
};

static FeedAccounting feedAccounting = FeedAccounting::hold;
static uint32_t feedAccountLastMs = 0;
static int32_t feedNetMs = 0;
static uint32_t feedDwellMs = 0;
static bool feedSeenSinceReset = false;
static volatile bool feedPrimed = false;

// Debounced inputs
struct DebouncedInput {
    bool stable = false;
    bool candidate = false;
    uint32_t candidateSinceMs = 0;
};

static DebouncedInput runoutInput;
static DebouncedInput localReverseInput;
static DebouncedInput localForwardInput;

// Local button motion state. The lockout starts set so a button that is
// already held (or stuck) at power-up does nothing until it is released.
static MotorState localManualState = MotorState::stop;
static uint32_t localManualStartMs = 0;
static bool localManualLockout = true;

// Driver refresh state
static uint32_t driverRefreshLastMs = 0;

// Error state as last saved to EEPROM
static bool errorLatchStored = false;

// Serial command state
static uint32_t serialLastByteMs = 0;

// MCP state
static bool mcpPresent = false;
static uint16_t mcpOlatShadow = 0xFFFF;
static uint8_t mcpGpioACache = 0x7F;
static uint8_t previousMcpGpioA = 0x7F;
static uint8_t mcpCandidateGpioA = 0x7F;
static uint32_t mcpCandidateSinceMs = 0;
static uint8_t mcpReadFailures = 0;
static uint32_t mcpHousekeepingLastMs = 0;

// Panel motion state
static bool mcpOverrideLatched = false;
static uint8_t mcpManualSpeedIndex = 2;
static MotorState panelLatchedMotorState = MotorState::stop;
static uint32_t panelLatchStartMs = 0;

// EEPROM
static const int eepromAddrTimeout = 0;
static const int eepromAddrErrorLatch = 4;
constexpr uint8_t errorLatchValue = 0xE5;

// Serial commands
constexpr uint32_t serialIdleGapMs = 50;
constexpr uint8_t serialMaxCommandLength = 24;

// MCP23017 registers
constexpr uint8_t mcpIodirA = 0x00;
constexpr uint8_t mcpIodirB = 0x01;
constexpr uint8_t mcpGppuA = 0x0C;
constexpr uint8_t mcpGppuB = 0x0D;
constexpr uint8_t mcpGpioA = 0x12;
constexpr uint8_t mcpOlatA = 0x14;

// MCP pin mapping from your panel schematic
constexpr uint8_t swOverridePin = 0;      // GPA0
constexpr uint8_t swCancelPin = 1;        // GPA1
constexpr uint8_t swSpeedIncPin = 2;      // GPA2
constexpr uint8_t swResetPin = 3;         // GPA3
constexpr uint8_t swForwardPin = 4;       // GPA4
constexpr uint8_t swReversePin = 5;       // GPA5
constexpr uint8_t swSpeedDecPin = 6;      // GPA6
constexpr uint8_t ledOverridePanelPin = 7;     // GPA7

constexpr uint8_t ledSpeedLowPanelPin = 8;     // GPB0
constexpr uint8_t ledFilamentPanelPin = 9;     // GPB1
constexpr uint8_t ledStatusPanelPin = 10;      // GPB2
constexpr uint8_t ledErrorPanelPin = 11;       // GPB3
constexpr uint8_t ledReversePanelPin = 12;     // GPB4
constexpr uint8_t ledSpeedMediumPanelPin = 13; // GPB5
constexpr uint8_t ledSpeedHighPanelPin = 14;   // GPB6
constexpr uint8_t ledForwardPanelPin = 15;     // GPB7

static const uint32_t mcpManualVactualTable[3] = {
    vactualNormal,
    vactualMedium,
    vactualButton
};

constexpr uint16_t swI2cDelayUs = 6;

// Forward declarations
static void _setMotorCurrent(uint16_t currentMa);
static void _setLocalLed(uint8_t pin, bool on);
static void _setDirectionLeds(bool forwardOn, bool reverseOn);
static void _allDirectionLedsOff();
static void _updatePanelIndicators();
static void _handleSerial();
static void _runSerialCommand(const String &command);
static void _printDriverCheck();
static void _configureDriver();
static void _refreshDriverWhileIdle();
static void _syncErrorLatch();
static bool _debounce(DebouncedInput &input, bool raw);
static void _startLocalManualMotion(MotorState direction);
static void _stopLocalManualMotion();
static bool _runLocalManualMotion();
static bool _runPanelManualMotion();
static void _applyAutomaticMotorControl();
static int32_t _getLoadFeedBudgetMs();
static int32_t _getFeedBudgetMs();
static void _resetFeedGuard();
static void _accountFeedTime();
static void _runStartupLedSequence();
static bool _getStatusLedOn();
static uint32_t _makeBootJitterSeed();
static uint32_t _getRandomMotorInitDelayMs();

// Software I2C helpers
static void _swI2cReleaseScl();
static void _swI2cPullSclLow();
static void _swI2cReleaseSda();
static void _swI2cPullSdaLow();
static bool _swI2cReadScl();
static bool _swI2cReadSda();
static void _swI2cDelay();
static bool _swI2cWaitSclHigh(uint32_t timeoutUs = 1000);
static void _swI2cInitPins();
static void _swI2cBusRecovery();
static bool _swI2cStart();
static void _swI2cStop();
static bool _swI2cWriteBit(bool bitValue);
static bool _swI2cReadBit(bool &bitValue);
static bool _swI2cWriteByte(uint8_t byteValue, bool &ackReceived);
static bool _swI2cReadByte(uint8_t &byteValue, bool ackAfterRead);
static bool _swI2cWriteRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t value);
static bool _swI2cWriteWord(uint8_t deviceAddress, uint8_t regAddress, uint16_t value);
static bool _swI2cReadRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t &value);
static bool _swI2cProbeAddress(uint8_t deviceAddress);

// MCP helpers
static bool _writeMcpSetup();
static bool _probeMcp(bool verbose);
static void _dropPanel();
static void _debouncePanelInputs(uint8_t gpioA);
static void _servicePanel();
static bool _readMcpGpioA(uint8_t &gpioA);
static void _updateMcpButtonsFromGpioA(uint8_t gpioA);
static void _setMcpPinRaw(uint8_t pin, bool highLevel);
static void _setPanelLed(uint8_t pin, bool on);
static void _pulsePanelLed(uint8_t pin, const char *name);
static void _runMcpLedSequence();

static void _setMotorCurrent(uint16_t currentMa) {
    if (currentMa != currentCached) {
        driver.rms_current(currentMa);
        currentCached = currentMa;
    }
}

static void _setLocalLed(uint8_t pin, bool on) {
    // Local LEDs remain active-low
    digitalWrite(pin, on ? LOW : HIGH);
}

static void _setDirectionLeds(bool forwardOn, bool reverseOn) {
    _setLocalLed(ledForwardPin, forwardOn);
    _setLocalLed(ledReversePin, reverseOn);

    if (useMcp && mcpPresent) {
        _setPanelLed(ledForwardPanelPin, forwardOn);
        _setPanelLed(ledReversePanelPin, reverseOn);
    }
}

static void _allDirectionLedsOff() {
    _setDirectionLeds(false, false);
}

static bool _getStatusLedOn() {
    const uint32_t cyclePos = millis() % statusLedPeriodMs;
    return cyclePos >= statusLedOffPulseMs;
}

static void _setPanelLed(uint8_t pin, bool on) {
    if (!useMcp || !mcpPresent) {
        return;
    }

    // Flipped as requested: panel LEDs active-high
    _setMcpPinRaw(pin, on);
}

static void _setMcpPinRaw(uint8_t pin, bool highLevel) {
    if (!mcpPresent) {
        return;
    }

    const uint16_t mask = static_cast<uint16_t>(1UL << pin);
    uint16_t newShadow = mcpOlatShadow;

    if (highLevel) {
        newShadow |= mask;
    } else {
        newShadow &= static_cast<uint16_t>(~mask);
    }

    if (newShadow != mcpOlatShadow) {
        mcpOlatShadow = newShadow;
        _swI2cWriteWord(mcpAddress, mcpOlatA, mcpOlatShadow);
    }
}

static void _updatePanelIndicators() {
    if (!(useMcp && mcpPresent)) {
        return;
    }

    const bool filamentPresent = !bufferState.materialSwitchState;
    const bool panelManualSpeedActive =
        useMcpInMotorControl &&
        (panelLatchedMotorState != MotorState::stop);

    _setPanelLed(ledOverridePanelPin, mcpOverrideLatched);
    _setPanelLed(ledFilamentPanelPin, filamentPresent);
    _setPanelLed(ledStatusPanelPin, _getStatusLedOn());
    _setPanelLed(ledErrorPanelPin, isError);

    _setPanelLed(ledSpeedLowPanelPin, panelManualSpeedActive && (mcpManualSpeedIndex == 0));
    _setPanelLed(ledSpeedMediumPanelPin, panelManualSpeedActive && (mcpManualSpeedIndex == 1));
    _setPanelLed(ledSpeedHighPanelPin, panelManualSpeedActive && (mcpManualSpeedIndex == 2));
}

static void _pulsePanelLed(uint8_t pin, const char *name) {
    Serial.print("pulse panel led: ");
    Serial.println(name);
    _setPanelLed(pin, true);
    delay(120);
    _setPanelLed(pin, false);
    delay(120);
}

static void _runMcpLedSequence() {
    if (!mcpPresent) {
        Serial.println("mcp led test skipped");
        return;
    }

    _pulsePanelLed(ledOverridePanelPin, "override");
    _pulsePanelLed(ledSpeedLowPanelPin, "speed low");
    _pulsePanelLed(ledFilamentPanelPin, "filament");
    _pulsePanelLed(ledStatusPanelPin, "status");
    _pulsePanelLed(ledErrorPanelPin, "error");
    _pulsePanelLed(ledReversePanelPin, "reverse");
    _pulsePanelLed(ledSpeedMediumPanelPin, "speed medium");
    _pulsePanelLed(ledSpeedHighPanelPin, "speed high");
    _pulsePanelLed(ledForwardPanelPin, "forward");

    _updatePanelIndicators();
    _allDirectionLedsOff();
}

static void _runStartupLedSequence() {
    _setLocalLed(ledReversePin, true);
    delay(120);
    _setLocalLed(ledReversePin, false);
    delay(120);

    _setLocalLed(ledForwardPin, true);
    delay(120);
    _setLocalLed(ledForwardPin, false);
    delay(120);

    if (useMcp && mcpPresent) {
        _runMcpLedSequence();
    }
}

static uint32_t _makeBootJitterSeed() {
    uint32_t seed = micros();

    seed ^= static_cast<uint32_t>(digitalRead(hall1Pin)) << 0;
    seed ^= static_cast<uint32_t>(digitalRead(hall2Pin)) << 1;
    seed ^= static_cast<uint32_t>(digitalRead(hall3Pin)) << 2;
    seed ^= static_cast<uint32_t>(digitalRead(endstop3Pin)) << 3;
    seed ^= static_cast<uint32_t>(digitalRead(keyReversePin)) << 4;
    seed ^= static_cast<uint32_t>(digitalRead(keyForwardPin)) << 5;
    seed ^= static_cast<uint32_t>(digitalRead(keyForward2Pin)) << 6;
    seed ^= static_cast<uint32_t>(digitalRead(keyReverse2Pin)) << 7;

    for (uint8_t i = 0; i < 16; ++i) {
        seed ^= micros() << (i & 0x07);
        delayMicroseconds(37);
    }

    if (seed == 0) {
        seed = 0xA5A5A5A5UL;
    }

    return seed;
}

static uint32_t _getRandomMotorInitDelayMs() {
    randomSeed(_makeBootJitterSeed());
    return static_cast<uint32_t>(random(100, 1001));
}

// ===== Software I2C =====

static void _swI2cReleaseScl() {
    pinMode(mcpSclPin, INPUT_PULLUP);
}

static void _swI2cPullSclLow() {
    pinMode(mcpSclPin, OUTPUT);
    digitalWrite(mcpSclPin, LOW);
}

static void _swI2cReleaseSda() {
    pinMode(mcpSdaPin, INPUT_PULLUP);
}

static void _swI2cPullSdaLow() {
    pinMode(mcpSdaPin, OUTPUT);
    digitalWrite(mcpSdaPin, LOW);
}

static bool _swI2cReadScl() {
    pinMode(mcpSclPin, INPUT_PULLUP);
    return digitalRead(mcpSclPin) == HIGH;
}

static bool _swI2cReadSda() {
    pinMode(mcpSdaPin, INPUT_PULLUP);
    return digitalRead(mcpSdaPin) == HIGH;
}

static void _swI2cDelay() {
    delayMicroseconds(swI2cDelayUs);
}

static bool _swI2cWaitSclHigh(uint32_t timeoutUs) {
    _swI2cReleaseScl();
    const uint32_t startUs = micros();

    while (!_swI2cReadScl()) {
        if (micros() - startUs > timeoutUs) {
            return false;
        }
    }

    return true;
}

static void _swI2cInitPins() {
    pinMode(muxResetPin, OUTPUT);
    digitalWrite(muxResetPin, HIGH);
    delay(5);

    pinMode(mcpIntPin, INPUT_PULLUP);

    _swI2cReleaseScl();
    _swI2cReleaseSda();
    delay(2);
}

static void _swI2cBusRecovery() {
    _swI2cReleaseSda();
    _swI2cReleaseScl();
    _swI2cDelay();

    for (int i = 0; i < 9; ++i) {
        _swI2cPullSclLow();
        _swI2cDelay();
        _swI2cReleaseScl();
        _swI2cDelay();
    }

    _swI2cPullSdaLow();
    _swI2cDelay();
    _swI2cReleaseScl();
    _swI2cDelay();
    _swI2cReleaseSda();
    _swI2cDelay();
}

static bool _swI2cStart() {
    _swI2cReleaseSda();
    _swI2cReleaseScl();
    _swI2cDelay();

    if (!_swI2cReadSda() || !_swI2cReadScl()) {
        return false;
    }

    _swI2cPullSdaLow();
    _swI2cDelay();
    _swI2cPullSclLow();
    _swI2cDelay();

    return true;
}

static void _swI2cStop() {
    _swI2cPullSdaLow();
    _swI2cDelay();
    _swI2cReleaseScl();
    _swI2cDelay();
    _swI2cReleaseSda();
    _swI2cDelay();
}

static bool _swI2cWriteBit(bool bitValue) {
    if (bitValue) {
        _swI2cReleaseSda();
    } else {
        _swI2cPullSdaLow();
    }

    _swI2cDelay();

    if (!_swI2cWaitSclHigh()) {
        _swI2cPullSclLow();
        return false;
    }

    _swI2cDelay();
    _swI2cPullSclLow();
    _swI2cDelay();

    return true;
}

static bool _swI2cReadBit(bool &bitValue) {
    _swI2cReleaseSda();
    _swI2cDelay();

    if (!_swI2cWaitSclHigh()) {
        _swI2cPullSclLow();
        return false;
    }

    _swI2cDelay();
    bitValue = _swI2cReadSda();
    _swI2cPullSclLow();
    _swI2cDelay();

    return true;
}

static bool _swI2cWriteByte(uint8_t byteValue, bool &ackReceived) {
    for (int bitIndex = 7; bitIndex >= 0; --bitIndex) {
        if (!_swI2cWriteBit(((byteValue >> bitIndex) & 0x01U) != 0U)) {
            ackReceived = false;
            return false;
        }
    }

    bool ackBit = true;
    if (!_swI2cReadBit(ackBit)) {
        ackReceived = false;
        return false;
    }

    ackReceived = !ackBit;
    return true;
}

static bool _swI2cReadByte(uint8_t &byteValue, bool ackAfterRead) {
    byteValue = 0;

    for (int bitIndex = 7; bitIndex >= 0; --bitIndex) {
        bool bitValue = false;
        if (!_swI2cReadBit(bitValue)) {
            return false;
        }

        if (bitValue) {
            byteValue |= static_cast<uint8_t>(1U << bitIndex);
        }
    }

    if (!_swI2cWriteBit(!ackAfterRead)) {
        return false;
    }

    return true;
}

static bool _swI2cWriteRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t value) {
    bool ack = false;

    if (!_swI2cStart()) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>((deviceAddress << 1) | 0U), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(regAddress, ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(value, ack) || !ack) {
        _swI2cStop();
        return false;
    }

    _swI2cStop();
    return true;
}

static bool _swI2cWriteWord(uint8_t deviceAddress, uint8_t regAddress, uint16_t value) {
    bool ack = false;

    if (!_swI2cStart()) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>((deviceAddress << 1) | 0U), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(regAddress, ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>(value & 0xFFU), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>((value >> 8) & 0xFFU), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    _swI2cStop();
    return true;
}

static bool _swI2cReadRegister(uint8_t deviceAddress, uint8_t regAddress, uint8_t &value) {
    bool ack = false;

    if (!_swI2cStart()) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>((deviceAddress << 1) | 0U), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(regAddress, ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cStart()) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cWriteByte(static_cast<uint8_t>((deviceAddress << 1) | 1U), ack) || !ack) {
        _swI2cStop();
        return false;
    }

    if (!_swI2cReadByte(value, false)) {
        _swI2cStop();
        return false;
    }

    _swI2cStop();
    return true;
}

static bool _swI2cProbeAddress(uint8_t deviceAddress) {
    bool ack = false;

    if (!_swI2cStart()) {
        _swI2cStop();
        return false;
    }

    const bool ok = _swI2cWriteByte(static_cast<uint8_t>((deviceAddress << 1) | 0U), ack);
    _swI2cStop();

    return ok && ack;
}

// ===== MCP helpers =====

// Pin directions, pull-ups and the current LED state
static bool _writeMcpSetup() {
    bool ok = true;
    ok &= _swI2cWriteRegister(mcpAddress, mcpIodirA, 0x7F);
    ok &= _swI2cWriteRegister(mcpAddress, mcpIodirB, 0x00);
    ok &= _swI2cWriteRegister(mcpAddress, mcpGppuA, 0x7F);
    ok &= _swI2cWriteRegister(mcpAddress, mcpGppuB, 0x00);
    ok &= _swI2cWriteWord(mcpAddress, mcpOlatA, mcpOlatShadow);
    return ok;
}

static bool _probeMcp(bool verbose) {
    if (!useMcp) {
        mcpPresent = false;
        return false;
    }

    _swI2cInitPins();

    if (!_swI2cReadScl() || !_swI2cReadSda()) {
        _swI2cBusRecovery();
    }

    mcpPresent = _swI2cProbeAddress(mcpAddress);

    if (verbose) {
        Serial.print("mcp present: ");
        Serial.println(mcpPresent ? "yes" : "no");
    }

    if (!mcpPresent) {
        return false;
    }

    mcpOlatShadow = 0x0000;

    if (!_writeMcpSetup()) {
        if (verbose) {
            Serial.println("mcp config failed");
        }
        mcpPresent = false;
        return false;
    }

    // Start from the buttons as they are now, so one that is already held is not a new press
    uint8_t gpioA = 0x7F;
    if (_swI2cReadRegister(mcpAddress, mcpGpioA, gpioA)) {
        mcpGpioACache = gpioA;
    }

    previousMcpGpioA = gpioA & 0x7F;
    mcpCandidateGpioA = previousMcpGpioA;
    mcpReadFailures = 0;
    mcpHousekeepingLastMs = millis();

    if (verbose) {
        Serial.println("mcp config ok");
    }
    return true;
}

static bool _readMcpGpioA(uint8_t &gpioA) {
    if (!(useMcp && mcpPresent)) {
        return false;
    }

    if (_swI2cReadRegister(mcpAddress, mcpGpioA, gpioA)) {
        mcpGpioACache = gpioA;
        return true;
    }

    return false;
}

static void _updateMcpButtonsFromGpioA(uint8_t gpioA) {
    bufferState.panelOverridePressed = (gpioA & (1U << swOverridePin)) == 0U;
    bufferState.panelCancelPressed = (gpioA & (1U << swCancelPin)) == 0U;
    bufferState.panelSpeedIncPressed = (gpioA & (1U << swSpeedIncPin)) == 0U;
    bufferState.panelResetPressed = (gpioA & (1U << swResetPin)) == 0U;
    bufferState.panelForwardPressed = (gpioA & (1U << swForwardPin)) == 0U;
    bufferState.panelReversePressed = (gpioA & (1U << swReversePin)) == 0U;
    bufferState.panelSpeedDecPressed = (gpioA & (1U << swSpeedDecPin)) == 0U;

    const uint8_t fallingEdges = static_cast<uint8_t>((previousMcpGpioA ^ gpioA) & previousMcpGpioA);
    previousMcpGpioA = gpioA;

    if (fallingEdges & (1U << swResetPin)) {
        isError = false;
        isFront = false;
        frontTime = 0;
        panelLatchedMotorState = MotorState::stop;
        autoRetryCount = 0;
        autoRetryPauseActive = false;
        autoRetryRequested = false;
        forceAutoRefresh = true;
        _resetFeedGuard();

        if (localManualState != MotorState::stop) {
            // Reset also ends a local button feed; the button has to be released first
            localManualState = MotorState::stop;
            localManualLockout = true;
        }

        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
    }

    if (fallingEdges & (1U << swSpeedIncPin)) {
        if (mcpManualSpeedIndex < 2) {
            ++mcpManualSpeedIndex;
        }
    }

    if (fallingEdges & (1U << swSpeedDecPin)) {
        if (mcpManualSpeedIndex > 0) {
            --mcpManualSpeedIndex;
        }
    }

    if (fallingEdges & (1U << swOverridePin)) {
        mcpOverrideLatched = !mcpOverrideLatched;
    }

    if (fallingEdges & (1U << swCancelPin)) {
        panelLatchedMotorState = MotorState::stop;
        isFront = false;
        frontTime = 0;
        forceAutoRefresh = true;
    }

    if (fallingEdges & (1U << swForwardPin)) {
        panelLatchedMotorState = MotorState::forward;
        panelLatchStartMs = millis();
        _resetFeedGuard();
        autoRetryCount = 0;
        autoRetryPauseActive = false;
        autoRetryRequested = false;
        isFront = false;
        frontTime = 0;
        isError = false;
    }

    if (fallingEdges & (1U << swReversePin)) {
        panelLatchedMotorState = MotorState::back;
        panelLatchStartMs = millis();
        _resetFeedGuard();
        autoRetryCount = 0;
        autoRetryPauseActive = false;
        autoRetryRequested = false;
        isFront = false;
        frontTime = 0;
        isError = false;
    }
}

// The panel stopped answering: forget it until it can be set up again
static void _dropPanel() {
    mcpPresent = false;
    mcpReadFailures = 0;
    mcpHousekeepingLastMs = millis();

    // Nothing is held any more, and nobody can press Cancel
    _updateMcpButtonsFromGpioA(0x7F);
    mcpCandidateGpioA = 0x7F;

    if (panelLatchedMotorState != MotorState::stop) {
        panelLatchedMotorState = MotorState::stop;
        forceAutoRefresh = true;
    }

    _swI2cBusRecovery();
}

static void _debouncePanelInputs(uint8_t gpioA) {
    const uint32_t nowMs = millis();

    if (gpioA != mcpCandidateGpioA) {
        mcpCandidateGpioA = gpioA;
        mcpCandidateSinceMs = nowMs;
        return;
    }

    if (gpioA == previousMcpGpioA || (nowMs - mcpCandidateSinceMs) < panelDebounceMs) {
        return;
    }

    // A panel chip that reset has lost its pull-ups, so its inputs float and
    // read as presses. Check before believing a change.
    uint8_t pullups = 0;
    if (!_swI2cReadRegister(mcpAddress, mcpGppuA, pullups) || pullups != 0x7F) {
        _writeMcpSetup();
        mcpCandidateGpioA = previousMcpGpioA;
        return;
    }

    _updateMcpButtonsFromGpioA(gpioA);
}

static void _servicePanel() {
    if (!useMcp) {
        return;
    }

    const uint32_t nowMs = millis();
    const bool housekeepingDue = (nowMs - mcpHousekeepingLastMs) >= panelSetupRefreshMs;

    if (!mcpPresent) {
        if (housekeepingDue) {
            mcpHousekeepingLastMs = nowMs;
            _probeMcp(false);
        }
        return;
    }

    if (housekeepingDue) {
        // Restores pin directions, pull-ups and LEDs if the panel chip lost power
        mcpHousekeepingLastMs = nowMs;
        _writeMcpSetup();
    }

    uint8_t gpioA = 0x7F;
    if (!_readMcpGpioA(gpioA)) {
        if (++mcpReadFailures >= panelMaxReadFailures) {
            _dropPanel();
        }
        return;
    }

    mcpReadFailures = 0;
    _debouncePanelInputs(gpioA & 0x7F);
}

static bool _debounce(DebouncedInput &input, bool raw) {
    const uint32_t nowMs = millis();

    if (raw != input.candidate) {
        input.candidate = raw;
        input.candidateSinceMs = nowMs;
    } else if (raw != input.stable && (nowMs - input.candidateSinceMs) >= inputDebounceMs) {
        input.stable = raw;
    }

    return input.stable;
}

void bufferSensorInit() {
    pinMode(hall1Pin, INPUT);
    pinMode(hall2Pin, INPUT);
    pinMode(hall3Pin, INPUT);
    pinMode(endstop3Pin, INPUT);

    pinMode(keyReversePin, INPUT_PULLUP);
    pinMode(keyForwardPin, INPUT_PULLUP);
    pinMode(keyReverse2Pin, INPUT_PULLUP);
    pinMode(keyForward2Pin, INPUT_PULLUP);

    pinMode(ledReversePin, OUTPUT);
    pinMode(ledForwardPin, OUTPUT);
    _allDirectionLedsOff();

    // Start the runout filter from the real switch state so boot does not
    // briefly believe filament is loaded
    runoutInput.stable = digitalRead(endstop3Pin) == HIGH;
    runoutInput.candidate = runoutInput.stable;

    if (useMcp) {
        _probeMcp(true);
    }
}

// Sends every driver setting. Only call this while the motor is stopped.
static void _configureDriver() {
    driver.I_scale_analog(false);
    driver.toff(5);
    driver.rms_current(currentCached);
    driver.microsteps(moveDivideNum);
    driver.VACTUAL(stopValue);
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);
}

// The driver forgets its settings when it loses motor power while the MCU
// stays up (USB power, brown-out), so they are repeated whenever it is idle
static void _refreshDriverWhileIdle() {
    if (motorState != MotorState::stop) {
        return;
    }

    if (millis() - driverRefreshLastMs < driverRefreshIntervalMs) {
        return;
    }

    driverRefreshLastMs = millis();
    _configureDriver();
}

void bufferMotorInit() {
    pinMode(enPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirMotorPin, OUTPUT);

    // The driver takes its settings while disabled; motor control enables it
    // when it starts a move
    digitalWrite(enPin, HIGH);

    driver.beginSerial(9600);
    _configureDriver();
    driverRefreshLastMs = millis();
}

void readSensorState() {
    bufferState.pos1SensorState = digitalRead(hall3Pin) == HIGH;
    bufferState.pos2SensorState = digitalRead(hall2Pin) == HIGH;
    bufferState.pos3SensorState = digitalRead(hall1Pin) == HIGH;

    bufferState.materialSwitchState =
        _debounce(runoutInput, digitalRead(endstop3Pin) == HIGH);

    bufferState.localReversePressed = _debounce(
        localReverseInput,
        (digitalRead(keyReversePin) == LOW) || (digitalRead(keyReverse2Pin) == LOW));

    bufferState.localForwardPressed = _debounce(
        localForwardInput,
        (digitalRead(keyForwardPin) == LOW) || (digitalRead(keyForward2Pin) == LOW));

    _servicePanel();
}

static void _startLocalManualMotion(MotorState direction) {
    const bool forward = (direction == MotorState::forward);

    panelLatchedMotorState = MotorState::stop;
    autoRetryCount = 0;
    autoRetryPauseActive = false;
    autoRetryRequested = false;
    forceAutoRefresh = false;
    isFront = false;
    frontTime = 0;

    _setDirectionLeds(forward, !forward);
    digitalWrite(enPin, LOW);
    driver.VACTUAL(stopValue);
    _setMotorCurrent(currentButtonMa);

    driver.shaft(forward ? 1 : 0);
    driver.VACTUAL(vactualButton);

    motorState = direction;
    localManualState = direction;
    localManualStartMs = millis();
}

static void _stopLocalManualMotion() {
    driver.VACTUAL(stopValue);
    motorState = MotorState::stop;
    localManualState = MotorState::stop;
    isFront = false;
    frontTime = 0;
    isError = false;
    digitalWrite(enPin, HIGH);
    _allDirectionLedsOff();
    forceAutoRefresh = true;
    _resetFeedGuard();
}

static bool _runLocalManualMotion() {
    const bool reversePressed = bufferState.localReversePressed;
    const bool forwardPressed = bufferState.localForwardPressed;

    if (!reversePressed && !forwardPressed) {
        localManualLockout = false;
    }

    if (localManualState != MotorState::stop) {
        const bool stillHeld =
            (localManualState == MotorState::back) ? reversePressed : forwardPressed;
        const bool capReached = (millis() - localManualStartMs) >= localButtonMaxMs;

        if (stillHeld && !capReached) {
            // The local buttons win over anything pressed on the panel meanwhile
            panelLatchedMotorState = MotorState::stop;
            return true;
        }

        if (stillHeld) {
            // Held past the cap (stuck button, shorted wire): ignore it until released
            localManualLockout = true;
        }

        _stopLocalManualMotion();
        return true;
    }

    if (localManualLockout) {
        return false;
    }

    if (reversePressed && forwardPressed) {
        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
        return true;
    }

    if (reversePressed) {
        _startLocalManualMotion(MotorState::back);
        return true;
    }

    if (forwardPressed) {
        _startLocalManualMotion(MotorState::forward);
        return true;
    }

    return false;
}

static bool _runPanelManualMotion() {
    if (!(useMcp && useMcpInMotorControl && mcpPresent)) {
        return false;
    }

    if (panelLatchedMotorState == MotorState::stop) {
        return false;
    }

    if (millis() - panelLatchStartMs >= panelLatchMaxMs) {
        // Nobody cancelled the latched motion, so hand back to automatic control
        panelLatchedMotorState = MotorState::stop;
        forceAutoRefresh = true;
        return false;
    }

    const uint32_t panelVactual = mcpManualVactualTable[mcpManualSpeedIndex];

    isFront = false;
    frontTime = 0;
    isError = false;

    if (panelLatchedMotorState == MotorState::forward) {
        _setDirectionLeds(true, false);
        digitalWrite(enPin, LOW);

        if (motorState != MotorState::forward) {
            driver.VACTUAL(stopValue);
        }

        _setMotorCurrent(currentButtonMa);
        driver.shaft(1);
        driver.VACTUAL(panelVactual);
        motorState = MotorState::forward;
        return true;
    }

    if (panelLatchedMotorState == MotorState::back) {
        _setDirectionLeds(false, true);
        digitalWrite(enPin, LOW);

        if (motorState != MotorState::back) {
            driver.VACTUAL(stopValue);
        }

        _setMotorCurrent(currentButtonMa);
        driver.shaft(0);
        driver.VACTUAL(panelVactual);
        motorState = MotorState::back;
        return true;
    }

    return false;
}

static void _applyAutomaticMotorControl() {
    static MotorState lastValidAutoState = MotorState::stop;
    static uint32_t lastValidAutoMs = 0;

    constexpr uint32_t autoInvalidGraceMs = 120;

    if (autoRetryPauseActive) {
        if (millis() - autoRetryPauseStartMs < autoRetryPauseMs) {
            if (motorState != MotorState::stop) {
                driver.VACTUAL(stopValue);
                motorState = MotorState::stop;
                digitalWrite(enPin, HIGH);
                _allDirectionLedsOff();
            }
            isFront = false;
            frontTime = 0;
            return;
        }

        autoRetryPauseActive = false;
        forceAutoRefresh = true;
    }

    const bool pos1 = bufferState.pos1SensorState;
    const bool pos2 = bufferState.pos2SensorState;
    const bool pos3 = bufferState.pos3SensorState;

    bool rawStateValid = false;
    MotorState rawRequestedState = MotorState::stop;

    if (pos1 && !pos2 && !pos3) {
        rawRequestedState = MotorState::forward;
        rawStateValid = true;
    } else if (!pos1 && pos2 && !pos3) {
        rawRequestedState = MotorState::stop;
        rawStateValid = true;
    } else if (!pos1 && !pos2 && pos3) {
        rawRequestedState = MotorState::back;
        rawStateValid = true;
    }

    MotorState requestedState = MotorState::stop;

    if (rawStateValid) {
        lastValidAutoState = rawRequestedState;
        lastValidAutoMs = millis();
        requestedState = rawRequestedState;
    } else {
        if (millis() - lastValidAutoMs <= autoInvalidGraceMs) {
            requestedState = lastValidAutoState;
        } else {
            requestedState = MotorState::stop;
        }
    }

    const uint32_t autoVactual = mcpManualVactualTable[0];

    if (requestedState == MotorState::forward) {
        isFront = true;
        feedAccounting = FeedAccounting::feeding;
    } else {
        isFront = false;
        frontTime = 0;
        autoRetryCount = 0;
        autoRetryPauseActive = false;
        autoRetryRequested = false;
        feedAccounting = (requestedState == MotorState::back)
            ? FeedAccounting::retracting
            : FeedAccounting::idle;
    }

    if (!forceAutoRefresh && requestedState == motorState) {
        return;
    }

    driver.VACTUAL(stopValue);

    switch (requestedState) {
        case MotorState::forward: {
            motorState = MotorState::forward;
            digitalWrite(enPin, LOW);
            _setMotorCurrent(currentAutoMa);
            driver.shaft(1);
            driver.VACTUAL(autoVactual);
            _setDirectionLeds(true, false);
            break;
        }

        case MotorState::stop: {
            motorState = MotorState::stop;
            digitalWrite(enPin, HIGH);
            _allDirectionLedsOff();
            break;
        }

        case MotorState::back: {
            motorState = MotorState::back;
            digitalWrite(enPin, LOW);
            _setMotorCurrent(currentAutoMa);
            driver.shaft(0);
            driver.VACTUAL(autoVactual);
            _setDirectionLeds(false, true);
            break;
        }
    }

    forceAutoRefresh = false;
}

// ===== Runaway feed guard =====

static int32_t _getLoadFeedBudgetMs() {
    constexpr uint32_t attempts = static_cast<uint32_t>(maxAutoRetries) + 1U;
    constexpr uint32_t budgetCapMs = 0x3FFFFFFFUL;

    if (timeoutMs > budgetCapMs / attempts) {
        return static_cast<int32_t>(budgetCapMs);
    }

    return static_cast<int32_t>(timeoutMs * attempts);
}

static int32_t _getFeedBudgetMs() {
    const int32_t loadBudgetMs = _getLoadFeedBudgetMs();

    if (feedPrimed && static_cast<int32_t>(primedFeedBudgetMs) < loadBudgetMs) {
        return static_cast<int32_t>(primedFeedBudgetMs);
    }

    return loadBudgetMs;
}

static void _resetFeedGuard() {
    feedNetMs = 0;
    feedDwellMs = 0;
    feedSeenSinceReset = false;
    feedPrimed = false;
}

// Charges the time since the previous pass to what automatic control was doing
static void _accountFeedTime() {
    constexpr uint32_t elapsedCapMs = 60000;

    const uint32_t nowMs = millis();
    uint32_t elapsedMs = nowMs - feedAccountLastMs;
    feedAccountLastMs = nowMs;

    if (elapsedMs > elapsedCapMs) {
        elapsedMs = elapsedCapMs;
    }

    const int32_t elapsed = static_cast<int32_t>(elapsedMs);

    switch (feedAccounting) {
        case FeedAccounting::hold: {
            return;
        }

        case FeedAccounting::feeding: {
            feedNetMs += elapsed;
            feedDwellMs = 0;
            feedSeenSinceReset = true;

            if (feedNetMs > _getFeedBudgetMs()) {
                isError = true;
            }
            return;
        }

        case FeedAccounting::retracting: {
            // Filament pulled back is credit for feeding it forward again
            const int32_t creditFloorMs = -_getLoadFeedBudgetMs();

            feedNetMs -= elapsed;
            if (feedNetMs < creditFloorMs) {
                // Pulled back a whole load more than was ever fed: the slider
                // is jammed in the full position or its sensor has failed
                feedNetMs = creditFloorMs;
                isError = true;
            }
            break;
        }

        case FeedAccounting::idle: {
            if (feedNetMs > 0) {
                feedNetMs -= elapsed * feedIdleRefundRate;
                if (feedNetMs < 0) {
                    feedNetMs = 0;
                }
            }
            break;
        }
    }

    if (feedDwellMs < feedPrimeDwellMs) {
        feedDwellMs += elapsedMs;
    }

    if (!feedPrimed && feedSeenSinceReset && feedDwellMs >= feedPrimeDwellMs) {
        // The buffer filled, so everything fed so far went where it should
        feedPrimed = true;
        if (feedNetMs > 0) {
            feedNetMs = 0;
        }
    }
}

void motorControl() {
    // Every path below that is not automatic control leaves the budget untouched
    _accountFeedTime();
    feedAccounting = FeedAccounting::hold;

    if (autoRetryRequested) {
        noInterrupts();
        autoRetryRequested = false;
        interrupts();

        if (autoRetryCount < UINT8_MAX) {
            ++autoRetryCount;
        }

        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();

        autoRetryPauseActive = true;
        autoRetryPauseStartMs = millis();
        isFront = false;
        frontTime = 0;
        return;
    }

    if (_runLocalManualMotion()) {
        return;
    }

    if (_runPanelManualMotion()) {
        return;
    }

    if (useMcp && useMcpInMotorControl && mcpPresent && mcpOverrideLatched) {
        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        isFront = false;
        frontTime = 0;
        _allDirectionLedsOff();
        return;
    }

    if (bufferState.materialSwitchState) {
        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        panelLatchedMotorState = MotorState::stop;
        autoRetryCount = 0;
        autoRetryPauseActive = false;
        autoRetryRequested = false;
        isFront = false;
        frontTime = 0;
        isError = false;
        _resetFeedGuard();
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
        return;
    }

    if (isError) {
        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        panelLatchedMotorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
        return;
    }

    _applyAutomaticMotorControl();
}

// Keeps the saved error state in step with the live one, so a unit that
// tripped stays stopped across a power cycle. Runs after motor control, so the
// motor is already stopped before the slow EEPROM write.
static void _syncErrorLatch() {
    if (isError == errorLatchStored) {
        return;
    }

    errorLatchStored = isError;
    EEPROM.write(eepromAddrErrorLatch, isError ? errorLatchValue : 0);
}

// Read-only: reports which microstep setting the driver is really using.
// microsteps() only takes effect when mstep_reg_select is set; otherwise the
// driver follows its MS1/MS2 pins.
static void _printDriverCheck() {
    const uint32_t ioin = driver.IOIN();

    if (driver.CRCerror) {
        Serial.println("driver reply=none");
        return;
    }

    const uint32_t gconf = driver.GCONF();
    bool allRead = !driver.CRCerror;
    const uint32_t chopconf = driver.CHOPCONF();
    allRead = allRead && !driver.CRCerror;
    const uint8_t ifcnt = driver.IFCNT();
    allRead = allRead && !driver.CRCerror;

    if (!allRead) {
        Serial.println("driver reply=partial, values below may be wrong");
    } else {
        Serial.println("driver reply=ok");
    }

    const uint8_t version = static_cast<uint8_t>(ioin >> 24);
    const bool ms1 = (ioin & (1UL << 2)) != 0;
    const bool ms2 = (ioin & (1UL << 3)) != 0;
    const bool pdnDisable = (gconf & (1UL << 6)) != 0;
    const bool mstepRegSelect = (gconf & (1UL << 7)) != 0;
    const uint16_t registerMicrosteps = static_cast<uint16_t>(256U >> ((chopconf >> 24) & 0x0FU));

    // TMC2209 pin table, indexed by MS2:MS1
    static const uint8_t pinMicrostepTable[4] = {8, 32, 64, 16};
    const bool isTmc2209 = (version == 0x21);
    const uint8_t pinMicrosteps =
        pinMicrostepTable[(ms2 ? 2U : 0U) | (ms1 ? 1U : 0U)];

    Serial.print("version=0x");
    Serial.print(version, HEX);
    Serial.print(" ms1=");
    Serial.print(ms1);
    Serial.print(" ms2=");
    Serial.println(ms2);

    Serial.print("pdn_disable=");
    Serial.print(pdnDisable);
    Serial.print(" mstep_reg_select=");
    Serial.println(mstepRegSelect);

    Serial.print("register microsteps=");
    Serial.println(registerMicrosteps);

    Serial.print("pin microsteps=");
    if (isTmc2209) {
        Serial.println(pinMicrosteps);
    } else {
        Serial.println("unknown chip");
    }

    Serial.print("microsteps in use=");
    if (mstepRegSelect) {
        Serial.print(registerMicrosteps);
        Serial.println(" (register)");
    } else if (isTmc2209) {
        Serial.print(pinMicrosteps);
        Serial.println(" (pins)");
    } else {
        Serial.println("set by pins");
    }

    Serial.print("ifcnt=");
    Serial.println(ifcnt);
}

static void _runSerialCommand(const String &command) {
    if (command == "rt") {
        Serial.print("read timeout=");
        Serial.println(timeoutMs);
        Serial.print("retry count=");
        Serial.println(autoRetryCount);
        Serial.print("feed used ms=");
        Serial.print(feedNetMs);
        Serial.print(" / ");
        Serial.print(_getFeedBudgetMs());
        Serial.println(feedPrimed ? " (primed)" : " (loading)");
        Serial.print("error=");
        Serial.println(isError);
        return;
    }

    if (command == "ml") {
        _runMcpLedSequence();
        return;
    }

    if (command == "mi") {
        Serial.print("mcp present=");
        Serial.println(mcpPresent ? "yes" : "no");
        if (mcpPresent) {
            uint8_t gpioA = 0x7F;
            if (_readMcpGpioA(gpioA)) {
                Serial.print("mcp gpioA=0x");
                if (gpioA < 0x10) {
                    Serial.print('0');
                }
                Serial.println(gpioA, HEX);
            }
        }
        return;
    }

    if (command == "dc") {
        _printDriverCheck();
        return;
    }

    if (command.startsWith("set")) {
        uint32_t value = 0;
        uint8_t digits = 0;
        bool valid = true;

        for (unsigned int i = 3; i < command.length(); ++i) {
            const char c = command.charAt(i);

            if (c == ' ' && digits == 0) {
                continue;
            }

            if (c < '0' || c > '9' || digits >= 6) {
                valid = false;
                break;
            }

            value = value * 10U + static_cast<uint32_t>(c - '0');
            ++digits;
        }

        if (!valid || digits == 0 || value < minTimeoutMs || value > maxTimeoutMs) {
            Serial.print("Error: timeout must be ");
            Serial.print(minTimeoutMs);
            Serial.print(" to ");
            Serial.print(maxTimeoutMs);
            Serial.println(" ms.");
            return;
        }

        timeoutMs = value;
        EEPROM.put(eepromAddrTimeout, timeoutMs);
        Serial.print("set succeed! timeout=");
        Serial.println(timeoutMs);
        return;
    }

    Serial.println(command);
    Serial.println("command error!");
}

static void _handleSerial() {
    bool lineEnded = false;

    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        serialLastByteMs = millis();

        if (c == '\r' || c == '\n') {
            if (serialBuffer.length() > 0) {
                lineEnded = true;
                break;
            }
            continue;
        }

        if (serialBuffer.length() < serialMaxCommandLength) {
            serialBuffer += c;
        }
    }

    if (serialBuffer.length() == 0) {
        return;
    }

    // Without a line ending, wait for the line to go quiet, so a command that
    // arrives in pieces is never run half-read
    if (!lineEnded && (millis() - serialLastByteMs) < serialIdleGapMs) {
        return;
    }

    _runSerialCommand(serialBuffer);
    serialBuffer = "";
}

void bufferInit() {
    bufferSensorInit();

    const uint32_t motorInitDelayMs = _getRandomMotorInitDelayMs();
    Serial.print("motor init delay ms=");
    Serial.println(motorInitDelayMs);
    delay(motorInitDelayMs);

    bufferMotorInit();

    delay(1000);

    EEPROM.get(eepromAddrTimeout, timeoutMs);
    if (timeoutMs < minTimeoutMs || timeoutMs > maxTimeoutMs) {
        timeoutMs = defaultTimeoutMs;
        EEPROM.put(eepromAddrTimeout, timeoutMs);
        Serial.println("stored timeout missing or out of range, using default");
    } else {
        Serial.print("read timeout: ");
        Serial.println(timeoutMs);
    }

    // A unit that tripped stays stopped across a power cycle until it is cleared
    errorLatchStored = EEPROM.read(eepromAddrErrorLatch) == errorLatchValue;
    isError = errorLatchStored;
    if (isError) {
        Serial.println("error state restored from before power-off");
    }

    timer6.pause();
    timer6.setPrescaleFactor(48);
    timer6.setOverflow(1000);
    timer6.attachInterrupt(&timerItCallback);
    timer6.resume();

    _runStartupLedSequence();
    _updatePanelIndicators();
    _allDirectionLedsOff();

    if (useMcp) {
        Serial.print("mcp enabled, present=");
        Serial.println(mcpPresent ? "yes" : "no");
    } else {
        Serial.println("mcp disabled");
    }

    // Started last: boot has long deliberate delays, and the driver is
    // disabled throughout it anyway
    if (IWatchdog.isReset(true)) {
        Serial.println("restarted by watchdog");
    }
    IWatchdog.begin(watchdogTimeoutUs);
}

void bufferLoop() {
    IWatchdog.reload();
    readSensorState();

    if (debugEnabled) {
        bufferDebug();
    } else {
        motorControl();
        _syncErrorLatch();
        _refreshDriverWhileIdle();
        _updatePanelIndicators();
        _handleSerial();
    }
}

void timerItCallback() {
    if (!isFront) {
        return;
    }

    frontTime = frontTime + 1;

    // This only restarts a long feed (stall recovery); the feed budget decides when to give up
    const uint32_t kickMs =
        (feedPrimed && primedFeedKickMs < timeoutMs) ? primedFeedKickMs : timeoutMs;

    if (frontTime <= kickMs) {
        return;
    }

    isFront = false;
    frontTime = 0;
    autoRetryRequested = true;
}

void bufferDebug() {
    Serial.print("pos1=");
    Serial.print(bufferState.pos1SensorState);
    Serial.print(" pos2=");
    Serial.print(bufferState.pos2SensorState);
    Serial.print(" pos3=");
    Serial.print(bufferState.pos3SensorState);
    Serial.print(" filamentMissing=");
    Serial.print(bufferState.materialSwitchState);
    Serial.print(" localRev=");
    Serial.print(bufferState.localReversePressed);
    Serial.print(" localFwd=");
    Serial.print(bufferState.localForwardPressed);
    Serial.print(" panelRev=");
    Serial.print(bufferState.panelReversePressed);
    Serial.print(" panelFwd=");
    Serial.print(bufferState.panelForwardPressed);
    Serial.print(" override=");
    Serial.print(mcpOverrideLatched);
    Serial.print(" speedIdx=");
    Serial.print(mcpManualSpeedIndex);
    Serial.print(" panelLatched=");
    Serial.print(static_cast<int>(panelLatchedMotorState));
    Serial.print(" retryCount=");
    Serial.print(autoRetryCount);
    Serial.print(" retryPause=");
    Serial.print(autoRetryPauseActive);
    Serial.print(" timeoutMs=");
    Serial.println(timeoutMs);
    delay(300);
}