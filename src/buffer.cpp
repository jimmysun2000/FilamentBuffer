#include "buffer.h"

TMC2209Stepper driver(uartPin, uartPin, rSense, driverAddress);

bool isError = false;
uint32_t frontTime = 0;
uint32_t timeoutMs = 60000;
bool isFront = false;

static BufferState bufferState = {};
static String serialBuffer;
static HardwareTimer timer6(TIM6);
static bool forceAutoRefresh = false;

static uint16_t currentCached = currentNormalMa;
static MotorState motorState = MotorState::stop;
static MotorState lastAutoMotorState = MotorState::stop;
static MotorState panelLatchedMotorState = MotorState::stop;

static const int eepromAddrTimeout = 0;
static const uint32_t defaultTimeoutMs = 60000;

// MCP state
static bool mcpPresent = false;
static uint16_t mcpOlatShadow = 0xFFFF;
static uint8_t mcpGpioACache = 0x7F;
static uint8_t previousMcpGpioA = 0x7F;

static bool mcpOverrideLatched = false;
static uint8_t mcpManualSpeedIndex = 2; // default high, matches original button jog speed
static bool statusBlinkState = false;
static uint32_t lastStatusToggleMs = 0;

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

static void _setMotorCurrent(uint16_t currentMa);
static void _setLocalLed(uint8_t pin, bool on);
static void _setDirectionLeds(bool forwardOn, bool reverseOn);
static void _allDirectionLedsOff();
static void _updatePanelIndicators();
static void _handleSerial();
static bool _runLocalManualMotion();
static bool _runPanelManualMotion();
static void _applyAutomaticMotorControl();
static void _runStartupLedSequence();
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
static bool _probeMcp();
static bool _readMcpGpioA(uint8_t &gpioA);
static void _updateMcpButtonsFromGpioA(uint8_t gpioA);
static void _setMcpPinRaw(uint8_t pin, bool highLevel);
static void _setPanelLed(uint8_t pin, bool on);
static void _pulsePanelLed(uint8_t pin, const char *name);
static void _runMcpLedSequence();

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

static void _setMotorCurrent(uint16_t currentMa) {
    if (currentMa != currentCached) {
        driver.rms_current(currentMa);
        currentCached = currentMa;
    }
}

static void _setLocalLed(uint8_t pin, bool on) {
    // Local LEDs are active-low in your original code
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

static void _setPanelLed(uint8_t pin, bool on) {
    if (!useMcp || !mcpPresent) {
        return;
    }

    // Panel LEDs are active-low
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

    // Only show speed LEDs when panel manual motion is actually active.
    const bool panelManualSpeedActive =
        useMcpInMotorControl &&
        (panelLatchedMotorState != MotorState::stop);

    _setPanelLed(ledOverridePanelPin, mcpOverrideLatched);
    _setPanelLed(ledFilamentPanelPin, filamentPresent);
    _setPanelLed(ledStatusPanelPin, statusBlinkState);
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

static bool _probeMcp() {
    if (!useMcp) {
        mcpPresent = false;
        return false;
    }

    _swI2cInitPins();

    if (!_swI2cReadScl() || !_swI2cReadSda()) {
        _swI2cBusRecovery();
    }

    mcpPresent = _swI2cProbeAddress(mcpAddress);

    Serial.print("mcp present: ");
    Serial.println(mcpPresent ? "yes" : "no");

    if (!mcpPresent) {
        return false;
    }

    bool ok = true;
    ok &= _swI2cWriteRegister(mcpAddress, mcpIodirA, 0x7F); // GPA0..6 in, GPA7 out
    ok &= _swI2cWriteRegister(mcpAddress, mcpIodirB, 0x00); // GPB all out
    ok &= _swI2cWriteRegister(mcpAddress, mcpGppuA, 0x7F);
    ok &= _swI2cWriteRegister(mcpAddress, mcpGppuB, 0x00);

    mcpOlatShadow = 0xFFFF;
    ok &= _swI2cWriteWord(mcpAddress, mcpOlatA, mcpOlatShadow);

    if (!ok) {
        Serial.println("mcp config failed");
        mcpPresent = false;
        return false;
    }

    uint8_t gpioA = 0x7F;
    if (_swI2cReadRegister(mcpAddress, mcpGpioA, gpioA)) {
        mcpGpioACache = gpioA;
        previousMcpGpioA = gpioA;
    }

    Serial.println("mcp config ok");
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
        forceAutoRefresh = true;
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

    // CANCEL exits latched manual mode and returns control to auto.
    if (fallingEdges & (1U << swCancelPin)) {
        panelLatchedMotorState = MotorState::stop;
        isFront = false;
        frontTime = 0;
        forceAutoRefresh = true;
    }

    // LOAD -> latched forward manual
    if (fallingEdges & (1U << swForwardPin)) {
        panelLatchedMotorState = MotorState::forward;
        isFront = false;
        frontTime = 0;
        isError = false;
    }

    // UNLOAD -> latched reverse manual
    if (fallingEdges & (1U << swReversePin)) {
        panelLatchedMotorState = MotorState::back;
        isFront = false;
        frontTime = 0;
        isError = false;
    }
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

    if (useMcp) {
        _probeMcp();
    }
}

void bufferMotorInit() {
    pinMode(enPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirMotorPin, OUTPUT);

    digitalWrite(enPin, LOW);

    driver.beginSerial(9600);
    driver.I_scale_analog(false);
    driver.toff(5);
    driver.rms_current(currentNormalMa);
    driver.microsteps(moveDivideNum);
    driver.VACTUAL(stopValue);
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);
}

void readSensorState() {
    // Keep the same logical mapping as your working code
    bufferState.pos1SensorState = digitalRead(hall3Pin) == HIGH;
    bufferState.pos2SensorState = digitalRead(hall2Pin) == HIGH;
    bufferState.pos3SensorState = digitalRead(hall1Pin) == HIGH;
    bufferState.materialSwitchState = digitalRead(endstop3Pin) == HIGH;

    bufferState.localReversePressed =
        (digitalRead(keyReversePin) == LOW) ||
        (digitalRead(keyReverse2Pin) == LOW);

    bufferState.localForwardPressed =
        (digitalRead(keyForwardPin) == LOW) ||
        (digitalRead(keyForward2Pin) == LOW);

    if (useMcp && mcpPresent) {
        uint8_t gpioA = 0x7F;
        if (_readMcpGpioA(gpioA)) {
            _updateMcpButtonsFromGpioA(gpioA);
        }
    } else {
        bufferState.panelOverridePressed = false;
        bufferState.panelCancelPressed = false;
        bufferState.panelSpeedIncPressed = false;
        bufferState.panelResetPressed = false;
        bufferState.panelForwardPressed = false;
        bufferState.panelReversePressed = false;
        bufferState.panelSpeedDecPressed = false;
    }
}

static bool _runLocalManualMotion() {
    const bool reversePressed = bufferState.localReversePressed;
    const bool forwardPressed = bufferState.localForwardPressed;

    if (reversePressed && forwardPressed) {
        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
        return true;
    }

    if (reversePressed) {
        _setDirectionLeds(false, true);
        digitalWrite(enPin, LOW);
        driver.VACTUAL(stopValue);
        _setMotorCurrent(currentButtonMa);

        driver.shaft(0);
        driver.VACTUAL(vactualButton);

        while ((digitalRead(keyReversePin) == LOW) || (digitalRead(keyReverse2Pin) == LOW)) {
            delay(1);
        }

        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;

        isFront = false;
        frontTime = 0;
        isError = false;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
        return true;
    }

    if (forwardPressed) {
        _setDirectionLeds(true, false);
        digitalWrite(enPin, LOW);
        driver.VACTUAL(stopValue);
        _setMotorCurrent(currentButtonMa);

        driver.shaft(1);
        driver.VACTUAL(vactualButton);

        while ((digitalRead(keyForwardPin) == LOW) || (digitalRead(keyForward2Pin) == LOW)) {
            delay(1);
        }

        driver.VACTUAL(stopValue);
        motorState = MotorState::stop;

        isFront = false;
        frontTime = 0;
        isError = false;
        digitalWrite(enPin, HIGH);
        _allDirectionLedsOff();
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

    // Auto load speed = low manual load speed
    const uint32_t autoVactual = mcpManualVactualTable[0];

    if (requestedState == MotorState::forward) {
        isFront = true;
    } else {
        isFront = false;
        frontTime = 0;
    }

    // Only skip if state matches AND we are not forcing an auto-speed refresh
    if (!forceAutoRefresh && requestedState == motorState) {
        return;
    }

    driver.VACTUAL(stopValue);

    switch (requestedState) {
        case MotorState::forward: {
            motorState = MotorState::forward;
            digitalWrite(enPin, LOW);
            _setMotorCurrent(currentNormalMa);
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
            _setMotorCurrent(currentNormalMa);
            driver.shaft(0);
            driver.VACTUAL(autoVactual);
            _setDirectionLeds(false, true);
            break;
        }
    }

    forceAutoRefresh = false;
}

void motorControl() {
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
        isFront = false;
        frontTime = 0;
        isError = false;
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

static void _handleSerial() {
    while (Serial.available() > 0) {
        const char c = static_cast<char>(Serial.read());
        serialBuffer += c;
    }

    if (serialBuffer.length() == 0) {
        return;
    }

    if (serialBuffer == "rt") {
        Serial.print("read timeout=");
        Serial.println(timeoutMs);
        serialBuffer = "";
        return;
    }

    if (serialBuffer == "ml") {
        _runMcpLedSequence();
        serialBuffer = "";
        return;
    }

    if (serialBuffer == "mi") {
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
        serialBuffer = "";
        return;
    }

    if (serialBuffer.startsWith("set")) {
        serialBuffer.remove(0, 3);
        const int64_t value = serialBuffer.toInt();

        if (value <= 0 || value > 0xFFFFFFFFLL) {
            Serial.println("Error: Invalid timeout value.");
            serialBuffer = "";
            return;
        }

        timeoutMs = static_cast<uint32_t>(value);
        EEPROM.put(eepromAddrTimeout, timeoutMs);
        Serial.print("set succeed! timeout=");
        Serial.println(timeoutMs);
        serialBuffer = "";
        return;
    }

    Serial.println(serialBuffer);
    Serial.println("command error!");
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
    if (timeoutMs == 0xFFFFFFFF || timeoutMs == 0) {
        timeoutMs = defaultTimeoutMs;
        EEPROM.put(eepromAddrTimeout, timeoutMs);
        Serial.println("EEPROM is empty");
    } else {
        Serial.print("read timeout: ");
        Serial.println(timeoutMs);
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
}

void bufferLoop() {
    if (millis() - lastStatusToggleMs >= 500) {
        lastStatusToggleMs = millis();
        statusBlinkState = !statusBlinkState;
    }

    readSensorState();

    if (debugEnabled) {
        bufferDebug();
    } else {
        motorControl();
        _updatePanelIndicators();
        _handleSerial();
    }
}

void timerItCallback() {
    if (isFront) {
        ++frontTime;
        if (frontTime > timeoutMs) {
            isError = true;
        }
    }
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
    Serial.println(mcpManualSpeedIndex);
    delay(300);
}