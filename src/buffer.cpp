#include "buffer.hpp"

/* static instance */
Buffer buffer;

/* ─────────────────────────── PUBLIC ────────────────────────── */
void Buffer::begin()
{
    Serial.begin(115200);
    Serial.dtr(false);

    Wire.begin();
    _initIo();

    pinMode(kHall1, INPUT);
    pinMode(kHall2, INPUT);
    pinMode(kHall3, INPUT);
    pinMode(kEndstop, INPUT);

    pinMode(kEnPin, OUTPUT);
    pinMode(kDirPin, OUTPUT);
    pinMode(kStepPin, OUTPUT);
    digitalWrite(kEnPin, LOW);

    _driver.beginSerial(9600);
    _driver.I_scale_analog(false);
    _driver.toff(5);
    _driver.rms_current(kCurrentIdle);
    _driver.microsteps(kMicroSteps);
    _driver.VACTUAL(0);
    _driver.en_spreadCycle(true);
    _driver.pwm_autoscale(true);

    EEPROM.get(0, _timeout);
    if (_timeout == 0 || _timeout == 0xFFFFFFFF) {
        _timeout = 30000;
        EEPROM.put(0, _timeout);
    }

    /* 1 kHz watchdog */
    _timer.pause();
    _timer.setPrescaleFactor(48);
    _timer.setOverflow(1000);
    _timer.attachInterrupt(Buffer::_isrThunk);
    _timer.resume();

    _blinkTimer = millis();
}

void Buffer::update()
{
    if (millis() - _blinkTimer >= 500) {
        _blinkTimer += 500;
        _toggleLed(McpPin::ledStatus);
    }

    _pollRstButton();
    _updateOverrideAndCancel();
    _updateSpeedTier();
    _showSpeedTier();
    _processDirectionKeys();
    _runOneClickJob();

    /* sample sensors */
    _s.hall1           = digitalRead(kHall3);
    _s.hall2           = digitalRead(kHall2);
    _s.hall3           = digitalRead(kHall1);
    _s.materialPresent = !digitalRead(kEndstop);
    _s.keyRev          = !_readKey(McpPin::swReverse);
    _s.keyFwd          = !_readKey(McpPin::swForward);

    _motorControl();
}

void Buffer::timerIsr()                   // called from thunk
{
    if (_isFront && ++_frontMs > _timeout) {
        _isError = true;
        _writeLed(McpPin::ledError, true);
    }
}

/* ─────────────────────────── PRIVATE ───────────────────────── */

void Buffer::_initIo()
{
    _io.begin_I2C(kMcpAddr);

    for (uint8_t p : {0,1,2,3,4,5,6})                 // GPA0-6 inputs
        _io.pinMode(p, INPUT_PULLUP);

    for (uint8_t p : {8,9,10,11,12,13,14,15,7}) {     // all outputs
        _io.pinMode(p, OUTPUT);
        _io.digitalWrite(p, HIGH);
    }
    _writeLed(McpPin::ledOverride, true);             // manual on boot
}

void Buffer::_writeLed(McpPin pin, bool lowActive)
{
    _io.digitalWrite(static_cast<uint8_t>(pin), lowActive ? LOW : HIGH);
}
void Buffer::_toggleLed(McpPin pin)
{
    uint8_t p = static_cast<uint8_t>(pin);
    _io.digitalWrite(p, !_io.digitalRead(p));
}
bool Buffer::_readKey(McpPin pin)
{
    return _io.digitalRead(static_cast<uint8_t>(pin));
}

/* ───── reset key ───── */
void Buffer::_pollRstButton()
{
    bool now = _readKey(McpPin::swRst);
    if (!now && _lastRst) {
        delay(5);
        if (!_readKey(McpPin::swRst)) {
            _writeLed(McpPin::ledOverride, true);
            delay(50);
            NVIC_SystemReset();
        }
    }
    _lastRst = now;
}

/* ───── override / cancel ───── */
void Buffer::_updateOverrideAndCancel()
{
    bool ov = _readKey(McpPin::swOverride);
    bool cn = _readKey(McpPin::swCancel);

    if (!ov && _lastOv) {
        _manualOverride = !_manualOverride;
        _writeLed(McpPin::ledOverride, _manualOverride);
        if (_manualOverride) _moveActive = false;
    }
    _lastOv = ov;

    if (!cn && _lastCn) {
        _moveActive = false;
        _driver.VACTUAL(0);
        digitalWrite(kEnPin, HIGH);
    }
    _lastCn = cn;
}

/* ───── speed ladder ───── */
void Buffer::_updateSpeedTier()
{
    bool inc = _readKey(McpPin::swSpeedInc);
    bool dec = _readKey(McpPin::swSpeedDec);

    if (!inc && _lastInc)
        _speedTier = static_cast<SpeedTier>((uint8_t(_speedTier)+1) %
                                            uint8_t(SpeedTier::Count));
    if (!dec && _lastDec)
        _speedTier = static_cast<SpeedTier>((uint8_t(_speedTier)+2) %
                                            uint8_t(SpeedTier::Count));

    _lastInc = inc;  _lastDec = dec;
}
void Buffer::_showSpeedTier()
{
    _writeLed(McpPin::ledSpeedLow,    _speedTier == SpeedTier::Low);
    _writeLed(McpPin::ledSpeedMedium, _speedTier == SpeedTier::Medium);
    _writeLed(McpPin::ledSpeedHigh,   _speedTier == SpeedTier::High);
}

/* ───── direction keys ───── */
void Buffer::_processDirectionKeys()
{
    bool fwd = _readKey(McpPin::swForward);
    bool rev = _readKey(McpPin::swReverse);

    if (_manualOverride) {                          /* jog mode */
        if (!fwd) {
            _setMotorCurrent(kCurrentBoost);
            _driver.shaft(1);                       // forward
            _driver.VACTUAL(speed::vTable[uint8_t(_speedTier)]);
            digitalWrite(kEnPin, LOW);
        } else if (!rev) {
            _setMotorCurrent(kCurrentBoost);
            _driver.shaft(0);                       // back
            _driver.VACTUAL(speed::vTable[uint8_t(_speedTier)]);
            digitalWrite(kEnPin, LOW);
        } else {
            _driver.VACTUAL(0);
            digitalWrite(kEnPin, HIGH);
        }
        _lastFwd = fwd; _lastRev = rev;
        return;
    }

    /* one-click mode */
    if (!_moveActive && !fwd && _lastFwd) {
        _moveDir     = MotorState::Forward;
        _moveActive  = true;
        _moveStopMs  = millis()+_timeForDistance(kLoadMm, speed::rpmMed);
    } else if (!_moveActive && !rev && _lastRev) {
        _moveDir     = MotorState::Back;
        _moveActive  = true;
        _moveStopMs  = millis()+_timeForDistance(kLoadMm, speed::rpmMed);
    }
    _lastFwd = fwd; _lastRev = rev;
}

/* ───── drive timed job ───── */
void Buffer::_runOneClickJob()
{
    if (!_moveActive) return;

    if (millis() >= _moveStopMs) {
        _driver.VACTUAL(0);
        digitalWrite(kEnPin, HIGH);
        _moveActive = false;
        return;
    }
    _driver.shaft(_moveDir == MotorState::Forward ? 1 : 0);
    _driver.VACTUAL(speed::vTable[uint8_t(_speedTier)]);
    digitalWrite(kEnPin, LOW);
}

/* ───── motor state-machine (hall sensors etc.) ───── */
void Buffer::_motorControl()
{
    /* run-out */
    if (!_s.materialPresent) {
        _driver.VACTUAL(0);
        digitalWrite(kEnPin, HIGH);
        _writeLed(McpPin::ledFilament, true);
        _writeLed(McpPin::ledForward,  false);
        _writeLed(McpPin::ledReverse,  false);
        _isFront = _isError = false; _frontMs = 0;
        _motorState = MotorState::Stop;
        return;
    }
    _writeLed(McpPin::ledFilament, false);

    if (_isError) {                                 // latched error
        _driver.VACTUAL(0);
        digitalWrite(kEnPin, HIGH);
        _writeLed(McpPin::ledError, true);
        return;
    }

    /* hall logic */
    if (_s.hall1)       { _motorState = MotorState::Forward; _isFront = true; }
    else if (_s.hall2)  { _motorState = MotorState::Stop;    _isFront = false; _frontMs = 0; }
    else if (_s.hall3)  { _motorState = MotorState::Back;    _isFront = false; _frontMs = 0; }

    if (_motorState == _lastMotorState) return;
    _lastMotorState = _motorState;

    switch (_motorState) {
    case MotorState::Forward:
        _writeLed(McpPin::ledForward, true);
        _writeLed(McpPin::ledReverse, false);
        digitalWrite(kEnPin, LOW);
        _setMotorCurrent(kCurrentIdle);
        _driver.shaft(1);
        _driver.VACTUAL(speed::vTable[uint8_t(_speedTier)]);
        break;
    case MotorState::Back:
        _writeLed(McpPin::ledForward, false);
        _writeLed(McpPin::ledReverse, true);
        digitalWrite(kEnPin, LOW);
        _setMotorCurrent(kCurrentIdle);
        _driver.shaft(0);
        _driver.VACTUAL(speed::vTable[uint8_t(_speedTier)]);
        break;
    case MotorState::Stop:
        _writeLed(McpPin::ledForward, false);
        _writeLed(McpPin::ledReverse, false);
        _driver.VACTUAL(0);
        digitalWrite(kEnPin, HIGH);
        break;
    }
}

void Buffer::_setMotorCurrent(uint16_t mA)
{
    if (mA == _cachedCurrent) return;
    _driver.rms_current(mA);
    _cachedCurrent = mA;
}

uint32_t Buffer::_timeForDistance(float mm, uint32_t rpm)
{
    float rev  = mm / kMmPerRev;
    float sec  = rev / (rpm / 60.0f);
    return uint32_t(sec * 1000.0f + 0.5f);
}

/* ───── static ISR trampoline ───── */
void Buffer::_isrThunk() { buffer.timerIsr(); }
