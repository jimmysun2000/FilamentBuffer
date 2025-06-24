#include "buffer.h"

Adafruit_MCP23X17 _io;
TMC2209Stepper driver(UART, UART, R_SENSE, DRIVER_ADDRESS);
HardwareTimer      _errorTimer{TIM6};
SpeedTier currentSpeedTier{SpeedTier::Medium};
static bool _lastInc{true}, _lastDec{true};

static BufferState   _buf{};
static Motor_State   _motorState{Stop};
static Motor_State   _lastMotorState{Stop};
static bool          _isFront{false};
static bool          _isError{false};
static uint32_t      _frontTime{0};
static uint32_t      _timeout{60000};
static uint16_t      _currentCached{CURRENT_NORMAL_MA};
static String        _serialBuffer;
static uint32_t 	 _lastBlink;
inline void _writeLed(uint8_t pin, bool level)       { _io.digitalWrite(pin, level); }
inline void _toggleLed(uint8_t pin)                  { _io.digitalWrite(pin, !_io.digitalRead(pin)); }
inline bool _readKey(uint8_t pin)                    { return _io.digitalRead(pin); }

static void _updateSpeedTier() {
    /* active-low buttons */
    bool inc = _readKey(swSpeedIncPin);
    bool dec = _readKey(swSpeedDecPin);

    /* rising-edge on either key? */
    if (!inc && _lastInc) {
        currentSpeedTier = static_cast<SpeedTier>(
            (static_cast<uint8_t>(currentSpeedTier) + 1) %
            static_cast<uint8_t>(SpeedTier::Count));
    }
    if (!dec && _lastDec) {
        currentSpeedTier = static_cast<SpeedTier>(
            (static_cast<uint8_t>(currentSpeedTier) + 2) %
            static_cast<uint8_t>(SpeedTier::Count));    // -1 mod 3
    }
    _lastInc = inc;
    _lastDec = dec;
}

/* refresh the three indicator LEDs */
static void _showSpeedTier() {
    _writeLed(ledSpeedLowPin,    currentSpeedTier == SpeedTier::Low    ? LOW : HIGH);
    _writeLed(ledSpeedMediumPin, currentSpeedTier == SpeedTier::Medium ? LOW : HIGH);
    _writeLed(ledSpeedHighPin,   currentSpeedTier == SpeedTier::High   ? LOW : HIGH);
}

static void _initIoExpander() {
    _io.begin_I2C(MCP_ADDR);

    /* inputs with pull-ups */
    for (uint8_t p : {swOverridePin, swCancelPin, swSpeedIncPin,
                      swRstPin, swForwardPin, swReversePin, swSpeedDecPin}) {
        _io.pinMode(p, INPUT_PULLUP);
    }

    /* outputs – default off (HIGH) */
    for (uint8_t p : {ledOverridePin, ledSpeedLowPin, ledFilamentPin, ledStatusPin,
                      ledErrorPin, ledReversePin, ledSpeedMediumPin,
                      ledSpeedHighPin, ledForwardPin}) {
        _io.pinMode(p, OUTPUT);
        _io.digitalWrite(p, HIGH);
    }
}

void bufferInit() {
    Wire.begin();
    _initIoExpander();

    /* local sensors */
    pinMode(HALL1, INPUT);
    pinMode(HALL2, INPUT);
    pinMode(HALL3, INPUT);
    pinMode(ENDSTOP_3, INPUT);

    /* stepper outputs */
    pinMode(EN_PIN,  OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN,OUTPUT);
    digitalWrite(EN_PIN, LOW);
    driver.beginSerial(9600);
    driver.I_scale_analog(false);
    driver.toff(5);
    driver.rms_current(CURRENT_NORMAL_MA);
    driver.microsteps(Move_Divide_NUM);
	driver.VACTUAL(STOP);
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(true);

	delay(1000);

    /* read timeout from EEPROM */
    EEPROM.get(0, _timeout);
    if (_timeout == 0 || _timeout == 0xFFFFFFFF) {
        _timeout = 30000;
        EEPROM.put(0, _timeout);
		Serial.println("EEPROM is empty");
	} else {
		Serial.print("read timeout: ");
		Serial.println(_timeout);
	}

    /* 1 kHz timer for error watchdog */
    _errorTimer.pause();
    _errorTimer.setPrescaleFactor(48);
    _errorTimer.setOverflow(1000);
    _errorTimer.attachInterrupt(_timerInterruptHandler);
    _errorTimer.resume();

    _lastBlink = millis();
}

static inline void _setMotorCurrent(uint16_t mA) {
    if (mA != _currentCached) {
        driver.rms_current(mA);
        _currentCached = mA;
	}
}

void bufferLoop() {
	/* heartbeat every 500 ms */
	if (millis() - _lastBlink >= 500) {
		_lastBlink = millis();
		_toggleLed(ledStatusPin);
	}

	/* poll speed keys & set speed leds */
	_updateSpeedTier();
	_showSpeedTier();

	_buf.hallPos1        = digitalRead(HALL3);
	_buf.hallPos2        = digitalRead(HALL2);
	_buf.hallPos3        = digitalRead(HALL1);
	_buf.materialPresent = !digitalRead(ENDSTOP_3);
	_buf.keyReverse      = !_readKey(swReversePin);
	_buf.keyForward      = !_readKey(swForwardPin);

        motorControl();
}

void motorControl(void) {
    /* Reverse button */
    if (_buf.keyReverse) {
        _writeLed(ledForwardPin, HIGH);
        _writeLed(ledReversePin, LOW);
		WRITE_EN_PIN(0); 		// Enable stepper
		driver.VACTUAL(STOP);	// Stop
		_setMotorCurrent(CURRENT_BUTTON_MA);      // boost current

		driver.shaft(BACK);
		driver.VACTUAL(speed::vTable[static_cast<uint8_t>(currentSpeedTier)]);
		while(_buf.keyReverse); // Wait for button to be released
					
		driver.VACTUAL(STOP);	// Stop
		_motorState = Stop;

		_isFront = false;
		_frontTime = 0;
		_isError = false;
		WRITE_EN_PIN(1); 		// Disable stepper
        _writeLed(ledReversePin, HIGH);
	}
    /* Forward button */
    else if (_buf.keyForward) {
        _writeLed(ledForwardPin, LOW);
        _writeLed(ledReversePin, HIGH);
		WRITE_EN_PIN(0);
		driver.VACTUAL(STOP);
		_setMotorCurrent(CURRENT_BUTTON_MA);      // boost current

    	driver.shaft(FORWARD);
		driver.VACTUAL(speed::vTable[static_cast<uint8_t>(currentSpeedTier)]);
		while(_buf.keyForward);
					
		driver.VACTUAL(STOP);
		_motorState = Stop;

		_isFront = false;
		_frontTime = 0;
		_isError = false;
		WRITE_EN_PIN(1);
        _writeLed(ledForwardPin, HIGH);
	}
    /* material run-out */
    if (!_buf.materialPresent) {
		// Filament run out, stop stepper
		driver.VACTUAL(STOP);
		_motorState = Stop;
		_isFront = false;
		_frontTime = 0;
		_isError = false;
		WRITE_EN_PIN(1);

		_writeLed(ledFilamentPin, HIGH);
        _writeLed(ledForwardPin, HIGH);
        _writeLed(ledReversePin, HIGH);
        return;
    }

	_writeLed(ledFilamentPin, LOW);

    /* error state */
    if (_isError) {
        _writeLed(ledErrorPin, LOW);
		driver.VACTUAL(STOP);
		_motorState = Stop;
		WRITE_EN_PIN(1);
        _writeLed(ledForwardPin, HIGH);
        _writeLed(ledReversePin, HIGH);     
		return;
    }

	// Buffer location detection
	if (_buf.hallPos1) {	//缓冲器位置为1，耗材往前推
		_lastMotorState = _motorState;		//记录上一次状态
		_motorState = Forward;
		_isFront = true;
	}
	else if (_buf.hallPos2) {	//缓冲器位置为2,电机停止转动
		_lastMotorState = _motorState;		//记录上一次状态
		_motorState = Stop;
		_isFront = false;
		_frontTime = 0;
	}
	else if(_buf.hallPos3) {	//缓冲器位置为3，回退耗材
		_lastMotorState = _motorState;		//记录上一次状态
		_motorState = Back;
		_isFront = false;
		_frontTime = 0;
	}
			
	if (_motorState == _lastMotorState) { //如果上次状态跟这次状态一致，则不需要再次发送控制命令,结束此次函数
		return;
	}

	//电机控制
	switch(_motorState) {
		case Forward://向前
		{
        	_writeLed(ledErrorPin, HIGH);
        	_writeLed(ledForwardPin, LOW);
			WRITE_EN_PIN(0);
			if (_lastMotorState == Back) {
				driver.VACTUAL(STOP);//上次是后退，先停下再前进
			}
			_setMotorCurrent(CURRENT_NORMAL_MA);      // steady current
			driver.shaft(FORWARD);
			driver.VACTUAL(speed::vTable[static_cast<uint8_t>(currentSpeedTier)]);

		} break;
		case Stop://停止
		{
        	_writeLed(ledForwardPin, HIGH);
        	_writeLed(ledReversePin, HIGH);
			WRITE_EN_PIN(1);
			driver.VACTUAL(STOP);

		} break;
		case Back://向后
		{
        	_writeLed(ledErrorPin, HIGH);
        	_writeLed(ledReversePin, LOW);
			WRITE_EN_PIN(0);
			if (_lastMotorState == Forward) {
				driver.VACTUAL(STOP);//上次是前进，先停下再后退
			}
			_setMotorCurrent(CURRENT_NORMAL_MA);      // steady current
			driver.shaft(BACK);
			driver.VACTUAL(speed::vTable[static_cast<uint8_t>(currentSpeedTier)]);
		} break;
	}
}

void _timerInterruptHandler() {
    if (_isFront && ++_frontTime > _timeout) {
        _isError = true;
        _writeLed(ledErrorPin, LOW);
    }
}
