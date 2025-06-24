#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <TMCStepper.h>
#include <EEPROM.h>

/* ───── port-expander wiring ───── */
enum class McpPin : uint8_t {
    swOverride = 0, swCancel,  swSpeedInc, swRst,
    swForward,  swReverse, swSpeedDec,
    ledOverride,                             // GPA7

    ledSpeedLow = 8,  ledFilament, ledStatus, ledError,
    ledReverse,     ledSpeedMedium, ledSpeedHigh, ledForward
};

constexpr uint8_t kMcpAddr{0x20};

/* ───── local MCU pins ───── */
constexpr uint8_t kHall1{PB2};
constexpr uint8_t kHall2{PB3};
constexpr uint8_t kHall3{PB4};
constexpr uint8_t kEndstop{PB7};

constexpr uint8_t kEnPin {PA6};
constexpr uint8_t kDirPin{PA7};
constexpr uint8_t kStepPin{PC13};
constexpr uint8_t kUartPin{PB1};

/* ───── stepper / motion constants ───── */
constexpr uint8_t  kDriverAddr {0b00};
constexpr float    kRsense      = 0.11f;
constexpr int32_t  kMicroSteps  = 64;
constexpr uint16_t kCurrentIdle = 300;   // mA
constexpr uint16_t kCurrentBoost= 900;   // mA

enum class SpeedTier : uint8_t {Low = 0, Medium, High, Count};

namespace speed {
    constexpr uint32_t rpmLow  {static_cast<uint32_t>(33 / 0.22f)};
    constexpr uint32_t rpmMed  {static_cast<uint32_t>(50 / 0.22f)};
    constexpr uint32_t rpmHigh {static_cast<uint32_t>(66 / 0.22f)};

    constexpr float _k  = (kMicroSteps * 200.0f) / (60.0f * 0.715f);

    constexpr uint32_t vLow  {uint32_t(rpmLow  * _k + 0.5f)};
    constexpr uint32_t vMed  {uint32_t(rpmMed  * _k + 0.5f)};
    constexpr uint32_t vHigh {uint32_t(rpmHigh * _k + 0.5f)};

    constexpr uint32_t vTable[static_cast<uint8_t>(SpeedTier::Count)] =
        {vLow, vMed, vHigh};
}

/* ───── one-click travel constants ───── */
constexpr float    kLoadMm     = 120.0f;
constexpr float    kMmPerRev   = 22.0f;                             // ≈ π·Øgear
constexpr uint32_t kMicroStepPerRev = kMicroSteps * 200;

class Buffer
{
public:
    void begin();
    void update();
    void timerIsr();

private:
    /* hardware helpers */
    void _initIo();
    void _writeLed(McpPin pin, bool lowActive);
    void _toggleLed(McpPin pin);
    bool _readKey(McpPin pin);

    /* UI / control layers */
    void _pollRstButton();
    void _updateOverrideAndCancel();
    void _updateSpeedTier();
    void _showSpeedTier();
    void _processDirectionKeys();
    void _runOneClickJob();

    /* motor layer */
    void _setMotorCurrent(uint16_t mA);
    void _motorControl();                      // original state-machine

    /* utility */
    static uint32_t _timeForDistance(float mm, uint32_t rpm);

    /* ───── state ───── */
    enum class MotorState : uint8_t {Forward, Stop, Back};

    struct Sensors {
        bool hall1{}, hall2{}, hall3{};
        bool materialPresent{};
        bool keyFwd{}, keyRev{};
    };

    /* hardware */
    Adafruit_MCP23X17 _io;
    TMC2209Stepper    _driver{ kUartPin, kUartPin, kRsense, kDriverAddr };
    HardwareTimer     _timer   {TIM6};

    /* run-time variables */
    Sensors     _s;
    MotorState  _motorState{MotorState::Stop};
    MotorState  _lastMotorState{MotorState::Stop};

    bool        _manualOverride{true};
    bool        _moveActive{false};
    MotorState  _moveDir{MotorState::Stop};
    uint32_t    _moveStopMs{0};

    SpeedTier   _speedTier{SpeedTier::Medium};
    uint16_t    _cachedCurrent{kCurrentIdle};

    bool        _isFront{false};
    bool        _isError{false};
    uint32_t    _frontMs{0};
    uint32_t    _timeout{60000};

    /* edge detectors */
    bool _lastRst{true}, _lastOv{true}, _lastCn{true},
         _lastFwd{true}, _lastRev{true}, _lastInc{true}, _lastDec{true};

    uint32_t _blinkTimer{0};

    /* static ISR trampoline */
    static void _isrThunk();
};

extern Buffer buffer;

#endif // BUFFER_HPP
