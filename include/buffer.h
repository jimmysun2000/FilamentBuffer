#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <TMCStepper.h>
#include <EEPROM.h>

/* =====  I²C port-expander wiring  =====
 *
 *            PORT-A (inputs except GPA7)         PORT-B (all outputs)
 * ─────────────────────────────────────────────────────────────────────────
 *  GPA0  SW_OVERRIDE               GPB0  LED_SPEED_LOW
 *  GPA1  SW_CANCEL                 GPB1  LED_FILAMENT
 *  GPA2  SW_SPEED_INC              GPB2  LED_STATUS
 *  GPA3  SW_RST                    GPB3  LED_ERROR
 *  GPA4  SW_FORWARD                GPB4  LED_REVERSE
 *  GPA5  SW_REVERSE                GPB5  LED_SPEED_MEDIUM
 *  GPA6  SW_SPEED_DEC              GPB6  LED_SPEED_HIGH
 *  GPA7  LED_OVERRIDE              GPB7  LED_FORWARD
 */
constexpr uint8_t MCP_ADDR{0x20};

enum McpPin : uint8_t {
    swOverridePin     = 0,
    swCancelPin       = 1,
    swSpeedIncPin     = 2,
    swRstPin          = 3,
    swForwardPin      = 4,
    swReversePin      = 5,
    swSpeedDecPin     = 6,

    ledOverridePin    = 7,

    ledSpeedLowPin    = 8,
    ledFilamentPin    = 9,
    ledStatusPin      = 10,
    ledErrorPin       = 11,
    ledReversePin     = 12,
    ledSpeedMediumPin = 13,
    ledSpeedHighPin   = 14,
    ledForwardPin     = 15
};

#define HALL1               PB2      // Hall Sensor 3
#define HALL2               PB3      // Hall Sensor 2
#define HALL3               PB4      // Hall Sensor 1
#define ENDSTOP_3           PB7      // Filament Detect

#define EN_PIN              PA6
#define DIR_PIN             PA7
#define STEP_PIN            PC13
#define UART                PB1

#define DRIVER_ADDRESS      0b00
#define R_SENSE             0.11f

#define Move_Divide_NUM	((int32_t)(64))	// Micro Stepping
#define STOP 				0			// Stop
#define CURRENT_NORMAL_MA   300   // steady-state current for 33 mm^3/s feed rate
// #define CURRENT_NORMAL_MA   900   // steady-state current for 66 mm^3/s feed rate
#define CURRENT_BUTTON_MA   900   // boost current while a key is held (66 mm^3/s)
#define WRITE_EN_PIN(x) digitalWrite(EN_PIN,x)// Enable Pin Write
#define FORWARD				1			// Filament Direction
#define BACK				0

enum class SpeedTier : uint8_t {Low = 0, Medium, High, Count};

namespace speed {
    /* delivered filament-flow (mm³ s⁻¹)  →  mechanical rpm */
    constexpr uint32_t rpmLow    = (33/0.22);   //  ≈ 33 mm³ s⁻¹
    constexpr uint32_t rpmMed    = (50/0.22);   //  ≈ 50 mm³ s⁻¹
    constexpr uint32_t rpmHigh   = (66/0.22);   //  ≈ 66 mm³ s⁻¹

    /* TMC2209 VACTUAL = rpm · (µstep · 200) / (60 · 0.715)            *
     * with µstep = 64   →   scale ≈ 298.692                           */
    constexpr float    _k       = (Move_Divide_NUM * 200.0f) / (60.0f * 0.715f);

    constexpr uint32_t vLow     = uint32_t(rpmLow  * _k + 0.5f);
    constexpr uint32_t vMed     = uint32_t(rpmMed  * _k + 0.5f);
    constexpr uint32_t vHigh    = uint32_t(rpmHigh * _k + 0.5f);

    constexpr uint32_t vTable[static_cast<uint8_t>(SpeedTier::Count)] = {vLow, vMed, vHigh};
}

extern SpeedTier currentSpeedTier;

/* distance per click [millimetres] in normal mode */
constexpr float kLoadDistanceMm{120.0f};
/* motor micro-steps per full rev */
constexpr uint32_t kMicroSteps{Move_Divide_NUM * 200};
/* filament displacement per full rev [mm]  (≈ π·Ødrive-gear) */
constexpr float kMmPerRev{22.0f};
/* derived run-time (ms) = D / v  */
constexpr uint32_t _timeForDistance(float mm, uint32_t rpm) {
    const float rev  = mm / kMmPerRev;
    const float sec  = rev / (rpm / 60.0f);
    return uint32_t(sec * 1000.0f + 0.5f);
}

// Input states
struct BufferState {
    bool hallPos1;
    bool hallPos2;
    bool hallPos3;
    bool materialPresent;
    bool keyReverse;
    bool keyForward;
};

// Output stepper states
typedef enum {
	Forward = 0,	// Forward
	Stop,			// Stop
	Back			// Reverse
} Motor_State;

extern void motorControl(void);
extern void bufferInit();
extern void bufferLoop();
extern void _timerInterruptHandler();
extern void _bufferDebug();
extern void _pollRstButton();

#endif