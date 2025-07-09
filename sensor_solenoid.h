#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include <MCP3421.h>

// GPIO Pin Assignments
#define PIN_SDA         39   // P4.2
#define PIN_SCL         38   // P4.3
#define PIN_RS485_RX    44  // P4.6 (U0RXD)
#define PIN_RS485_TX    43  // P4.8 (U0TXD)
#define RS485_DE        40  //  
#define PIN_IN1         48  // P4.25
#define PIN_IN2         47  // P4.26
#define PIN_IN3         41  // P4.28
#define PIN_IN4         42  // P4.27
#define PIN_VE_CTL      18  //  enable power for second LDR for external loads on the MCU board
#define PIN_EN_SENSE_PWR      45 // P4.34   for power on the sensor board, also LED on the vision master board
#define PIN_REED_P      17  // P4.38
#define ADC_CTL_PIN     46  //
#define VBAT_READ_PIN   7   // read voltge divider with 100k/490k divider.  

#define PULSE_THRESHOLD  10   // default, but MVS stores last uint16_t sent to port 8
                              // change this value to adjust the default wake-up count for the reed
#define RTC_GPIO_SENSOR_PIN GPIO_NUM_17  // GPIO pin connected to the sensor for ulp
#define RTC_GPIO_INDEX 17  //  RTCIO_CHANNEL_17 is 17

// Optional power rails
#define VDD_3V3         1
#define GND             0

// For the reed flow meter
#define TICKS_PER_MIN   209354u      //  based on   I_DELAY(0x13dc), 3500 Hz   //  integer math please
#define VOLUME_PER_TICK 100u         //  whatever the volume per tick of the meter is displayed as lpm

// ADC (MCP3421) Settings
#define ADC_ADDR            0x68   // 7-bit address for MCP3421
#define ADC_CFG_SINGLESHOT  0x90   // 16-bit, single-shot start, PGA=1

#define CYCLE_TIME_VALVE_ON   600000    // 10 min in ms, cycle time with valve on

// typedef struct __attribute__((packed)) {

//     unsigned vlv_A_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
//     unsigned vlv_B_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
//     unsigned vlv_A_status        : 1;  // 1 bit
//     unsigned vlv_B_status        : 1;  // 1 bit
//     unsigned vlv_A_off           : 1;  //  instruction to shut off vlv A
//     unsigned vlv_B_off           : 1;  //  instruction to shut off vlv B
//     unsigned unused_also         : 4;  //  extra bits to fill the 32 bit str
//     unsigned unused              : 8;  // fill up to 32 bit word, packed as the last byte
// } ValveData_t;

// typedef union __attribute__((packed)){
//     ValveData_t bits;
//     uint8_t    raw[sizeof(ValveData_t)];
// } ValvePacket_t;

typedef struct __attribute__((packed)) {
    uint8_t time;       // 10-minute countdown
    union {
        struct {
            uint8_t onA : 1;  // on/off state
            uint8_t onB : 1;  // on/off state
            uint8_t onC : 1;  // on/off state
            uint8_t onD : 1;  // on/off state
            uint8_t offA    : 1;  // pending shutdown
            uint8_t offB    : 1;  // pending shutdown
            uint8_t offC    : 1;  // pending shutdown
            uint8_t offD    : 1;  // pending shutdown
        };
        uint8_t flags;
    };
} ValveState_t;

enum {
  // count up each RTC_DATA_ATTR 16-bit word…
  ULP_RSSI,            // 0
  ULP_SNR,             // 1
  ULP_BAT_PCT,         // 2
  ULP_LAST_SENT,       // 3
  ULP_COUNT,           // 4
  ULP_PREV_STATE,      // 5
  ULP_VALVE_A,         // 6  (valve a status)
  ULP_VALVE_B,         // 7  (uvalve b status)
  ULP_DEBUG_PIN_STATE, // 8
  ULP_TICK_POP,          // 9   
  ULP_TS_DELTA_LO,      // 10
  ULP_TS_DELTA_HI,      // 11
  ULP_TS_DELTA_TICK_POP,   // 12
  ULP_TIMER_LO,        // 13 ← low 16 bits of running counter
  ULP_TIMER_HI,        // 14 ← high 16 bits
  ULP_REED_DELTA,       // 15 last reed delta
  ULP_FLOW_RATE,        // 16  calculated flow rate, used for display
  ULP_VOLUME_DELTA,             // 17  flow since last LoraWAN frame, in liters (reed_delta * vol_per_reed)
  ULP_WAKE_THRESHOLD,       //  18 
  ULP_PROG_START = 19,   // load instructions here
};

enum {
  ULP_ENTRY_LABEL   = 0,
  ULP_SKIP_HI_INC   = 1,
  ULP_SET_OVF   = 2,
  SKIP_HI_INC    = 3,
  ULP_STORE_PREV  = 4,
  ULP_NO_EDGE       = 5,
  ULP_WRAP_DONE       = 6,
  ULP_NO_WRAP       = 7,
  ULP_WRAP_CHECK_LABEL = 8,
  ULP_NO_WAKE   = 9,
  ULP_HI_INC_LABEL     = 10,
  ULP_NO_WRAP_LABEL    = 11,
  ULP_STORE_CUR = 12,
  ULP_EXIT_LABEL = 13,
  ULP_INC_LABEL        = 14,
  ULP_SET_TICK_POP,
  ULP_NO_TIMER_WRAP,
};

// Shared buffers defined in sensor_solenoid.cpp
extern uint8_t aTxBuffer0[8];     // first soil moisture sensor message
extern uint8_t aTxBuffer1[8];     // second soil moisture sensor message
extern uint8_t sTempC[4];         // temperature data from soil probes
extern uint8_t sMoist[4];         // moisture data from soil probes
extern uint8_t wPres[2];         // raw pressure from adc, needs calibration and conversion
extern uint8_t aRx[10];           // RS-485 returned data buffer
extern uint8_t reedCount[2];  // reed pulses counted, MSB, LSB
extern uint8_t reedcyclesTenMin[2]; // intra reed pulse converted to frequency in activations in 10 min, MSB, LSB

// Function prototypes
void hardware_pins_init();
void controlValve(uint8_t valve_number, uint8_t status);
uint8_t bat_cap8();

uint16_t readMCP3421avg_cont();

void setPowerEnable(uint8_t powerState);
void RS485Sub(uint8_t depth);
void RS485Get();
bool readFrame(uint8_t depth, uint8_t header, int& outIdx);

