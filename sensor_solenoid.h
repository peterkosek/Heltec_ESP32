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

#define PULSE_THRESHOLD  10   // ← change this value to adjust your wake-up count for the reed
#define GPIO_SENSOR_PIN GPIO_NUM_17  // GPIO pin connected to the sensor for ulp
#define RTC_GPIO_SENSOR_PIN GPIO_SENSOR_PIN  //  RTCIO_CHANNEL_17 is 17

// Optional power rails
#define VDD_3V3         1
#define GND             0

// ADC (MCP3421) Settings
#define ADC_ADDR            0x68   // 7-bit address for MCP3421
#define ADC_CFG_SINGLESHOT  0x90   // 16-bit, single-shot start, PGA=1

#define CYCLE_TIME_VALVE_ON   600000    // 10 min in ms, cycle time with valve on

typedef struct __attribute__((packed)) {

    unsigned vlv_A_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
    unsigned vlv_B_on_remaining  : 8;  // 8 bits, max 1023, 10 min incriments
    unsigned vlv_A_status        : 1;  // 1 bit
    unsigned vlv_B_status        : 1;  // 1 bit
    unsigned vlv_A_off           : 1;  //  instruction to shut off vlv A
    unsigned vlv_B_off           : 1;  //  instruction to shut off vlv B
    unsigned unused_also         : 4;  //  extra bits to fill the 32 bit str
    unsigned unused              : 8;  // fill up to 32 bit word, packed as the last byte
} ValveData_t;

typedef union __attribute__((packed)){
    ValveData_t bits;
    uint8_t    raw[sizeof(ValveData_t)];
} ValvePacket_t;

// Shared buffers defined in sensor_solenoid.cpp
extern uint8_t aTxBuffer0[8];     // first soil moisture sensor message
extern uint8_t aTxBuffer1[8];     // second soil moisture sensor message
extern uint8_t sTempC[4];         // temperature data from soil probes
extern uint8_t sMoist[4];         // moisture data from soil probes
extern uint8_t wPres[2];         // raw pressure from adc, needs calibration and conversion
extern uint8_t aRx[10];           // RS-485 returned data buffer

// Function prototypes
void hardware_pins_init();
void controlValve(uint8_t valve_number, uint8_t status);
uint8_t bat_cap8();
void beginReedCount();      // call once to start counting
bool isReedCountDone();     // true after 3 s
uint16_t endReedCount();    // call once done to get count

uint16_t readMCP3421avg_cont();

void setPowerEnable(uint8_t powerState);
void RS485Sub(uint8_t depth);
void RS485Get();
bool readFrame(uint8_t depth, uint8_t header, int& outIdx);
