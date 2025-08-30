#include "esp32-hal-adc.h"
#include "sensor_solenoid.h"
#include "esp32s3/ulp.h"  //  this also includes ulp_common.h 
#include <HardwareSerial.h>

// Define your pin constants somewhere accessible:

#define RS485_SERIAL    Serial2
#define RS485_TX_ENABLE RS485_DE

CMCP3421 adc(0.1692);

uint8_t aTxBuffer0[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xcb }; // first soil moisture sensor message
uint8_t aTxBuffer1[8] = { 0x02, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xf8 }; //  second soil moisture message
uint8_t aTxBuffer3[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd };  //  for three metal proble soil moisture sensor
uint8_t aTxBuffer4[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0a };  //  for three metal proble soil moisture sensor
uint8_t aSoilSensShallow[] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09 };
uint8_t aSoilSensDeep[]    = { 0x02, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x3a };
// Query to read 2 input registers (pressure), from address 0x0001
const uint8_t depthQuery[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x0a, 0xc5, 0xcd };
uint8_t wPres[2] = {0x01, 0x02}; // raw uncalibrated adc output
uint8_t sTempC[4], sMoist[4];   //  for final data from soil probes
uint8_t aRx[10];    //  for rs-485 returned data from soil probes
uint8_t soilSensorOut[6];  //  for the two soil sensors including moisture, temp and pH
extern uint8_t bat_pct;
extern HardwareSerial Serial1;
extern uint32_t depthraw;

// Hardware pin setup
void hardware_pins_init(void) {
    // Set input pins
    pinMode(PIN_VE_CTL, OUTPUT);
    digitalWrite(PIN_VE_CTL, HIGH); // power to external functions of the MCU
    pinMode(PIN_REED_P, INPUT_PULLUP);
    pinMode(RS485_DE, OUTPUT);
    pinMode(ADC_CTL_PIN, OUTPUT);
    analogSetPinAttenuation(VBAT_READ_PIN,ADC_2_5db);  //  6db reduces voltage to 1/2 of input
    pinMode(VBAT_READ_PIN, INPUT);  // not needed
    //analogReadResolution(12); // 12 bit res
    // Set solenoid control pins as outputs, start is locked high Z state
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_IN3, OUTPUT);
    pinMode(PIN_IN4, OUTPUT);

    // Set EN_PWR as output, and start with this on
    pinMode(PIN_EN_SENSE_PWR, OUTPUT);
    digitalWrite(PIN_EN_SENSE_PWR, HIGH);
    // Set up UART pins if needed separately
    // (Typically UART driver will configure TX/RX pins automatically)

}

// Map valves 1→index 0, 2→index 1
static const uint8_t forwardPins[] = { PIN_IN1, PIN_IN3 };
static const uint8_t reversePins[] = { PIN_IN2, PIN_IN4 };
static const size_t valveCount = sizeof(forwardPins) / sizeof(forwardPins[0]);

void controlValve(uint8_t valve_number, uint8_t status) {
    if (valve_number < 0 || valve_number > 1) return;

    size_t idx = valve_number;
    uint8_t pinF = forwardPins[idx];
    uint8_t pinR = reversePins[idx];

    digitalWrite(pinF, status ? HIGH : LOW);
    digitalWrite(pinR, status ? LOW  : HIGH);

    delay(500);

    digitalWrite(pinF, LOW);
    digitalWrite(pinR, LOW);
}

// return a 8 bit '% charged' that is  proportional to the range of voltages from 3.5 to 4.1
uint8_t bat_cap8(){
  const int16_t mid = 850;
  const int16_t span = 150;  // 1000–850

  Serial.print("in bat_cap8()\n");
  digitalWrite(ADC_CTL_PIN, HIGH);
  delay(40);
  uint16_t raw = analogRead(VBAT_READ_PIN);
  Serial.printf("read bat_cap8() pin_adc_ctl HIGH = %u\n", raw);
  digitalWrite(ADC_CTL_PIN, LOW);

  int16_t diff = raw - mid;                // –∞…+∞
  if (diff < 0) diff = -diff;              // abs(diff)
  uint16_t udiff = (uint16_t)diff;
  // now scale 0…150 → 0…100
  uint16_t pct = uint16_t(diff * 100 / span);
  if (pct > 100) pct = 100;                // clamp

  RTC_SLOW_MEM[ULP_BAT_PCT] = pct;  //  ULP_BAT_PCT is defined in the .h
  return ((uint8_t)(pct * 2.55));   //  lorawan.cpp wants this full byte form
}

uint16_t readMCP3421avg_cont() {
  const uint8_t addr    = 0x68;
  const uint8_t cfgCont = 0b10011000; // start continuous 16-bit
  const uint8_t samples = 8;
  int32_t       sum     = 0;

  // begin continuous conversions
  Wire.beginTransmission(addr);
  Wire.write(cfgCont);
  Wire.endTransmission();

  for (uint8_t i = 0; i < samples; i++) {
    // wait for RDY=0
    delay(70);
    uint8_t msb, lsb, stat;
    do {
      Wire.requestFrom(addr, (uint8_t)3);
      msb  = Wire.read();
      lsb  = Wire.read();
      stat = Wire.read();
    } while (stat & 0x80);

    // assemble and accumulate
    int16_t raw = (int16_t)(msb << 8) | lsb;
    sum += raw;
  }

  // stop continuous by kicking a single-shot
  Wire.beginTransmission(addr);
  Wire.write(0b10000000);
  Wire.endTransmission();
  
  return (uint16_t)(sum / samples);
}

// Assume PIN_EN_SENSE_PWR was pinMode’d to OUTPUT in setup()
void setPowerEnable(uint8_t powerState) {
    digitalWrite(PIN_EN_SENSE_PWR, powerState ? HIGH : LOW);
}

void RS485Sub(uint8_t depth);

static const uint8_t* const buffers[] = { aTxBuffer0, aTxBuffer1 };
static const size_t      bufLens[] = { sizeof(aTxBuffer0), sizeof(aTxBuffer1) };

void RS485Sub(uint8_t depth) {
  if (depth > 1) return;              // guard
  digitalWrite(RS485_DE, HIGH);
  delay(50);

  Serial2.write(buffers[depth], bufLens[depth]);
  Serial2.flush();

  digitalWrite(RS485_DE, LOW);
}

bool readFrame(uint8_t depth, uint8_t header, int& outIdx) {
    constexpr uint8_t maxRetries = 3;
    for (uint8_t attempt = 0; attempt < maxRetries; attempt++) {
        RS485Sub(depth);
        memset(aRx, 0, sizeof(aRx));
        size_t len = Serial2.readBytes(aRx, sizeof(aRx));
        for (int i = 0; i + 3 < len; i++) {
            if (aRx[i] == header && aRx[i+1] == 0x03 && aRx[i+2] == 0x04) {
                outIdx = i;
                return true;
            }
        }
    }
    return false;
}

void RS485Get() {
    Serial2.setTimeout(250);
    int idx;
    for (uint8_t depth = 0; depth < 2; depth++) {
        uint8_t header = depth ? 0x02 : 0x01;
        if (!readFrame(depth, header, idx)) continue;

        uint8_t base = depth * 2;
        sTempC[base  ] = aRx[idx+1];
        sTempC[base+1] = aRx[idx+2];
        sMoist[base  ] = aRx[idx+3];
        sMoist[base+1] = aRx[idx+4];
    }
}

void initRS485(uint16_t baud) {
  RS485_SERIAL.begin(baud, SERIAL_8N1, 44, 43);  // RX=44, TX=43
  pinMode(RS485_TX_ENABLE, OUTPUT);
  digitalWrite(RS485_TX_ENABLE, LOW); // Listen by default
}

bool readDepthSensor(uint16_t &depthRaw) {
  const uint8_t expected_len = 7;
  const uint8_t max_retries = 7;

  for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
    while (RS485_SERIAL.available()) RS485_SERIAL.read();  // Clear buffer

    digitalWrite(RS485_TX_ENABLE, HIGH);
    delay(10); // Settling time
    RS485_SERIAL.write(aTxBuffer4, sizeof(aTxBuffer3));
    RS485_SERIAL.flush(true);
    delayMicroseconds(200);
    digitalWrite(RS485_TX_ENABLE, LOW);

    uint32_t start = millis();
    while (RS485_SERIAL.available() < expected_len && millis() - start < 200) {
      delay(1);
    }

    if (RS485_SERIAL.available() >= expected_len) {
      uint8_t response[expected_len];
      for (int i = 0; i < expected_len; i++) {
        response[i] = RS485_SERIAL.read();
      }

      uint16_t crc = modbusCRC(response, expected_len - 2);
      uint16_t received_crc = response[5] | (response[6] << 8);

      if (crc == received_crc && response[0] == 0x01 && response[1] == 0x03) {
        depthRaw = ((uint16_t)response[3] << 8) | response[4];
        return true;
      } else {
        Serial.printf("Bad CRC or header on attempt %u\n", attempt + 1);
      }
    } else {
      Serial.printf("Timeout on attempt %u\n", attempt + 1);
    }

    delay(50);  // Short retry delay
  }

  Serial.println("Failed to read valid deth sensor rs-485 response after retries");
  return false;
}

/* read soil sensor by RS-485, check CRC and fill global array 
    soilSensorOut with data
    call ith the number of  sensors in the node, but will work if one call with 2 has only one node
    as the second one will not reply and it wil report as zero.
    
*/

bool readSoilSensor(uint8_t sensNumber) {
  const uint8_t expected_len = 13;
  const uint8_t max_retries = 7;
  static const char *TAG = "Soil";
  
  for (uint8_t depth = 0; depth < sensNumber; ++depth) {
    soilSensorOut[depth*1 + 0] = 0;  // clear old data  -  moist
		soilSensorOut[depth*1 + 1] = 0;  // tempc
		soilSensorOut[depth*1 + 2] = 0;  // pH

    for (uint8_t attempt = 0; attempt < max_retries; ++attempt) {
      while (RS485_SERIAL.available()) RS485_SERIAL.read();  // Clear buffer

      digitalWrite(RS485_TX_ENABLE, HIGH);
      delay(10); // Settling time
      RS485_SERIAL.write((depth == 0) ? aSoilSensShallow : aSoilSensDeep, sizeof((depth == 0) ? aSoilSensShallow : aSoilSensDeep));
      ESP_LOGI(TAG, "aSoilSensShallow(len) %u; ", (unsigned)sizeof(aSoilSensShallow));
      RS485_SERIAL.flush(true);
      delayMicroseconds(200);
      digitalWrite(RS485_TX_ENABLE, LOW);

      uint32_t start = millis();
      while (RS485_SERIAL.available() < expected_len && millis() - start < 200) {
        delay(1);
      }

      if (RS485_SERIAL.available() >= expected_len) {
        uint8_t response[expected_len];
        for (int i = 0; i < expected_len; i++) {
          response[i] = RS485_SERIAL.read();
        }

        uint16_t crc = modbusCRC(response, expected_len - 2);
        uint16_t received_crc = response[11] | (response[12] << 8);

        if (crc == received_crc && response[0] == 0x01 && response[1] == 0x03) {
          soilSensorOut[depth*1 + 0] = (uint8_t)(((response[3] << 8) | response[4]) / 10);  // moist as a byte
		      soilSensorOut[depth*1 + 1] = (uint8_t)(((response[5] << 8) | response[6]) / 10);  // temp as a byte, bytes 7 and 8 are for EC, not on these sensors
		      soilSensorOut[depth*1 + 2] = (uint8_t)(((response[9] << 8) | response[10]));  // pH as a byte
          return true;
        } else {
          Serial.printf("Bad CRC or header on attempt %u\n", attempt + 1);
        }
      } else {
        Serial.printf("Timeout on attempt %u\n", attempt + 1);
      }

      delay(50);  // Short retry delay
    }  //  attempt loop
  }  //  depth loop
  
  Serial.println("Failed to read valid RS-485 response after retries");
  return false;
}  // of function

bool buildModbusRequest(uint8_t slaveAddr, uint16_t regStart, uint16_t regCount, uint8_t (&request)[8]) {
  request[0] = slaveAddr;
  request[1] = 0x03;  // Function code: Read Holding Registers
  request[2] = (regStart >> 8) & 0xFF;
  request[3] = regStart & 0xFF;
  request[4] = (regCount >> 8) & 0xFF;
  request[5] = regCount & 0xFF;

  uint16_t crc = modbusCRC(request, 6);
  request[6] = crc & 0xFF;         // CRC Lo
  request[7] = (crc >> 8) & 0xFF;  // CRC Hi

  return true;  // for chaining or logging if desired
}

uint16_t modbusCRC(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];

    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }

  return crc;  // LSB-first when sending over Modbus
}
