#include "esp32-hal-adc.h"
#include "sensor_solenoid.h"

// Define your pin constants somewhere accessible:

volatile uint16_t   reedCount = 0;
static uint32_t     reedStart;
CMCP3421 adc(0.1692);

uint8_t aTxBuffer0[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xcb }; // first soil moisture sensor message
uint8_t aTxBuffer1[8] = { 0x02, 0x03, 0x00, 0x01, 0x00, 0x02, 0x95, 0xf8 }; //  second soil moisture message
uint8_t wPres[2] = {0x01, 0x02}; // raw uncalibrated adc output
uint8_t sTempC[4], sMoist[4];   //  for final data from soil probes
uint8_t aRx[10];    //  for rs-485 returned data from soil probes
extern uint8_t bat_pct;

// Hardware pin setup
void hardware_pins_init(void) {
    // Set input pins
    pinMode(PIN_VE_CTL, OUTPUT);
    digitalWrite(PIN_VE_CTL, HIGH); // fower to external functions of the MCU
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

    // Set EN_PWR as output, and start with this off
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

    delay(250);

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

  bat_pct = pct;  //  for RTC we are using only uint16_t vars
  return ((uint8_t)pct);
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

static void IRAM_ATTR onReedFall() {
  reedCount++;
}

void beginReedCount() {
  reedCount   = 0;
  reedStart   = millis();
  pinMode(PIN_REED_P, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_REED_P), onReedFall, FALLING);
}

bool isReedCountDone() {
  return (millis() - reedStart) >= 3000;
}

uint16_t endReedCount() {
  detachInterrupt(digitalPinToInterrupt(PIN_REED_P));
  return reedCount;
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
