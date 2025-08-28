/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. Accept and display messages on the screen
 * 3. Calibrate the time from the network and then use an internal crystal oscillator to determine the time
 * 4. Read sensor data and upload it to the server. Receive server messages and display them on the screen.
 * 
 * If you do not fill in the correct WIFI SSID and password, the E-Ink will not display.
 * 
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * 
 * Library url: https://github.com/HelTecAutomation/Heltec_ESP32
 * Support: support@heltec.cn
 *
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * https://www.heltec.org
 * */

#include <Arduino.h>
#include "LoRaWan_APP.h"
//#include "LoRaMacCommands.h"
#include "Wire.h"
//#include "GXHTC.h"
#include "img.h"
#include "HT_DEPG0290BxS800FxX_BW.h"
#include "sensor_solenoid.h"
#include "esp_sleep.h"
#include "esp32s3/ulp.h"  //  this also includes ulp_common.h 
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include <Preferences.h>
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"   // for RTC_CNTL_RTC_UPDATE_REG
#include "soc/soc.h"
#include "soc/sens_struct.h"
#include "soc/sens_reg.h"
#include <Arduino.h>
#include "Adafruit_SHT4x.h"
#include "esp_heap_caps.h"


// 2) A handy macro to get a pointer to any struct at a given word‐index:
// makes rtc vars volatile
#define RTC_SLOW_BYTE_MEM   ((uint8_t*)SOC_RTC_DATA_LOW)
#define RTC_SLOW_MEMORY ((volatile uint32_t*) SOC_RTC_DATA_LOW)
#define RTC_SLOW_STRUCT_PTR(type, idx) \
    ((volatile type*)(RTC_SLOW_BYTE_MEM + (idx) * sizeof(uint32_t)))

// 3) Now you can declare C-pointers into that region:
volatile ValveState_t* valveA = RTC_SLOW_STRUCT_PTR(ValveState_t, ULP_VALVE_A);
volatile ValveState_t* valveB = RTC_SLOW_STRUCT_PTR(ValveState_t, ULP_VALVE_B);
RTC_DATA_ATTR char g_name[13] = "no name";  // screen display name
RTC_DATA_ATTR uint32_t epd_hygiene = 0;  //    to reset the screen ever 50 wakes

DEPG0290BxS800FxX_BW display(5, 4, 3, 6, 2, 1, -1, 6000000);  // rst,dc,cs,busy,sck,mosi,miso,frequency
//GXHTC gxhtc;
Preferences prefs;  // for NVM
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
// for display
volatile bool g_need_display = false;

// [GPT] helper: wait for E‑Ink BUSY to go idle without blocking the whole system
static bool eink_wait_idle(uint32_t timeout_ms) {
  const bool BUSY_ACTIVE  = HIGH;  // DEPG0290 panels pull BUSY high while refreshing
  uint32_t deadline = millis() + timeout_ms;
  while (digitalRead(EPD_BUSY_PIN) == BUSY_ACTIVE) {
    vTaskDelay(pdMS_TO_TICKS(10));  // yield to other tasks (LoRa, etc.)
    if ((int32_t)(millis() - deadline) >= 0) return false;  // timed out
  }
  return true;
}


char buffer[64];
ValveState_t vlv_packet_pend;  // used to keep the command and the state independent until resolved

#define REED_NODE       true
//#define VALVE_NODE      true
//#define SOIL_SENSOR_NODE true
//#define LAKE_NODE  true

/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xef };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = {0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x46 };

/* ABP para --  not used for this project*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels US915 band 2 hybrid*/
uint16_t userChannelsMask[6] = { 0xff00, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
RTC_DATA_ATTR uint32_t appTxDutyCycle = 360 * 60 * 1000;
RTC_DATA_ATTR uint32_t TxDutyCycle_hold;
RTC_DATA_ATTR uint32_t initialCycleFast = 6;        //  number of time to cycle fast on startup
static const uint32_t TX_CYCLE_FAST_TIME = 30000ul;
//These are in RTC defined in lorawanapp.cpp
extern int revrssi;
extern int revsnr;


/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
#ifdef REED_NODE
uint8_t appPort = 8;          //  REED_NODE port 8
#elif defined VALVE_NODE
uint8_t appPort = 9;          // VALVE_NODE port 9
#elif defined SOIL_SENSOR_NODE
uint8_t appPort = 6;          // SOIL_SENSOR_NODE port 6
#elif defined LAKE_NODE
uint8_t appPort = 12;          // LAKE_NODE port 12
#else
#error "Define a node type in the list REED_NODE, VALVE_NODE, SOIL_SENSOR_NODE, LAKE_NODE"
#endif

/*!
 * Number of trials to transmit the frame, if the LoRaMAC layer did not
 * receive an acknowledgment. The MAC performs a datarate adaptation,
 * according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
 * to the following table:
 *
 * Transmission nb | Data Rate
 * ----------------|-----------
 * 1 (first)       | DR
 * 2               | DR
 * 3               | max(DR-1,0)
 * 4               | max(DR-1,0)
 * 5               | max(DR-2,0)
 * 6               | max(DR-2,0)
 * 7               | max(DR-3,0)
 * 8               | max(DR-3,0)
 *
 * Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
 * the datarate, in case the LoRaMAC layer did not receive an acknowledgment
 */
uint8_t confirmedNbTrials = 4;

static inline uint32_t read_count32(){
  uint16_t hi1=(uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_HI];
  uint16_t lo =(uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO];
  uint16_t hi2=(uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_HI];
  if(hi1!=hi2){ lo=(uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO]; hi1=hi2; }
  return ((uint32_t)hi1<<16)|lo;
}
// Function to display the binary representation of an integer
void displayBits16(uint16_t v) {
  for (int i = 15; i >= 0; --i) {
    Serial.print((v >> i) & 1);
  }
  Serial.printf("\nvalve A time %d status %d off %d\n", (int)valveA->time, (int)valveA->onA, (int)valveA->offA);
  Serial.printf("valve B time %d status %d off %d\n", (int)valveB->time, (int)valveB->onB, (int)valveB->offB);
  Serial.printf("pending time %d status %d:%d off %d:%d\n", (int)vlv_packet_pend.time, (int)vlv_packet_pend.onA, (int)vlv_packet_pend.onB, (int)vlv_packet_pend.offA, (int)vlv_packet_pend.offB);
   //Serial.printf("vbat read %d \n", bat_cap8());
}

void displayPacketBits(const ValveState_t &p) {
  uint16_t v = 0;
  memcpy(&v, &p, sizeof(v));  // pull the raw bytes straight into a uint16_t
  displayBits16(v);
}

//  display status draws the screen before sleep, after a delay from uploading to receive any download
static void show_vlv_status(uint8_t vlv) {
  switch (vlv) {
    case 0:
      if (valveA->onA) {
        snprintf(buffer, sizeof(buffer), "%u min\n", (unsigned)(valveA->time * 10));
      } else {
        snprintf(buffer, sizeof(buffer),  "A off");
      }
      break;
    case 1:
      if (valveB->onB) {
        snprintf(buffer, sizeof(buffer), "%u min\n", (unsigned)(valveB->time * 10));
      } else {
        snprintf(buffer, sizeof(buffer),  "B off");
      }
      break;
  }
}

/*
POPULATE THE DATA FIELDS OF PRESSURE, COUNT DELTA AND TIMER DELTA FOR THE LAST REED COUNT

wPress[2]
reedCount[2]
ticksDelat[2]
ULP_BAT_PCT (uint8_t)

*/
void pop_data(void){



  //  BATTERY PERCENTAGE TO rtc
(void)bat_cap8();

#ifdef VALVE_NODE
  //  line pressure, for valve node
  uint16_t pressResult = readMCP3421avg_cont();
  wPres[0] = (pressResult >> 8);    //  MSB
  wPres[1] = (pressResult & 0xff); //  LSB
#endif 

#ifdef REED_NODE

  uint32_t count = read_count32();  //  guards against the change of the upper 16 bits 
  uint16_t lo   = (uint16_t)RTC_SLOW_MEMORY[ULP_COUNT_LO];
  uint16_t last = (uint16_t)RTC_SLOW_MEMORY[ULP_LAST_SENT];
  uint16_t diff = (uint16_t)(lo - last);   // modulo-16, safe across wrap

  //  reed count delta FROM rtc , low two bytes only
  Serial.printf("ULP_COUNT %lu \n", count);
  Serial.printf("ULP_LAST_SENT %lu \n", RTC_SLOW_MEMORY[ULP_LAST_SENT]);

  RTC_SLOW_MEMORY[ULP_REED_DELTA] = diff;    // used in display screen
  RTC_SLOW_MEMORY[ULP_LAST_SENT] = lo;     // advance marker
  Serial.printf("ULP_TS_DELTA_TICK_POP %lu \n", RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP]);
  Serial.printf("ULP_REED_DELTA %lu \n", RTC_SLOW_MEMORY[ULP_REED_DELTA]);
  //  flow calc stored in RTC_SLOW_MEMORY[ULP_FLOW_RATE]
  //  if ULP_TICK_POP <> 0 or no reed delta, then flow is zero
  uint32_t ticks_delta = (((uint32_t)((uint16_t)RTC_SLOW_MEMORY[ULP_TS_DELTA_HI] << 16)) | (uint16_t)(RTC_SLOW_MEMORY[ULP_TS_DELTA_LO]));
  if (diff && ticks_delta) {
        RTC_SLOW_MEMORY[ULP_FLOW_RATE] = (uint32_t)(((uint64_t)VOLUME_PER_TICK * (uint64_t)TICKS_PER_MIN) / (uint64_t)ticks_delta);
  } else {
    RTC_SLOW_MEMORY[ULP_FLOW_RATE] = 0;
  }
  Serial.printf("ticks_delta %lu \n", ticks_delta);
  //  only determine rate if the delta_tick_pop is not set (indicates that the timer has overflowed) and if the reed count has changed
  if ((RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP]) || (RTC_SLOW_MEMORY[ULP_REED_DELTA] == 0)) {
    RTC_SLOW_MEMORY[ULP_VOLUME_DELTA] = 0;    
  } else {    
    uint32_t vol = (uint32_t)(((uint64_t)diff * (uint64_t)VOLUME_PER_TICK) & 0xFFFFFFFFu);
    RTC_SLOW_MEMORY[ULP_VOLUME_DELTA] = vol; // or clamp if you prefer
  }
  RTC_SLOW_MEMORY[ULP_TS_DELTA_TICK_POP] = 0x0000;         //  clear the pop flag here, so we can see if the timer pops off again
#endif

}

static void display_status() {
  Serial.println("in display_status fx ");
  // [GPT] init-once in setup; removed display.init() here to avoid heap churn
  display.clear(); // wipe framebuffer before drawing
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  //common screen entries:  battery, cycle time, rssi, snr, display name
  snprintf(buffer, sizeof(buffer),  "battery: %lu %%", RTC_SLOW_MEMORY[ULP_BAT_PCT]);  // bat_cap8() populates this, and is run in prepareDataFrame
  display.drawString(210, 50, buffer);
  snprintf(buffer, sizeof(buffer),  "cycle %lu min", (uint32_t)appTxDutyCycle / 60000);  //  SHOULD BE 60000 milliseconds to mins
  display.drawString(210, 70, buffer);
  snprintf(buffer, sizeof(buffer),  "rssi (dBm): %d snr: %d", revrssi, revsnr);
  display.drawString(210, 90, buffer);
  snprintf(buffer, sizeof(buffer),  "Eui: ...%02x%02x", devEui[6], devEui[7]);  //  last two bytes of devEui
  display.drawString(210, 110, buffer);
  display.setFont(ArialMT_Plain_24);
  display.drawString(60, 100, g_name);  //  screen name

  #ifdef VALVE_NODE
  display.drawLine(0, 25, 120, 25);
  display.drawLine(150, 25, 270, 25);
  display.drawString(60, 0, "valve");
  show_vlv_status(0);
  display.drawString(60, 40, buffer);
  show_vlv_status(1);
  display.drawString(60, 65, buffer);  
  display.drawString(210, 0, "xxx psi");
  #endif

  #ifdef REED_NODE
  display.drawLine(0, 25, 80, 25);
  display.drawLine(95, 25, 300, 25);
  display.drawString(80, 0, "interval  total c");
  snprintf(buffer, sizeof(buffer),  "%u gal/m", (uint32_t)(RTC_SLOW_MEMORY[ULP_FLOW_RATE]));
  display.drawString(60, 35, buffer);
  snprintf(buffer, sizeof(buffer),  "%u gal", (uint32_t)(RTC_SLOW_MEMORY[ULP_VOLUME_DELTA]));
  display.drawString(60, 65, buffer); 
  uint32_t count = read_count32();
  snprintf(buffer, sizeof(buffer),  "%lu", count);  // counter
  display.drawString(220, 0, buffer);
  display.setFont(ArialMT_Plain_16);
  snprintf(buffer, sizeof(buffer),  "reed/wake: %lu", RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD]);  // reed closures per wake cycle 
  display.drawString(210, 30, buffer);
  #endif

  Serial.print("about to display.display \n");
  display.display();

    // for(int i = 0; i < 25; i++){
    // Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEMORY[i]);
    // }

  // [GPT] wait for E‑Ink to finish via BUSY pin (non‑blocking yield with timeout)
bool ok = eink_wait_idle(4000);
if (!ok) Serial.println("E‑ink wait timeout; proceeding cautiously");
}  // of function

/* Prepares the payload of the frame and decrements valve counter or turns off if time is up*/
static void prepareTxFrame(uint8_t port) {
  /*resolve valve status -- ? do we need to turn a valve off?  if so do that and then 
   *reset the valve cycle time if both valves are off
   *two ways to close a valve, based on end of timer and based on a close command.  
   *eiher way resets the cycle time only if BOTH valvees are closed.  
   *appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
   *data is pressure (msb, lsb) in raw adc conversion needs to be calibrated and converted to psi
   */
  if (valveA->onA) {
    if (valveA->time) {
      valveA->time--;
    } else {
      controlValve(0, 0);
      valveA->onA = 0;
      if (valveB->offB || !valveB->onB) appTxDutyCycle = TxDutyCycle_hold;
    }
  }
  if (valveB->onB) {
    if (valveB->time) {
      valveB->time--;
    } else {
      controlValve(1, 0);
      valveB->onB = 0;
      if (valveA->offA || !valveA->onA) appTxDutyCycle = TxDutyCycle_hold;
    }
  }
  // [GPT] replace hard 2 s delay with cooperative short wait (~200 ms)
uint32_t t_end = millis() + 200;
while ((int32_t)(millis() - t_end) < 0) {
  vTaskDelay(pdMS_TO_TICKS(10));
}
  pop_data();   //  populates the data fields from the sensors and calculations
  // [GPT] removed unused 'puc' variable
  appDataSize = 0; 

  #ifdef REED_NODE
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_REED_DELTA] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_FLOW_RATE] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_HI] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_HI] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_COUNT_LO] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //  
  #endif

  #ifdef VALVE_NODE
  appData[appDataSize++] = wPres[0];       //  msb
  appData[appDataSize++] = wPres[1];    //  lsb
  appData[appDataSize++] = (uint8_t)(valveA -> time);       //  valve A
  appData[appDataSize++] = (uint8_t)(valveB -> time);       //  valve B 
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEMORY[ULP_BAT_PCT] & 0xff));  //
  #endif

  // for(int i = 0; i < 16; i++){
  //   Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEMORY[i]);
  //   }
}

/* process the valve state:  Check for valve off commands first, then if on command set the TxDutyCycle
 * and turn on the valve, set the timer as cycles based on new TxDutyCycle */
void set_vlv_status() {
  //  to turn off and reset the TxDutyCycle to prior or valve on
  if (vlv_packet_pend.offA) {
    controlValve(0, 0);
    valveA->time = 0;
    valveA->onA = 0;
    if (valveB->offB || !valveB->onB) appTxDutyCycle = TxDutyCycle_hold;
    LoRaWAN.cycle(appTxDutyCycle);
  }
  if (vlv_packet_pend.offB) {
    controlValve(1, 0);
    valveB->time = 0;
    valveB->onB = 0;
    if (valveA->offA || !valveA->onA) appTxDutyCycle = TxDutyCycle_hold;
    LoRaWAN.cycle(appTxDutyCycle);
  }

  if (vlv_packet_pend.onA) {  //  valve A is sent command to open
    controlValve(0, 1);
    valveA->onA = 1;
    valveA->time = vlv_packet_pend.time;
    TxDutyCycle_hold = appTxDutyCycle;
    appTxDutyCycle = CYCLE_TIME_VALVE_ON;
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
  }
  if (vlv_packet_pend.onB) {  //  valve B is sent command to open
    controlValve(1, 1);
    valveB->onB = 1;
    valveB->time = vlv_packet_pend.time;
    TxDutyCycle_hold = appTxDutyCycle;
    appTxDutyCycle = CYCLE_TIME_VALVE_ON;
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
  }
}
void downLinkDataHandle(McpsIndication_t *mcpsIndication) {
  uint32_t TxDutyCycle_pend = 0;
  uint8_t *buf = mcpsIndication->Buffer;
  size_t len = mcpsIndication->BufferSize;
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }

  RTC_SLOW_MEMORY[ULP_RSSI] = mcpsIndication->Rssi;
  RTC_SLOW_MEMORY[ULP_SNR] = mcpsIndication->Snr;
  switch (mcpsIndication->Port) {
    case 5:  // cycle time set, 2 bytes MSB, LSB of new cycle time in min, convert to ms, update cycle time when valves are off
      if (len != 1 && len != 2) { Serial.println("cycle-time ignored (len!=1/2)"); break; } 
      if (len == 1) {
        TxDutyCycle_pend = (uint32_t)(mcpsIndication->Buffer[0]) * 1000 * 60;
      } else if (len == 2){
        TxDutyCycle_pend = ((uint32_t)(((mcpsIndication->Buffer[0]) << 8) | (mcpsIndication->Buffer[1]))) * 1000 * 60;
      } 
      if (TxDutyCycle_pend < 10 * 1000 * 60){
        TxDutyCycle_pend = 10 * 1000 * 60;  //  no less than 10 min
      }
      if (valveA->onA || valveB->onB ) {
        TxDutyCycle_hold = TxDutyCycle_pend;
      } else if (appTxDutyCycle != TxDutyCycle_pend){
        appTxDutyCycle = TxDutyCycle_pend;
        LoRaWAN.cycle(appTxDutyCycle);
      }

      g_need_display = true;
      break;

    case 6:  // valves, 2 bytes of data
    Serial.println("in the case 6 statement for valve command pending");
    {
      ValveState_t tmp{};                       // zero-init to avoid stale bits
      size_t n = (len < sizeof(tmp)) ? len : sizeof(tmp);
      memcpy(&tmp, buf, n);                     // copy only what we actually received
      vlv_packet_pend = tmp;                    // commit
    }
    set_vlv_status();
    g_need_display = true;
    break;
case 7: { // device name (ASCII, max 12), no-Arduino-String version
  constexpr size_t MAX_NAME = 12;
  if (len == 0 || len > MAX_NAME) { Serial.println("name ignored (len)"); break; }

  char new_name[MAX_NAME + 1];
  bool valid = true;
  size_t wrote = 0;
  for (size_t i = 0; i < len; ++i) {
    char c = (char)buf[i];
    if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E) { valid = false; break; }
    new_name[i] = c; wrote = i + 1;
  }
  if (!valid) { Serial.println("name ignored (non-ASCII)"); break; }
  new_name[wrote] = '\0';

  // ... (NVS compare/write) ...
  strncpy(g_name, new_name, sizeof(g_name)-1);
  g_name[sizeof(g_name)-1] = '\0';
  g_need_display = true;
  break;
  }


case 8: {
  uint16_t th = (mcpsIndication->BufferSize == 1)
                  ? (uint16_t)mcpsIndication->Buffer[0]
                  : (uint16_t)(((uint16_t)mcpsIndication->Buffer[0] << 8) |
                                (uint16_t)mcpsIndication->Buffer[1]);
  RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD] = th;

  if (!prefs.begin("cfg", false)) {   // NEW namespace
    Serial.println("NVS begin failed");
    break;
  }
  size_t w = prefs.putUShort("wake_th", th);  // NEW key
  uint16_t v = prefs.getUShort("wake_th", 0xFFFF);
  prefs.end();
  Serial.printf("NVS write %s (wrote=%u bytes, value=%u, verify=%u)\n",
                (w == sizeof(uint16_t)) ? "OK" : "FAIL",
                (unsigned)w, (unsigned)th, (unsigned)v);
  break;
  } // of CASE 8

} // of switch
  revrssi = mcpsIndication->Rssi;
  revsnr = mcpsIndication->Snr;

  Serial.println("downlink processed");
}  // of function

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

static const ulp_insn_t ulp_program[] = {
  M_LABEL(ULP_ENTRY_LABEL),
  I_DELAY(0x8fff),  // needs to be calibrated to time

// At top of ulp_program, BEFORE merging PENDING into COUNT:
I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
M_BG(ULP_SKIP_MERGE, 0),   // if CPU is awake (>0), skip the merge this cycle

// ── Merge PENDING (16-bit) into 32-bit COUNT (HI:LO) ──
I_MOVI(R1, ULP_COUNT_PENDING),
I_LD  (R0, R1, 0),
I_SUBI(R0, R0, 0),
M_BXZ (ULP_SKIP_MERGE),

I_MOVI(R2, ULP_COUNT_LO),
I_LD  (R3, R2, 0),
I_ADDR(R3, R3, R0),         // add pending → LO
I_ST  (R3, R2, 0),
M_BXF (ULP_BUMP_HI_MERGE),  // overflow → bump HI
M_BX  (ULP_CLR_PENDING),

M_LABEL(ULP_BUMP_HI_MERGE),
I_MOVI(R2, ULP_COUNT_HI),
I_LD  (R3, R2, 0),
I_ADDI(R3, R3, 1),
I_ST  (R3, R2, 0),

M_LABEL(ULP_CLR_PENDING),
I_MOVI(R0, 0),
I_MOVI(R1, ULP_COUNT_PENDING),
I_ST  (R0, R1, 0),

M_LABEL(ULP_SKIP_MERGE),

  //── 0) ULP_TIMER (low/hi) ────────────────────────────────
  I_MOVI(R1, ULP_TIMER_LO), I_LD(R3, R1, 0), I_ADDI(R3, R3, 1), I_ST(R3, R1, 0),
  I_MOVR(R0, R3),
  M_BG(ULP_NO_TIMER_WRAP, 0),  // skip if low > 0

    // wrap → bump hi or tick-pop
    I_MOVI(R1, ULP_TIMER_HI), I_LD(R2, R1, 0),
    I_MOVI(R0, 0xFFFF), I_SUBR(R0, R0, R2),      // 0xFFFF - hi
    M_BL(ULP_SET_TICK_POP, 1),                    // if hi == max → pop
    I_ADDI(R2, R2, 1), I_ST(R2, R1, 0), M_BX(ULP_NO_TIMER_WRAP),

  M_LABEL(ULP_SET_TICK_POP),
    I_MOVI(R0, 1), I_MOVI(R1, ULP_TICK_POP), I_ST(R0, R1, 0),
    I_MOVI(R2, 0), I_MOVI(R1, ULP_TIMER_HI), I_ST(R2, R1, 0),
    M_BX(ULP_NO_TIMER_WRAP),

  M_LABEL(ULP_NO_TIMER_WRAP),

  //── 1) Sample GPIO → raw_bit ─────────────────────────────
  I_RD_REG(RTC_GPIO_IN_REG, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S, RTC_GPIO_INDEX + RTC_GPIO_IN_NEXT_S),
  I_ANDI(R0, R0, 1), I_MOVR(R2, R0),

  //── 2) Rising-edge detect (raw_bit - prev_state == 1) ─────
  I_MOVI(R1, ULP_PREV_STATE), I_LD(R0, R1, 0), I_SUBR(R0, R2, R0),
  I_ST(R2, R1, 0),
  M_BL(ULP_NO_EDGE, 1), M_BG(ULP_NO_EDGE, 1),

  //── 3) Conditional bump ─────────────────────────────────────
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BG(ULP_CPU_IS_AWAKE, 0),  // if flag == 1, CPU is awake, use pending count

  // ── CPU idle → bump 32-bit COUNT (HI:LO) ──
I_MOVI(R1, ULP_COUNT_LO),
I_LD  (R3, R1, 0),
I_ADDI(R3, R3, 1),
I_ST  (R3, R1, 0),
M_BXF (ULP_BUMP_HI_EDGE),
M_BX  (ULP_AFTER_COUNT),

M_LABEL(ULP_BUMP_HI_EDGE),
I_MOVI(R1, ULP_COUNT_HI),
I_LD  (R3, R1, 0),
I_ADDI(R3, R3, 1),
I_ST  (R3, R1, 0),
M_BX  (ULP_AFTER_COUNT),

  M_LABEL(ULP_CPU_IS_AWAKE),
  // CPU is active → bump ULP_COUNT_PENDING
  I_MOVI(R1, ULP_COUNT_PENDING),
  I_LD(R3, R1, 0),
  I_ADDI(R3, R3, 1),
  I_ST(R3, R1, 0),
  M_BX(ULP_NO_WAKE),

  M_LABEL(ULP_AFTER_COUNT),

  //── 4) Snapshot ULP_TIMER → Δ lo/hi/pop ───────────────────
  I_MOVI(R1, ULP_TIMER_LO), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_LO), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_HI), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TICK_POP), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_TICK_POP), I_ST(R0, R1, 0),
  // clear timer & pop
  I_MOVI(R0, 0), I_MOVI(R1, ULP_TIMER_LO), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI), I_ST(R0, R1, 0), I_MOVI(R1, ULP_TICK_POP), I_ST(R0, R1, 0),

// ── Wake logic: use LO only (diff16 = LO - LAST_SENT) ──
I_MOVI(R1, ULP_COUNT_LO),
I_LD  (R0, R1, 0),            // R0 = LO
I_MOVI(R1, ULP_LAST_SENT),
I_LD  (R1, R1, 0),            // R1 = last_sent (16-bit)
I_SUBR(R0, R0, R1),           // diff16
I_MOVI(R2, ULP_DEBUG_PIN_STATE),
I_ST  (R0, R2, 0),
I_MOVI(R1, ULP_WAKE_THRESHOLD),
I_LD  (R1, R1, 0),
I_SUBR(R0, R1, R0),           // threshold - diff
M_BG  (ULP_NO_WAKE, 0),

  // only wake if main CPU idle
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BG(ULP_NO_WAKE, 0),            // skip if CPU awake
  I_WAKE(),

  M_LABEL(ULP_NO_WAKE),
  M_LABEL(ULP_NO_EDGE),
    M_BX(ULP_ENTRY_LABEL),
  };


void setup() {
  Serial.begin(115200);
  hardware_pins_init();  //
  setPowerEnable(1);
  pinMode(EPD_BUSY_PIN, INPUT);

  //Serial.printf("LORAWAN_APP_DATA_MAX_SIZE = %u\n", (unsigned)LORAWAN_APP_DATA_MAX_SIZE);

  // Wire.begin(PIN_SDA, PIN_SCL);  // three lines set up the display
  // if (!sht4.begin()) {
  //   Serial.println("Couldn't find SHT40");
  //   } else{
  //     Serial.println("found the SHT40");
  //   }
  // sht4.setPrecision(SHT4X_MED_PRECISION); // or HIGH, MED, LOW
  // sht4.setHeater(SHT4X_NO_HEATER);         // optional: use 

static bool displayInited = false;
if (!displayInited) { display.init(); displayInited = true; }
  if ((++epd_hygiene % 50) == 0) display.clear();  //  rest the screen ever 50 cycles to clean up any isssues.  
  display.screenRotate(ANGLE_180_DEGREE);
  display.setFont(ArialMT_Plain_24);
  //heap_caps_check_integrity_all(true);
    // configure RTC GPIO, enable ULP wake up.  
  ESP_ERROR_CHECK(rtc_gpio_hold_dis(RTC_GPIO_SENSOR_PIN)); 
  ESP_ERROR_CHECK(rtc_gpio_init(RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_set_direction(RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_set_direction_in_sleep((gpio_num_t)RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY));
  ESP_ERROR_CHECK(rtc_gpio_pullup_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_pulldown_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(rtc_gpio_hold_en(RTC_GPIO_SENSOR_PIN));
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    // figure out why we’re here
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  switch(cause){
  case ESP_SLEEP_WAKEUP_ULP:
    {
    // ── ULP woke us up ───────────────────────────────────────────────────────
    uint32_t count = read_count32();

    Serial.printf("Woke by ULP reed trigger, pulse count = %u\n", count);
    }
    break;

  case ESP_SLEEP_WAKEUP_TIMER:
    {      
    // ── timer woke us up ───────────────────────────────────────────────────────
    uint32_t count = read_count32();
    Serial.printf("Woke by RTC TIMER, pulse count = %u\n", count);
    }
    break;

  case ESP_SLEEP_WAKEUP_UNDEFINED : //  THIS IS COLD RESTART
    {
      display.drawString(0,40,"Connecting -> LoRaWAN");
      display.display();
      memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
      // see if we have a stored wake threshold
      uint16_t th = PULSE_THRESHOLD;  // default
      if (prefs.begin("cfg", true)) { // read-only
      th = prefs.getUShort("wake_th", PULSE_THRESHOLD);
      prefs.end();
      }  // of if
      RTC_SLOW_MEMORY[ULP_WAKE_THRESHOLD] = th;
      Serial.printf("ULP_WAKE_THRESHOLD loaded: %u\n", th);

      // see if we have a stored name port 7 to set
    
      if (prefs.begin("flash_namespace", true)) {
      char tmp[13] = {};
      prefs.getString("screenMsg", tmp, sizeof(tmp));
      strncpy(g_name, tmp, sizeof(g_name)-1);
      g_name[sizeof(g_name) - 1] = '\0';
      Serial.printf("Restored screen name %s from NVS\n", g_name);
      prefs.end();
      }  //  of if
     //  clear and then load and then kick off the ULP

      size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
      esp_err_t result = ulp_process_macros_and_load(ULP_PROG_START, ulp_program, &size);
      Serial.printf("load → %s, %u instructions\n",
              esp_err_to_name(result),
              (unsigned)size);
      // 4) Kick off the ULP
      ESP_ERROR_CHECK( ulp_run(ULP_PROG_START) );
      }  // of case UNDEFINED (reboot)
      break;

    default: {
      uint32_t count = read_count32();
      Serial.printf("Woke by DEFAULT, pulse count = %u\n", count);
      }
    break;
  }  // cause

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.printf("LORAWAN_APP_DATA_MAX_SIZE = %u\n", (unsigned)LORAWAN_APP_DATA_MAX_SIZE);
  g_need_display = true;

}; // of function

void loop() {
  switch (deviceState) {
    case DEVICE_STATE_INIT:
      {
        LoRaWAN.init(loraWanClass, loraWanRegion);
        // both set join DR and DR when ADR off
        LoRaWAN.setDefaultDR(3);
        break;
      }
    case DEVICE_STATE_JOIN:
      {
        LoRaWAN.join();
        break;
      }
    case DEVICE_STATE_SEND:
      {
        prepareTxFrame(appPort);
        LoRaWAN.send();
        // [GPT] trigger a render after every uplink; actual drawing happens outside the switch
        g_need_display = true;
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
          int32_t j = randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
          txDutyCycleTime = (uint32_t)((int32_t)appTxDutyCycle + j);
          if ((int32_t)txDutyCycleTime <= 0) txDutyCycleTime = appTxDutyCycle;  // simple clamp
          LoRaWAN.cycle(txDutyCycleTime);
          deviceState = DEVICE_STATE_SLEEP;
        break;
      }
    case DEVICE_STATE_SLEEP:
      {
        LoRaWAN.sleep(loraWanClass);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  } // of switch
  // [GPT] single draw point so RX callback stays light and we still refresh the screen
  if (g_need_display) {
    g_need_display = false;
    display_status();
  }
}  // of loop function
