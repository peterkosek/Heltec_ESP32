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

// 2) A handy macro to get a pointer to any struct at a given word‐index:
#define RTC_SLOW_BYTE_MEM   ((uint8_t*)SOC_RTC_DATA_LOW)
#define RTC_SLOW_STRUCT_PTR(type, idx) \
    ((volatile type*)(RTC_SLOW_BYTE_MEM + (idx) * sizeof(uint32_t)))

// 3) Now you can declare C-pointers into that region:
volatile ValveState_t* valveA = RTC_SLOW_STRUCT_PTR(ValveState_t, ULP_VALVE_A);
volatile ValveState_t* valveB = RTC_SLOW_STRUCT_PTR(ValveState_t, ULP_VALVE_B);

DEPG0290BxS800FxX_BW display(5, 4, 3, 6, 2, 1, -1, 6000000);  // rst,dc,cs,busy,sck,mosi,miso,frequency
//GXHTC gxhtc;
Preferences prefs;  // for NVM
Adafruit_SHT4x sht4 = Adafruit_SHT4x();

char buffer[64];
ValveState_t vlv_packet_pend;  // used to keep the command and the state independent until resolved
Preferences pref;

#define REED_NODE       true
//#define VALVE_NODE      true
/* OTAA para*/
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x06, 0x53, 0xee };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x74, 0xD6, 0x6E, 0x63, 0x45, 0x82, 0x48, 0x27, 0xFE, 0xC5, 0xB7, 0x70, 0xBA, 0x2B, 0x50, 0x45 };

/* ABP para*/
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;

/*LoraWan channelsmask, default channels 0-7*/
uint16_t userChannelsMask[6] = { 0xff00, 0x0000, 0x0000, 0x0001, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
RTC_DATA_ATTR uint32_t appTxDutyCycle = 15000 * 2 * 1;
RTC_DATA_ATTR uint32_t TxDutyCycle_hold;

/*OTAA or ABP*/
bool overTheAirActivation = true;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;

/* Application port */
#ifdef REED_NODE
uint8_t appPort = 8;          //  REED_NODE port 8
#endif
#ifdef VALVE_NODE
uint8_t appPort = 8;          // VALVE+NODE port 9
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
        sprintf(buffer,"%u min\n", (unsigned)(valveA->time * 10));
      } else {
        sprintf(buffer, "A off");
      }
      break;
    case 1:
      if (valveB->onB) {
        sprintf(buffer,"%u min\n", (unsigned)(valveB->time * 10));
      } else {
        sprintf(buffer, "B off");
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
  uint16_t pressResult = 0;
  uint32_t reedDelta = 0;
  uint32_t flow = 0;
  uint32_t ticks_delta=0;

  //  BATTERY PERCENTAGE TO rtc
  RTC_SLOW_MEM[ULP_BAT_PCT] = bat_cap8();

#ifdef VALVE_NODE
  //  line pressure, for valve node
  pressResult = readMCP3421avg_cont();
  wPres[0] = (pressResult >> 8);    //  MSB
  wPres[1] = (pressResult & 0xff); //  LSB
#endif 

#ifdef REED_NODE
  //  reed count delta FROM rtc , low two bytes only
  Serial.printf("ULP_COUNT %lu \n", RTC_SLOW_MEM[ULP_COUNT]);
  Serial.printf("ULP_LAST_SENT %lu \n", RTC_SLOW_MEM[ULP_LAST_SENT]);
  Serial.printf("ULP_TS_DELTA_LO %lu \n", RTC_SLOW_MEM[ULP_TS_DELTA_LO]);
  RTC_SLOW_MEM[ULP_REED_DELTA] = (RTC_SLOW_MEM[ULP_COUNT] - RTC_SLOW_MEM[ULP_LAST_SENT]);    // used in display screen
  RTC_SLOW_MEM[ULP_LAST_SENT] = RTC_SLOW_MEM[ULP_COUNT];     // update ULP_LAST_SENT
  Serial.printf("ULP_REED_DELTA %lu \n", RTC_SLOW_MEM[ULP_REED_DELTA]);
  //  flow calc stored in RTC_SLOW_MEM[ULP_FLOW_RATE]
  //  if ULP_TICK_POP <> 0 or no reed delta, then flow is zero
  ticks_delta = (((uint32_t)((uint16_t)RTC_SLOW_MEM[ULP_TS_DELTA_HI] << 16)) | (uint16_t)(RTC_SLOW_MEM[ULP_TS_DELTA_LO]));
  Serial.printf("ticks_delta %lu \n", ticks_delta);

  if ((RTC_SLOW_MEM[ULP_TS_DELTA_TICK_POP]) || (RTC_SLOW_MEM[ULP_REED_DELTA] == 0)) {
    RTC_SLOW_MEM[ULP_FLOW_RATE] = 0;
    RTC_SLOW_MEM[ULP_VOLUME_DELTA] = 0;    
  } else {    
    RTC_SLOW_MEM[ULP_VOLUME_DELTA] = (uint32_t)(RTC_SLOW_MEM[ULP_REED_DELTA]) * VOLUME_PER_TICK;
    RTC_SLOW_MEM[ULP_FLOW_RATE] = (uint32_t)((VOLUME_PER_TICK * TICKS_PER_MIN) / ticks_delta);
  }
  RTC_SLOW_MEM[ULP_TS_DELTA_TICK_POP] |= 0x02;         //  set bit 1 so that we know this is stale (sent)  
#endif

}

static void display_status() {
  //Serial.print("VLV_STATUS      ");
  //displayPacketBits(vlv_packet);
  Serial.print("in display_status fx \n");
  //displayPacketBits(vlv_packet_pend);
  display.init();
  display.clear();
  
  display.drawLine(0, 25, 120, 25);
  display.drawLine(150, 25, 270, 25);
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  //common screen entries:  battery, cycle time, rssi, snr, display name
  sprintf(buffer, "battery: %lu %%", RTC_SLOW_MEM[ULP_BAT_PCT]);  // bat_cap8() populates this, and is run in prepareDataFrame
  display.drawString(210, 50, buffer);
  sprintf(buffer, "cycle %lu min", (uint32_t)appTxDutyCycle / 60000);  //  SHOULD BE 60000 milliseconds to mins
  display.drawString(210, 70, buffer);
  sprintf(buffer, "rssi (dBm): %d", (int16_t)(RTC_SLOW_MEM[ULP_RSSI] & 0xffff));
  display.drawString(210, 90, buffer);
  sprintf(buffer, "SNR: %d", (int8_t)(RTC_SLOW_MEM[ULP_SNR] & 0xff));
  display.drawString(210, 110, buffer);
  display.setFont(ArialMT_Plain_24);
  prefs.begin("flash_namespace", true);                                  // open as read only
  display.drawString(60, 100, prefs.getString("screenMsg", "no name"));  //  default "no name"
  prefs.end();

  #ifdef VALVE_NODE
  display.drawString(60, 0, "valve");
  show_vlv_status(0);
  display.drawString(60, 40, buffer);
  show_vlv_status(1);
  display.drawString(60, 65, buffer);  
  display.drawString(210, 0, "xxx psi");
  #endif

  #ifdef REED_NODE
  display.drawString(60, 0, "meter ticks");
  sprintf(buffer, "%u lpm", (uint32_t)(RTC_SLOW_MEM[ULP_FLOW_RATE]));
  display.drawString(60, 35, buffer);
  sprintf(buffer, "%u l", (uint32_t)(RTC_SLOW_MEM[ULP_VOLUME_DELTA]));
  display.drawString(60, 65, buffer); 
  sprintf(buffer, "%lu", RTC_SLOW_MEM[ULP_COUNT]);  // counter
  display.drawString(210, 0, buffer);
  display.setFont(ArialMT_Plain_16);
  sprintf(buffer, "reed/wake: %lu", RTC_SLOW_MEM[ULP_WAKE_THRESHOLD]);  // reed closures per wake cycle 
  display.drawString(210, 30, buffer);
  #endif

  Serial.print("about to display.display \n");
  display.display();

    for(int i = 0; i < 20; i++){
    Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEM[i]);
    }

  delay(5000);    // allow screen to  finish refresh befre power down
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
  delay(2000);
  pop_data();   //  populates the data fields from the sensors and calculations
  unsigned char *puc;
  appDataSize = 0; 

  #ifdef REED_NODE
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_REED_DELTA] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_REED_DELTA] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_FLOW_RATE] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_FLOW_RATE] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_COUNT] >> 8));       //  msb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_COUNT] & 0xff));    //  lsb
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_BAT_PCT] & 0xff));  //  
  #endif

  #ifdef VALVE_NODE
  appData[appDataSize++] = wPress[0];       //  msb
  appData[appDataSize++] = xPress[1];    //  lsb
  appData[appDataSize++] = (uint8_t)(valveA -> time);       //  valve A
  appData[appDataSize++] = (uint8_t)(valveB -> time);       //  valve B 
  appData[appDataSize++] = (uint8_t)((RTC_SLOW_MEM[ULP_BAT_PCT] & 0xff));  //
  #endif

  // for(int i = 0; i < 16; i++){
  //   Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEM[i]);
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
  char data[256];
  uint32_t TxDutyCycle_pend = 0;
  uint8_t *buf = mcpsIndication->Buffer;
  size_t len = mcpsIndication->BufferSize;
  char str[len + 1];
  Serial.printf("+REV DATA:%s,RXSIZE %d,PORT %d\r\n", mcpsIndication->RxSlot ? "RXWIN2" : "RXWIN1", mcpsIndication->BufferSize, mcpsIndication->Port);
  Serial.print("+REV DATA:");
  for (uint8_t i = 0; i < mcpsIndication->BufferSize; i++) {
    Serial.printf("%02X", mcpsIndication->Buffer[i]);
  }
  sprintf(data, "%02X", mcpsIndication->Buffer);
  RTC_SLOW_MEM[ULP_RSSI] = mcpsIndication->Rssi;
  RTC_SLOW_MEM[ULP_SNR] = mcpsIndication->Snr;
  switch (mcpsIndication->Port) {
    case 5:  // cycle time set, 2 bytes MSB, LSB of new cycle time in min, convert to ms, update cycle time when valves are off
      Serial.println("in the case 5 statement");
      if (len == 1) {
        TxDutyCycle_pend = (uint32_t)(mcpsIndication->Buffer[0]) * 1000 * 60;
      } else {
        TxDutyCycle_pend = ((uint32_t)(((mcpsIndication->Buffer[0]) << 8) | (mcpsIndication->Buffer[1]))) * 1000 * 60;
      }
      if (valveA->onA | valveB->onB ) {
        TxDutyCycle_hold = TxDutyCycle_pend;
      } else {
        appTxDutyCycle = TxDutyCycle_pend;
      }
  LoRaWAN.cycle(appTxDutyCycle);
  display_status();
  break;
  case 6:  // valves, 2 bytes of data
    Serial.println("in the case 6 statement for valve command pending");
    if (mcpsIndication->BufferSize == 2) {
      memcpy(&vlv_packet_pend, mcpsIndication->Buffer, sizeof(ValveState_t));
    }
    set_vlv_status();
    display_status();
    break;
  case 7:  // screen text up to 12 characters for display on screen (like a name)
    Serial.println("in the case 7 statement");

    // build a String directly from the raw ASCII bytes:
    memcpy(str, buf, len);
    str[len] = '\0';
    // store it as a String in NVS
    prefs.begin("flash_namespace", false);
    prefs.putString("screenMsg", str);
    prefs.end();
    break;
  case 8:       //  reset ULP_WAKE_THRESHOLD, the numberof reed activations to wake the cpu
    Serial.println("in the case 8 statement for ULP_PULSE_THRESHOLD");
    if (mcpsIndication->BufferSize == 1) {
      RTC_SLOW_MEM[ULP_WAKE_THRESHOLD] = (uint16_t)mcpsIndication->Buffer[0];
    } else if (mcpsIndication->BufferSize == 2) {
      RTC_SLOW_MEM[ULP_WAKE_THRESHOLD] =
     (uint16_t)mcpsIndication->Buffer[0] <<8 | ((uint16_t)mcpsIndication->Buffer[1]);
    } 
    // store it as a String in NVS
    prefs.begin("flash_namespace", false);
    prefs.putUShort("ULP_WAKE_THRESHOLD", RTC_SLOW_MEM[ULP_WAKE_THRESHOLD]);
    prefs.end();
    break;
    }  // of switch
  RTC_SLOW_MEM[ULP_RSSI] = mcpsIndication->Rssi;
  RTC_SLOW_MEM[ULP_SNR] = mcpsIndication->Snr;

  Serial.println("downlink processed");
}  // of function

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

// ULP program: reed-pulse counter with dynamic threshold in RTC_SLOW_MEM
static const ulp_insn_t ulp_program[] = {
  M_LABEL(ULP_ENTRY_LABEL),
  I_DELAY(45),  // ≈500 µs @90 kHz

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


  //── 3) Bump count ──────────────────────────────────────────
  I_MOVI(R1, ULP_COUNT), I_LD(R3, R1, 0), I_ADDI(R3, R3, 1), I_ST(R3, R1, 0),

  //── 4) Snapshot ULP_TIMER → Δ lo/hi/pop ───────────────────
  I_MOVI(R1, ULP_TIMER_LO), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_LO), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_HI), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TICK_POP), I_LD(R0, R1, 0), I_MOVI(R1, ULP_TS_DELTA_TICK_POP), I_ST(R0, R1, 0),
  // clear timer & pop
  I_MOVI(R0, 0), I_MOVI(R1, ULP_TIMER_LO), I_ST(R0, R1, 0),
  I_MOVI(R1, ULP_TIMER_HI), I_ST(R0, R1, 0), I_MOVI(R1, ULP_TICK_POP), I_ST(R0, R1, 0),

  //── 5) Wake logic: diff = count - last_sent - threshold ─────
  I_MOVI(R1, ULP_COUNT),           I_LD(R0, R1, 0),         // R0 = count
  I_MOVI(R1, ULP_LAST_SENT),       I_LD(R1, R1, 0),         // R1 = last_sent
  I_SUBR(R0, R0, R1),                                      // diff = count - last_sent
  I_MOVI(R1, ULP_WAKE_THRESHOLD),  I_LD(R1, R1, 0),         // R1 = threshold
  I_SUBR(R0, R1, R0),                                      // R0 = diff - threshold

  I_MOVI(R1, ULP_DEBUG_PIN_STATE), I_ST(R0, R1, 0),       // debug the diff
  M_BG(ULP_NO_WAKE, 0),            // skip wake if diff < threshold

  // only wake if main CPU idle
  I_RD_REG(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_MAIN_STATE_IN_IDLE_S, RTC_CNTL_MAIN_STATE_IN_IDLE_S),
  M_BL(ULP_NO_WAKE, 0),            // skip if CPU awake
  I_WAKE(),

  M_LABEL(ULP_NO_WAKE),
    M_BX(ULP_ENTRY_LABEL),
  M_LABEL(ULP_NO_EDGE),
    M_BX(ULP_ENTRY_LABEL),
};


void setup() {
  Serial.begin(115200);
  hardware_pins_init();  //
  setPowerEnable(1);
  // Enable the RTC-I2C clock so ULP snapshots actually work
  SET_PERI_REG_MASK(
    SENS_SAR_PERI_CLK_GATE_CONF_REG,
    SENS_RTC_I2C_CLK_EN
  );
  // make sure none of the selector bits are set:
//   clear bits 30, 29, 28 and leave only bit 0 free
uint32_t reg = READ_PERI_REG(RTC_CNTL_TIME_UPDATE_REG);
reg &= ~(
    (1u << RTC_CNTL_TIMER_SYS_RST_S)  |
    (1u << RTC_CNTL_TIMER_XTL_OFF_S)  |
    (1u << RTC_CNTL_TIMER_SYS_STALL_S)
  );
// now write it back
WRITE_PERI_REG(RTC_CNTL_TIME_UPDATE_REG, reg);

  Wire.begin(PIN_SDA, PIN_SCL);  // three lines set up the display
  if (!sht4.begin()) {
    Serial.println("Couldn't find SHT40");
    } else{
      Serial.println("found the SHT40");
    }
  sht4.setPrecision(SHT4X_MED_PRECISION); // or HIGH, MED, LOW
  sht4.setHeater(SHT4X_NO_HEATER);         // optional: use 

  display.init();
  display.clear();
  display.drawString(0, 0, "init >>> alive ");
  display.display();
  delay(5000);
  
    // 1) set up ULP if this is reboot

    // figure out why we’re here
  uint32_t count;
    // 1) always clear the asleep‐flag so ULP won’t wake you while CPU is up
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  uint16_t defaultTh;
  switch(cause){
  case ESP_SLEEP_WAKEUP_ULP:
    {
    // ── ULP woke us up ───────────────────────────────────────────────────────
    // fire the ULP every 100 ms (≈9000 ticks @90 kHz RTC slow-clk)
    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 90) );
    count = RTC_SLOW_MEM[ULP_COUNT];
    Serial.printf("Woke by ULP reed trigger, pulse count = %u\n", count);
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    }
    break;

  case ESP_SLEEP_WAKEUP_TIMER:
    { 
      
    // ── timer woke us up ───────────────────────────────────────────────────────
    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 90) );
    count = RTC_SLOW_MEM[ULP_COUNT];
    Serial.printf("Woke by RTC TIMER, pulse count = %u\n", count);
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    }
    break;

  case ESP_SLEEP_WAKEUP_UNDEFINED : //  THIS IS COLD RESTART
    {
    memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
    ESP_ERROR_CHECK( rtc_gpio_init(RTC_GPIO_SENSOR_PIN) );
    //ESP_ERROR_CHECK( rtc_gpio_input_enable(RTC_GPIO_SENSOR_PIN) );
    ESP_ERROR_CHECK( rtc_gpio_set_direction(RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY) );
    ESP_ERROR_CHECK( rtc_gpio_set_direction_in_sleep((gpio_num_t)RTC_GPIO_SENSOR_PIN, RTC_GPIO_MODE_INPUT_ONLY) );
    ESP_ERROR_CHECK( rtc_gpio_pullup_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN) );
    ESP_ERROR_CHECK( rtc_gpio_pulldown_dis((gpio_num_t)RTC_GPIO_SENSOR_PIN) );
    ESP_ERROR_CHECK( rtc_gpio_hold_en(RTC_GPIO_SENSOR_PIN));
    ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 90) );
    size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);

    if (!prefs.begin("flash_namespace", false)) {
      Serial.println("ERROR: could not open NVS namespace");
    }
    // see if we have a stored wake threshold
    if (prefs.isKey("ULP_WAKE_THRESHOLD")) {
      // key exists, grab it (stored as a 16-bit value)
      defaultTh = prefs.getUShort("ULP_WAKE_THRESHOLD");
      RTC_SLOW_MEM[ULP_WAKE_THRESHOLD] = defaultTh;
      Serial.printf("Restored threshold %u from NVS\n", defaultTh);
    } else {
      // no key yet – use some default
      defaultTh = PULSE_THRESHOLD;  // set in sensor-solenoid.h
      RTC_SLOW_MEM[ULP_WAKE_THRESHOLD] = defaultTh;
      Serial.printf("No saved threshold, using default %u\n", defaultTh);
    }
    prefs.end();

    esp_err_t result = ulp_process_macros_and_load(ULP_PROG_START, ulp_program, &size);
    Serial.printf("load → %s, %u instructions\n",
              esp_err_to_name(result),
              (unsigned)size);
    ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
    // 4) Kick off the ULP
    ESP_ERROR_CHECK( ulp_run(ULP_PROG_START) );
    }
    break;

    default: {
      ESP_ERROR_CHECK( ulp_set_wakeup_period(0, 90) );
      count = RTC_SLOW_MEM[ULP_COUNT];
      Serial.printf("Woke by DEFAULT, pulse count = %u\n", count);
      ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
      }
    break;
  }  // cause
// while(1){
//     for(int i = 0; i < 16; i++){
//     Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEM[i]);
//     }
//     delay(60000);
// }
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
}; // of function

void loop() {
  esp_sleep_wakeup_cause_t wakeup_reason;
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
        display_status();
        LoRaWAN.send();
        //delay(5000);
        deviceState = DEVICE_STATE_CYCLE;
        break;
      }
    case DEVICE_STATE_CYCLE:
      {
        // Schedule next packet transmission
        txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
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
}  // of loop function