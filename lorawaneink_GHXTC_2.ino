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
// int16_t rssi __attribute__((section(".rtc_slow.data.0")));
// int16_t snr __attribute__((section(".rtc_slow.data.1")));
// int16_t bat_pct __attribute__((section(".rtc_slow.data.2")));
// uint32_t ulp_last_sent __attribute__((section(".rtc_slow.data.3")));
// uint32_t ulp_count __attribute__((section(".rtc_slow.data.4")));
// //RTC_DATA_ATTR uint16_t ulp_prev_state = 0;
// ValveState_t valveA __attribute__((section(".rtc_slow.data.6")));
// ValveState_t valveB __attribute__((section(".rtc_slow.data.7")));
// uint32_t ulp_debug_pin_state __attribute__((section(".rtc_slow.data.8")));
uint16_t reed_count_delta = 0;
ValveState_t vlv_packet_pend;  // used to keep the command and the state independent until resolved
Preferences pref;


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
uint8_t appPort = 2;
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
        sprintf(buffer,"%u min\n", (int16_t)valveA->time * 10);
      } else {
        sprintf(buffer, "A off");
      }
      break;
    case 1:
      if (valveB->onB) {
        sprintf(buffer,"%u min\n", (int16_t)valveB->time * 10);
      } else {
        sprintf(buffer, "B off");
      }
      break;
  }
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
  display.setFont(ArialMT_Plain_24);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(60, 0, "valve");
  show_vlv_status(0);
  display.drawString(60, 40, buffer);
  show_vlv_status(1);
  display.drawString(60, 65, buffer);
  prefs.begin("flash_namespace", true);                                  // open as read only
  display.drawString(60, 100, prefs.getString("screenMsg", "no name"));  //  default "Fupi"
  prefs.end();
  display.drawString(210, 0, "xxx psi");
  display.setFont(ArialMT_Plain_16);
  sprintf(buffer, "reed cycles: %lu", RTC_SLOW_MEM[ULP_COUNT]);  // counter
  display.drawString(210, 30, buffer);
  sprintf(buffer, "battery: %lu %%", RTC_SLOW_MEM[ULP_BAT_PCT]);  // bat_cap8() populates this, and is run in prepareDataFrame
  display.drawString(210, 50, buffer);
  sprintf(buffer, "cycle %lu min", (uint32_t)appTxDutyCycle / 60000);  //  SHOULD BE 60000 milliseconds to mins
  display.drawString(210, 70, buffer);
  sprintf(buffer, "rssi (dBm): %d", (int16_t)(RTC_SLOW_MEM[ULP_RSSI] & 0xffff));
  display.drawString(210, 90, buffer);
  sprintf(buffer, "SNR: %d", (int8_t)(RTC_SLOW_MEM[ULP_SNR] & 0xff));
  display.drawString(210, 110, buffer);
  Serial.print("about to display.display \n");
  display.display();
  delay(5000);
}
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
  //setPowerEnable(1);
  delay(2000);
  unsigned char *puc;
  appDataSize = 0;
  appData[appDataSize++] = wPres[0];
  appData[appDataSize++] = wPres[1];
  appData[appDataSize++] = bat_cap8();  //  function returns a uint8_t
  //setPowerEnable(0);
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
      if (mcpsIndication->BufferSize == 1) {
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

    uint8_t *buf = mcpsIndication->Buffer;
    size_t len = mcpsIndication->BufferSize;

    // build a String directly from the raw ASCII bytes:
    char str[len + 1];
    memcpy(str, buf, len);
    str[len] = '\0';
    // store it as a String in NVS
    prefs.begin("flash_namespace", false);
    prefs.putString("screenMsg", str);
    prefs.end();
    break;
    display_status();
    }  // of switch
  RTC_SLOW_MEM[ULP_RSSI] = mcpsIndication->Rssi;
  RTC_SLOW_MEM[ULP_SNR] = mcpsIndication->Snr;

  Serial.println("downlink processed");
}  // of function

/* set up the ULP to count pulses, and wake on 100 pulses, after each lorawan data send, the last_pulse_count is advanced*/

#define PULSE_THRESHOLD 10   // wake every 10 pulses (tune as you like)

static const ulp_insn_t ulp_program[] = {
  M_LABEL(ULP_ENTRY_LABEL),

    //─── 1) Sample GPIO17 → R0 ─────────────────────────────────────────────
    I_RD_REG( RTC_GPIO_IN_REG,
              RTC_GPIO_REG + RTC_GPIO_IN_NEXT_S,
              RTC_GPIO_REG + RTC_GPIO_IN_NEXT_S ),
    
    I_ANDI( R0, R0, 1 ),

    // stash raw bit in R2
    I_MOVR( R2, R0 ),

    //─── 2) Detect rising edge: delta = R0 – prev_state(R1) ─────────────────
    I_MOVI( R1, ULP_PREV_STATE ),
    I_LD(   R1, R1, 0 ),
    I_SUBR( R0, R0, R1 ),

    // if delta < 1 → no new pulse
    M_BL(   ULP_NO_EDGE, 1 ),

    //─── 3) Rising edge! bump the counter (in R3) ────────────────────────────
    I_MOVI( R3, ULP_COUNT ),
    I_LD(   R3, R3, 0 ),
    I_ADDI( R3, R3, 1 ),
    I_MOVI( R1, ULP_COUNT ),
    I_ST(   R3, R1, 0 ),         // RTC_SLOW_MEM[ULP_COUNT] = R3

    // update prev_state = raw bit
    I_MOVI( R1, ULP_PREV_STATE ),
    I_ST(   R2, R1, 0 ),         // RTC_SLOW_MEM[ULP_PREV_STATE] = R2

    //─── 4) Wake logic: diff = count – last_sent ─────────────────────────────
    I_MOVI( R1, ULP_LAST_SENT ),
    I_LD(   R1, R1, 0 ),         // R1 = RTC_SLOW_MEM[ULP_LAST_SENT]
    I_SUBR( R2, R3, R1 ),        // R2 = new_count – last_sent

    // if diff < threshold → skip wake
    M_BL(   ULP_NO_WAKE, PULSE_THRESHOLD ),

    // else wake the CPU
    I_WAKE(),

  M_LABEL( ULP_NO_WAKE ),
    I_HALT(),                    // stop to let ulp_run() return

    // loop back for next manual ping
    M_BX(    ULP_ENTRY_LABEL ),

  //─── no-edge path ──────────────────────────────────────────────────────────
  M_LABEL( ULP_NO_EDGE ),
    // just update prev_state
    I_MOVI( R1, ULP_PREV_STATE ),
    I_ST(    R2, R1, 0 ),

    M_BX(    ULP_ENTRY_LABEL ),
};



void setup() {


  Serial.begin(115200);
  hardware_pins_init();  //
  Wire.begin(PIN_SDA, PIN_SCL);  // three lines set up the display
  // if (!sht4.begin()) {
  //   Serial.println("Couldn't find SHT4x");
  //   while (1);
  // }
  // sht4.setPrecision(SHT4X_MED_PRECISION); // or HIGH, MED, LOW
  // sht4.setHeater(SHT4X_NO_HEATER);         // optional: use 

  display.init();
  display.clear();
  display.drawString(0, 0, "init >>> alive ");
  display.display();
  delay(5000);
  
    // 1) set up ULP
  
  memset(RTC_SLOW_MEM, 0, CONFIG_ULP_COPROC_RESERVE_MEM);
  bool ok = rtc_gpio_is_valid_gpio(RTC_GPIO_REG);
  printf("RTC GPIO %d valid: %s\n", RTC_GPIO_REG, ok ? "yes" : "no");

  rtc_gpio_init(RTC_GPIO_REG);
  rtc_gpio_set_direction(RTC_GPIO_REG, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_set_direction_in_sleep((gpio_num_t)RTC_GPIO_REG, RTC_GPIO_MODE_INPUT_ONLY);
  rtc_gpio_pullup_dis((gpio_num_t)RTC_GPIO_REG);
  rtc_gpio_pulldown_dis((gpio_num_t)RTC_GPIO_REG);
  //rtc_gpio_hold_en(RTC_GPIO_REG);

  // fire every 100 ms
  esp_err_t t = ulp_set_wakeup_period(0, 9000);
  Serial.printf("ulp_set_wakeup_period returned %s\n", esp_err_to_name(t));



  size_t size = sizeof(ulp_program) / sizeof(ulp_insn_t);
  esp_err_t result = ulp_process_macros_and_load(ULP_PROG_START, ulp_program, &size);
  // 2) Program the ULP wake-timer: 1 000 000 ticks ≈ 1 s @ 90 kHz
  Serial.printf("load → %s, %u instructions\n",
              esp_err_to_name(result),
              (unsigned)size);

  while(1==1){
    esp_err_t err = ulp_run(ULP_PROG_START);
  Serial.printf("ulp_run → %s\n", esp_err_to_name(err));
    // Dump the first N 32-bit words
    for(int i = 0; i < 16; i++){
       Serial.printf(" [%2d]: 0x%08X\n", i, RTC_SLOW_MEM[i]);
       }
    Serial.printf("count      %lu \n", RTC_SLOW_MEM[ULP_COUNT]);
    Serial.printf("prev_state %lu \n", RTC_SLOW_MEM[ULP_PREV_STATE]);
    delay(2500);
  }
      // while (1==1){
      // controlValve(1, 0);
      // controlValve(0, 0);
      // delay(1000);
      // controlValve(1, 1);
      // controlValve(0, 1);
      // delay(2000);
      // }

      // while (1==1){
      //   uint16_t adc_value;
      //   adc_value = readMCP3421avg_cont();
      //   printf("ADC value: %i \n", adc_value);
      //   delay(1000);
        
      // sensors_event_t humidity, temp;
      // if (sht4.getEvent(&humidity, &temp)) {
      //   Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" °C");
      //   Serial.print("Humidity: "); Serial.print(humidity.relative_humidity); Serial.println(" %");
      // } else {
      //   Serial.println("Read failed");
      // }
      // controlValve(1, 0);
      // controlValve(0, 0);
      // delay(1000);
      // controlValve(1, 1);
      // controlValve(0, 1); 
      // delay(2000);
      // digitalWrite(PIN_EN_SENSE_PWR, LOW);
      // delay(2000);
      // digitalWrite(PIN_EN_SENSE_PWR, HIGH);
      // printf("ULP count: %i \n", ulp_count);
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
        //digitalWrite(PIN_VE_CTL, LOW); // no more power to external MCU loads
        LoRaWAN.sleep(loraWanClass);
        wakeup_reason = esp_sleep_get_wakeup_cause();
        printf("I just awoke, and my reason for arousal is %i \n", wakeup_reason);
        break;
      }
    default:
      {
        deviceState = DEVICE_STATE_INIT;
        break;
      }
  } // of switch
}  // of loop function