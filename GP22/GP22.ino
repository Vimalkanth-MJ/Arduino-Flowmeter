#include <Timber.h>
#include "gp22.h"
#include <Wire.h>
#include <SPI.h>
#include <avr/io.h>
#include "valve.h"

#include <LiquidCrystal_I2C.h>
#include <LoRa.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>

/* Connection to TDC-GP22

      Arduino             TDC-GP22
       12  <--------------> MISO
       11  <--------------> MOSI
       13  <--------------> SCK
       10  <--------------> SS
       02  <--------------> INT
      3V3  <--------------> VCC
      GND  <--------------> GND

  /* Connection of LoRa

      Arduino                LoRa
       12  <--------------> MISO
       11  <--------------> MOSI
       13  <--------------> SCK
       09  <--------------> NSS
       08  <--------------> RST
       03  <--------------> DIO0
      3V3  <--------------> VCC
      GND  <--------------> GND
*/

#define GP22_DEBUG
#define _CHECK_STATUS_ERROR(status, field) ((status & field) == field)
#define GP22_CLKHS_FREQ_MHZ 4
#define GP22_CLKHS_CONSTANT (GP22_CLKHS_FREQ_MHZ * GP22_CLKHS_FREQ_MHZ * 61.03515625)

/// Pipe inner DIAMETER (meters)
#define PIPE_D (0.0244)
#define SENSOR_LENGTH (0.072 + (0.005 * 2)) //(0.072) 
///  meters
#define WAVE_TRAVEL_LENGTH (SENSOR_LENGTH + PIPE_D)

/// cos of angle between sensor placement
#define COS_ALPHA (1.0)

/// pi/8
#define PI_OVER_8 (0.3926990816987)
/// SPEED_OF_SOUND_WATER(m/s)
#define CW (1480.0)
/// cubic meter per second
#define FLOW_CONSTANT_CMPS (PI_OVER_8 * CW * CW * PIPE_D * PIPE_D / (WAVE_TRAVEL_LENGTH * COS_ALPHA))
/// cubic meter per hour
#define FLOW_CONSTANT_CMPH (FLOW_CONSTANT_CMPS * 3600.0)

// Status bit definitions
#define STAT_TIMEOUT_TDC_OVERFLOW   0x0200
#define STAT_PRECOUNTER_OVERFLOW    0x0400
#define STAT_ERROR_OPEN_SENSOR      0x0800
#define STAT_ERROR_SHORT_SENSOR     0x1000
#define STAT_EEPROM_EQ_CREG         0x2000
#define STAT_EEPROM_DED             0x4000
#define STAT_EEPROM_ERROR           0x8000

//define the pins used by the transceiver module
#define ss 9
#define rst 8
#define dio0 3

const int FAST_MODE = 1;
const int PIN_INT = 2;
const int PIN_LED = 5;

unsigned long previousMillis = 0;
const unsigned long interval = 10000;
int counter = 0;
byte nuidPICC[4];
String tagId = "None";

double CLKHS_freq_cal = 1.0;
static uint16_t status_byte_UP = 0, status_byte_DOWN = 0;
static double Temp_cold_in_Celcius = 0.0, Temp_hot_in_Celcius = 0.0, flow_rate = 0.0;
static double PW1ST = 0.0;
bool state = false;
uint8_t id[7];

//void gp22_set_cal_factor(double cal_factor)
//{
//  CLKHS_freq_cal = cal_factor;
//}

GP22 tdc(PIN_INT, false);
LiquidCrystal_I2C lcd(0x27, 16, 2);
PN532_I2C pn532_i2c(Wire);
NfcAdapter nfc = NfcAdapter(pn532_i2c);

void readID()
{
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    digitalWriteFast(SS, LOW);
    // Transfer the opcode
    SPI.transfer(OPCODE_READ_ID);
    // Read the 7 bytes into the data array
    for (int i = 0; i < 7; i++)
    {
      id[i] = SPI.transfer(0x00);
    }
    digitalWriteFast(SS, HIGH);
  }
}

double gp22_calibrate()
{
  Serial.println("starting calibration");
  // Start Calibrate High Speed Clock Cycle
  tdc.sendOpcode(OPCODE_INIT);
  tdc.sendOpcode(OPCODE_START_CAL_RESONATOR);
  long t0 = tdc.waitForInterrupt(100000);
  //  while (digitalRead(PIN_INT) == HIGH)
  //    ;
  Timber.v("t0 %d", t0);
  //  delay(100);
  //  delayMicroseconds(500);
  /// \todo check calculation well
  // Calculate Correction factor
  // The time interval to be measured is set by ANZ_PER_CALRES
  // which defines the number of periods of the 32.768 kHz clock:
  // 2 periods = 61.03515625 µs
  double db = gp22_read_n_bytes(4, 0xB0, 0x00, 16);
  CLKHS_freq_cal = GP22_CLKHS_CONSTANT / db;
  //  printf("\n Correction factor for clock = %1.3f\n", CLKHS_freq_corr_fact);
  // CLKHS_freq_cal *= CLKHS_freq_corr_fact; // Calibrated Clock freque ncy
  Serial.print("clock cal factor= ");
  Serial.println(CLKHS_freq_cal);
  // Calibrated Clock frequency
  //  tdc.writeNBytes(0x83, 0x80000000, 4);
  return CLKHS_freq_cal;
}

void gp22_set_config(uint32_t cr0, uint32_t cr1, uint32_t cr2, uint32_t cr3,
                     uint32_t cr4, uint32_t cr5, uint32_t cr6)
{
  // Writing to the configuration registers (CR)
  // CR0: ANZ_FIRE=d20 DIV_FIRE=d3 ANZ_PER_CALRES=d0 ANZ_PORT=1...
  tdc.writeNBytes(0x80, cr0, 4);
  // CR1: ...
  tdc.writeNBytes(0x81, cr1, 4);
  // CR2: EN_INT=b0101 RFEDGE1=RFEDGE=0 DELVAL1=d5000 ID2=0
  tdc.writeNBytes(0x82, cr2, 4);
  // CR3: EN_AUTOCALC=1 EN_FIRST_WAVE=1 DELREL1=d8 DELREL2=d9 DELREL3=d10
  tdc.writeNBytes(0x83, cr3, 4);
  // CR4: DIS_PW=0 EDGE_PW=0 OFFSRNG1=0 OFFSRNG2=1 OFFS=0 ID4=0
  tdc.writeNBytes(0x84, cr4, 4);
  // CR5: FIRE_UP=1
  tdc.writeNBytes(0x85, cr5, 4);
  // CR6: ...
  tdc.writeNBytes(0x86, cr6, 4);
}

double gp22_compute_flow_rate()
{
  double tof_diff = gp22_measure_avg_tof_diff();
  if (isnan(tof_diff)) return NAN;
  return FLOW_CONSTANT_CMPH * tof_diff;
}

uint8_t gp22_status_count_error()
{
  uint16_t STAT_REG = (uint16_t)tdc.readNBytes(0xB0 | 0x04, 2);
  uint8_t count_error = 0;

  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_TIMEOUT_TDC_OVERFLOW))
    count_error++;// Bit9: Timeout_TDC
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_PRECOUNTER_OVERFLOW))
    count_error++;// Bit10: Timeout_Precounter
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_ERROR_OPEN_SENSOR))
    count_error++;// Bit11: Error_open
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_ERROR_SHORT_SENSOR))
    count_error++;// Bit12: Error_short
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_EEPROM_EQ_CREG))
    count_error++;// Bit13: EEPROM_eq_CREG
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_EEPROM_DED))
    count_error++;// Bit14: EEPROM_DED
  if (_CHECK_STATUS_ERROR(STAT_REG, STAT_EEPROM_ERROR))
    count_error++;// Bit15: EEPROM_Error

  return count_error;
}

void gp22_analyse_error_bit(uint16_t err) {

  // Bit9: Timeout_TDC
  if ((err & STAT_TIMEOUT_TDC_OVERFLOW) == STAT_TIMEOUT_TDC_OVERFLOW)
    Serial.println("-Indicates an overflow of the TDC unit");
  // Bit10: Timeout_Precounter
  if ((err & STAT_PRECOUNTER_OVERFLOW) == STAT_PRECOUNTER_OVERFLOW)
    Serial.println("-Indicates an overflow of the 14 bit precounter in MR 2");
  // Bit11: Error_open
  if ((err & STAT_ERROR_OPEN_SENSOR) == STAT_ERROR_OPEN_SENSOR)
    Serial.println("-Indicates an open sensor at temperature measurement");
  // Bit12: Error_short
  if ((err & STAT_ERROR_SHORT_SENSOR) == STAT_ERROR_SHORT_SENSOR)
    Serial.println("-Indicates a shorted sensor at temperature measurement");
  // Bit13: EEPROM_eq_CREG
  if ((err & STAT_EEPROM_EQ_CREG) == STAT_EEPROM_EQ_CREG)
    Serial.println("-Indicates whether the content of the configuration registers "
                   "equals the EEPROM");
  // Bit14: EEPROM_DED
  if ((err & STAT_EEPROM_DED) == STAT_EEPROM_DED)
    Serial.println("-Double error detection. A multiple error has been detected "
                   "which can not be corrected.");
  // Bit15: EEPROM_Error
  if ((err & STAT_EEPROM_ERROR) == STAT_EEPROM_ERROR)
    Serial.println("-Single error in EEPROM which has been corrected");
}

double gp22_read_n_bytes(uint8_t n_bytes, uint8_t read_opcode, uint8_t read_addr, uint8_t fractional_bits) {
  double Result = 0;
  uint32_t Result_read = tdc.readNBytes(read_opcode | read_addr, n_bytes);
  Result = Result_read / pow(2, fractional_bits);
  return Result;
}


double gp22_measure_avg_tof_diff()
{
  uint8_t Error_Bit = 0;
  /// \todo get CLKHS_freq_cal here or use start to compute it once?
  //  const double CLKHS_freq_cal = gp22_calibrate();

  // Start Time Of Flight Measurement Cycle
  tdc.sendOpcode(OPCODE_INIT);

  delayMicroseconds(5);

  tdc.sendOpcode(OPCODE_START_TOF_RESTART);

  // Wait for INT Slot_x, UPSTREAM
  long t1 = tdc.waitForInterrupt(1000000);
  //  Timber.i("Interrupt 1");

  //  long t2 = tdc.waitForInterrupt(1000000);

  // Read the Status Register
  status_byte_UP = (uint16_t)tdc.readNBytes(0xB0 | 0x04, 2);
  Error_Bit += gp22_status_count_error();

  // Result_UP, TOF in µs
  // RES_0 to RES_2 to be displayed only for evaluation purposes
  // RES_3 will be used for e.g. flow calculation
  double average_Result_up = gp22_read_n_bytes(4, 0xB0, 0x03, 16) / CLKHS_freq_cal;
#ifdef GP22_DEBUG
  double Result_0_up = gp22_read_n_bytes(4, 0xB0, 0x00, 16) / CLKHS_freq_cal;
  double Result_1_up = gp22_read_n_bytes(4, 0xB0, 0x01, 16) / CLKHS_freq_cal;
  double Result_2_up = gp22_read_n_bytes(4, 0xB0, 0x02, 16) / CLKHS_freq_cal;
#endif

  //  Timber.i("Send init down");

  tdc.sendOpcode(OPCODE_INIT);


  // Wait for INT Slot_x, DOWNSTREAM
  long t2 = tdc.waitForInterrupt(1000000);
  //  Timber.i("Interrupt 2");

  //  Timber.wtf("T1:%d T2:%d", t1, t2);

  // Read the Status Register
  status_byte_DOWN = (uint16_t)tdc.readNBytes(0xB0 | 0x04, 2);
  Error_Bit += gp22_status_count_error();

  // Result_DOWN, TOF in µs
  double average_Result_down = gp22_read_n_bytes(4, 0xB0, 0x03, 16) / CLKHS_freq_cal;
#ifdef GP22_DEBUG
  double Result_0_down = gp22_read_n_bytes(4, 0xB0, 0x00, 16) / CLKHS_freq_cal;
  double Result_1_down = gp22_read_n_bytes(4, 0xB0, 0x01, 16) / CLKHS_freq_cal;
  double Result_2_down = gp22_read_n_bytes(4, 0xB0, 0x02, 16) / CLKHS_freq_cal;
  Timber.i("result up(0,1,2,3)=" + String(Result_0_up, 4) + "|" + String(Result_1_up, 4) + "|" + String(Result_2_up, 4) + "|" + String(average_Result_up, 4));
  Timber.i("result down(0,1,2,3)=" + String(Result_0_down, 4) + "|" + String(Result_1_down, 4) + "|" + String(Result_2_down, 4) + "|" + String(average_Result_down, 4));
  //Timber.i("result up(0,1,2,3)=%s|%s|%s|%s down(0,1,2,3)=%s|%s|%s|%s", String(Result_0_up, 4), String(Result_1_up, 4), String(Result_2_up, 4), String(average_Result_up, 4), String(Result_0_down, 4), String(Result_1_down, 4), String(Result_2_down, 4), String(average_Result_down, 4));
#endif
  //  Timber.i("Avg Result -> Up: " + String(average_Result_up, 4) + "\tDown: " + String(average_Result_down, 4));

  //    if (Error_Bit > 0) {
  //        //gp22_analyse_error_bit();
  //        //println("\nError occured check status\n");
  //        return NAN;
  //    }

  //------------------------------------------------------------------------
  // Result after two measurements (first UPSTREAM then DOWNSTREAM)
  // Calculate UP- / DOWNSTREAM transit time difference with MCU

  // Divider for multi-hit sum (1..3)
  average_Result_up /= 3;
  average_Result_down /= 3;

  // discharge time in ns
  // Result_0_up *= 1000;
  // Result_1_up *= 1000;
  // Result_2_up *= 1000;

  //convert time to seconds
  average_Result_up /= 1000000;

  // Result_0_down *= 1000;
  // Result_1_down *= 1000;
  // Result_2_down *= 1000;

  //convert time to seconds
  average_Result_down /= 1000000;

  //  Timber.i("Time -> Up: " + String(average_Result_up, 4) + "\tDown: " + String(average_Result_down, 4));

  /// \todo check order
  //  return average_Result_down - average_Result_up;
  return average_Result_up - average_Result_down;
}

void logCallback(Level level, unsigned long time, String message)
{
  Serial.print(message);

}


double totalFlow;

void setup() {

  Serial.begin(115200);
  Timber.setLogCallback(logCallback);
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(7,OUTPUT);
  digitalWrite(7,HIGH);
  LoRa.setPins(ss, rst, dio0);
  while (!LoRa.begin(868E6)) {
    Serial.print("LoRa Initialising");
    Serial.print(".");
    delay(500);
  }
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Hello world!");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("LoRa Initialize");
  lcd.setCursor(2, 1);
  lcd.print("Success !!");
  Serial.println("LoRa Initializing OK!");
  delay(2000);
  lcd.clear();
  nfc.begin();
  delay(1000);
  lcd.setCursor(0, 0);
  lcd.print("NFC Initialize");
  lcd.setCursor(2, 1);
  lcd.print("Success !!");
  Serial.println("NFC Initializing OK!");
  delay(2000);
  lcd.clear();

  tdc.init();
  Timber.i("Starting up");
  tdc.sendOpcode(OPCODE_POWER_ON_RESET);
  delay(10);
  tdc.sendOpcode(OPCODE_INIT);
  delay(10);
  // Test SPI read/write to the TDC
  bool res = tdc.testCommunication();
  if (!res) {
    Timber.e("Read/write test failed!");
    lcd.print("TDC Failure");
  } else {
    Timber.i("Read/write test succeeded.");
    state = true;
  }

  if (state) {

    //    gp22_set_config(0x13076800,
    //                    0x21447FAD,
    //                    0xA01388BE,
    //                    0xD0A248AF,
    //                    0x200040BA,
    //                    0x400000AD,
    //                    0xC0C060AF);

    gp22_set_config(0x23076800,
                    0x21447FAD,
                    0xA01388BE,
                    0xD0A248AF,
                    0x200040BA,
                    0x400000AD,
                    0xC0C061AF);

    gp22_calibrate();
    readID();
    Serial.print("ID = 0x");
    for (int i = 0; i < 7; ++i) {
    Serial.print(id[i], HEX);
    }
    Serial.println();
  }
}


void loop()
{
  sendLoRaPacket();
  readNFC();
  updateMotor();  // Continuously check and stop the motor if needed
  if (state) {
    flow_rate = gp22_compute_flow_rate();
    Serial.print("status reg up=0x");
    Serial.print(status_byte_UP, HEX);
    Serial.print(" down=0x");
    Serial.print(status_byte_DOWN, HEX);
    Serial.println();
    //    gp22_analyse_error_bit(status_byte_UP);
    //    gp22_analyse_error_bit(status_byte_DOWN);
    //    Serial.print("flow rate = ");
    //    Serial.print(flow_rate, 4);
    //    Serial.println(" m^3/h");
    double lpm = flow_rate * (1000.0 / 60.0); // convert to litres per minute

    Serial.print("flow rate = ");
    Serial.print(lpm, 4);
    Serial.println(" ltrs/minute");

    // calculate the flow
    double currentFlow = lpm * (500 / (1000.0 * 60.0));

    totalFlow += currentFlow;

    lcd.setCursor(0, 1);
    lcd.print(String(lpm, 4));
    lcd.setCursor(0, 2);
    lcd.print(String(totalFlow, 4));

    if (status_byte_UP == 0x408 || status_byte_DOWN == 0x408) {
      lcd.setCursor(0, 3);
      lcd.print("Empty Pipe");
    } else {
      lcd.setCursor(0, 3);
      lcd.print("");
    }
    PW1ST = gp22_read_n_bytes(1, 0xB0, 0x08, 7);
    Serial.print("PW1ST = ");
    Serial.println(PW1ST, 2);
  }
  delay(500);
}
void sendLoRaPacket() {
  unsigned long currentMillis = millis();  // Get the current time
  // Check if the interval has passed since the last transmission
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;  // Save the current time
    Serial.print("Sending packet: ");
    Serial.println(counter);
    // Send LoRa packet to receiver
    LoRa.beginPacket();
    LoRa.print("Hi");
    LoRa.endPacket();
    lcd.setCursor(0, 0);
    lcd.print("Data Packet sent");
    lcd.setCursor(2, 1);
    lcd.print("Successfully!!");
    Serial.println("Data Packet Sent!");
    delay(1000);
    lcd.clear();
    counter++;
  }
}

void readNFC() {
  if (nfc.tagPresent())
  {
    NfcTag tag = nfc.read();
    tagId = tag.getUidString();
    Serial.println("Tag id");
    Serial.println(tagId);
    lcd.setCursor(0, 0);
    lcd.print("RFID Card Detected!");
    lcd.setCursor(0, 1);
    lcd.print(tagId);
    delay(1000);
    lcd.clear();
  }
}

void tdc_ISR()
{
  Timber.d("ISR");
}
