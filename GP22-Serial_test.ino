/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care
  of use the correct LED pin whatever is the board used.
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald

  modified 2 Sep 2016
  by Arturo Guadalupi
*/

#include <SPI.h>
#include <GP22_regs.h>
#include <AtomicBlock_121b.h>
#include "GP22_Device.h"

//#define _BV(x) ((uint32_t)1 << x)

//#define REGISTER_0 ((0x1 << 28) | (0x1 << 24) |  ) //0x00241000
//#define REGISTER_1 ( (0x1 << 28) | (0x9 << 24) | _BV(23) | (0x1 << 19) | (0x1 << 16) | _BV(14) | (0x1 << 11) )

#define CONTENT_REGISTER_0 (uint32_t)( REGISTER_0_ANZ_FIRE(1) | REGISTER_0_DIV_FIRE(1) | REGISTER_0_ANZ_PER_CALRES(0) | REGISTER_0_DIV_CLKHS(0) | REGISTER_0_START_CLKHS(1) | REGISTER_0_ANZ_PORT(0) | REGISTER_0_TCYCLE(1) | REGISTER_0_ANZ_FAKE(0) | REGISTER_0_CALIBRATE(1) | REGISTER_0_NO_CAL_AUTO(0) | REGISTER_0_MESSB2(0) | REGISTER_0_SEL_ECLK_TEMP(1) )
#define CONTENT_REGISTER_1 (uint32_t)( REGISTER_1_HIT2(1) | REGISTER_1_HIT1(9) | REGISTER_1_HITIN2(1) | REGISTER_1_HITIN1(1) | REGISTER_1_EN_FAST_INIT(0) | REGISTER_1_CURR32K(0) | REGISTER_1_SEL_START_FIRE(1) | REGISTER_1_SEL_TSTO2(6) | REGISTER_1_SEL_TSTO1(0) )
// not working:#define CONTENT_REGISTER_1 (uint32_t)( REGISTER_1_HIT2(1) | REGISTER_1_HIT1(9) | REGISTER_1_HITIN2(1) | REGISTER_1_HITIN1(1) | REGISTER_1_EN_FAST_INIT(0) | REGISTER_1_CURR32K(1) | REGISTER_1_SEL_START_FIRE(1) | REGISTER_1_SEL_TSTO2(0) | REGISTER_1_SEL_TSTO1(0) )
#define CONTENT_REGISTER_2 (uint32_t)( REGISTER_2_EN_INT(0b101) | REGISTER_2_RFEDGE2(0) | REGISTER_2_RFEDGE1(0))
#define CONTENT_REGISTER_3 (uint32_t)( REGISTER_3_EN_ERR_VAL(0) | REGISTER_3_SEL_TIMO_MB2(2) | REGISTER_3_EN_FIRST_WAVE(0))
#define CONTENT_REGISTER_4 (uint32_t)( 0 ) //default Value DIS_PW = 1
#define CONTENT_REGISTER_5 (uint32_t)( REGISTER_5_CONF_FIRE(0b010) | REGISTER_5_DIS_PHASESHIFT(1) )
#define CONTENT_REGISTER_6 (uint32_t)( REGISTER_6_EN_ANALOG(0) | REGISTER_6_NEG_STOP_TEMP(1) | REGISTER_6_EN_INT(0b0101) | REGISTER_6_START_CLKHS(1) | REGISTER_6_DOUBLE_RES(1) )

#define SPI_SPEED 20000000
//#define SPI_SPEED 2000000

#define MAX_REMEASURE 5

const int rst_pin = 3;
const int ssn_pin = 2;

const int int_pin = 34;
const int fin_pin = 36;
const int dis_pin = 38;

const int flip_flop_pin = 24;

double powArray[32];

void write32(uint8_t address, uint32_t content, int n = 4) {
  if (n < 1 | n > 4) {
    return;
  }
  byte b0 = (byte)((content >> 24) & 0xFF);
  byte b1 = (byte)((content >> 16) & 0xFF);
  byte b2 = (byte)((content >> 8) & 0xFF);
  byte b3 = (byte)((content) & 0xFF);

//  for (int i = 31; i >= 0; i--) {
//    if ((content >> i) & 0x1) Serial.print("1");
//    else Serial.print("0");
//  }
//  Serial.println("");

  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  SPI.transfer(b0, SPI_CONTINUE);
  if (n > 1) SPI.transfer(b1, SPI_CONTINUE);
  if (n > 2) SPI.transfer(b2, SPI_CONTINUE);
  if (n > 3) SPI.transfer(b3);
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

}

void write8(uint8_t address, uint8_t content) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  SPI.transfer(content, SPI_CONTINUE);
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

}

void writeOpCode(uint8_t code) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(code);
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

}


uint32_t read32(uint8_t address, int n = 4) {
  if (n < 1 | n > 4) {
    return 0;
  }
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  uint32_t result = 0;
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  result = SPI.transfer(0, SPI_CONTINUE);
  if (n > 1) {
    result <<= 8;
    result |= SPI.transfer(0, SPI_CONTINUE);
  }
  if (n > 2) {
    result <<= 8;
    result |= SPI.transfer(0, SPI_CONTINUE);
  }
  if (n > 3) {
    result <<= 8;
    result |= SPI.transfer(0);
  }
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

  return result;
}

uint8_t read8(uint8_t address) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  uint8_t result = 0;
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  result = SPI.transfer(0, SPI_CONTINUE);
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

  return result;
}

uint64_t readID() {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  uint64_t id = 0;
  digitalWrite(ssn_pin, LOW);
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE1));
  SPI.transfer(READ_ID, SPI_CONTINUE);
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0, SPI_CONTINUE);
  id <<= 8;
  id = SPI.transfer(0);
  SPI.endTransaction();
  digitalWrite(ssn_pin, HIGH);

  return id;
}

void printValue(uint32_t val) {
  for (int i = 31; i >= 0; i--) {
    if ((val >> i) & 1) {
      Serial.print("1");
    }
    else {
      Serial.print("0");
    }
  }
}

double resultToMicroSeconds(uint32_t value) {
  double result = 0;
  for (int i = 0; i < 32; i++) {
    if (((value >> i) & 0x01) == 1) {
      result += powArray[i];
    }
  }
  //result liegt als Vielfaches des Uhrentaktes vor -> 1/4000000 sec.
  // -> 1/4 µSec
  return result / 4.0f;
}

double convertResult(uint32_t value, int integers, int len){
  if(len > 32 || integers > len){
    return 0.0f;
  }
  double result = 0;
  int split = len - integers;
  for (int i = 0; i < len; i++) {
    if (((value >> i) & 0x01) == 1) {
      result += (double)pow(2, i - split);
    }
  }
  return result;
}

GP22_Device * measureDevice1;

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println("Start");
  Serial.flush();
  measureDevice1 = new GP22_Device(20000000, 2, 34, 3);
  uint32_t config_registers[7] = {CONTENT_REGISTER_0, CONTENT_REGISTER_1, CONTENT_REGISTER_2, CONTENT_REGISTER_3, CONTENT_REGISTER_4, CONTENT_REGISTER_5, CONTENT_REGISTER_6};
  measureDevice1->setRegisters(config_registers);
  measureDevice1->init();
  switch(measureDevice1->getDeviceState()){
    case uninitialized:
      Serial.println("ERROR - Constructor does not exist");
      break;
    case init_failed:
      Serial.println("ERROR - Check communication and Pins");
      break;
    case running:
      Serial.println("Device Initialized");
      break;
    default:
      Serial.print("Unknown device state: ");
      Serial.println(measureDevice1->getDeviceState(), DEC);
      break;
  }
  Serial.flush();
}

double printAndMeasure(){
    boolean measure = true;
    int cnt = 0;
    uint16_t result = 0;
    while(measure){
      writeOpCode(Start_TOF);
      long timeoutTimer = millis();
      while (digitalRead(int_pin) == 1 && millis() - timeoutTimer < 3);
      //Read Result after Timeout (quick) or Hit (slow - processing takes a while -)
      result = (uint16_t) read32(READ_STAT, 2);
      if((result >> 9) & 0x1){
        if(cnt >= MAX_REMEASURE)
          return 0xFFFFFFFE;
      } else {
        measure = false;
      }
    }
    int alu_ptr = result & (uint16_t)0b111;
    uint32_t msmnt = 0;
    if(alu_ptr == 1){
      msmnt=read32(READ_RES_0);
    }
    else if(alu_ptr == 2){
      msmnt = read32(READ_RES_1);
    }
    else if(alu_ptr == 3){
      msmnt = read32(READ_RES_2);
    }
    else if(alu_ptr == 4){
      msmnt = read32(READ_RES_3);
    }
    double converted = resultToMicroSeconds(msmnt)*1000;
//    Serial.println(converted, DEC);
  return msmnt;
}

int msmnts = 0;
int good_msmnts = 0;
double rate = 15000; //Hz
long t_delay = (1.0f/rate)*1000000.0f - (double)(40) -1;

long t_start = millis();
long startTime = 0;
long stopTime = 0;
void loop() {
//    startTime = micros();
    uint32_t msmnt = measureDevice1->softStartMeasure(5);//rintAndMeasure();
//    stopTime = micros();
    if(msmnt != 0xFFFFFFFE)
      good_msmnts++;
    msmnts ++;
    if(millis() - t_start >= 1000){
      t_start = millis();
      Serial.print("measurements/sec: ");
      Serial.println(msmnts, DEC);
      Serial.print("good measurements/sec: ");
      Serial.print(good_msmnts, DEC);
//      if(good_msmnts <= msmnts-100 && rate > 0){
//        rate-=10;
//        t_delay = (1.0f/rate)*1000000.0f - 39;
//      } else {
//        if(rate < 40000) rate+=10;
//        t_delay = (1.0f/rate)*1000000.0f - 39;
//      }
      if(good_msmnts < msmnts){
        Serial.println(" !");
      }
      else
      {
        Serial.println("");
      }
      msmnts=0;
      good_msmnts=0;
    }
    //delayMicroseconds(max(0,min(t_delay, 100000)));
    //delay(100);
    delayMicroseconds(55); // 10k hits/sec
}

