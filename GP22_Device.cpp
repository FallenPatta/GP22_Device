#include <Arduino.h>
#include <SPI.h>
#include <GP22_regs.h>
#include <AtomicBlock_121b.h>
#include "GP22_Device.h"

#define SOFTSTART_DEFINES

#ifdef SOFTSTART_DEFINES

#define CONTENT_REGISTER_0 (uint32_t)( REGISTER_0_ANZ_FIRE(1) | REGISTER_0_DIV_FIRE(1) | REGISTER_0_ANZ_PER_CALRES(0) | REGISTER_0_DIV_CLKHS(0) | REGISTER_0_START_CLKHS(1) | REGISTER_0_ANZ_PORT(0) | REGISTER_0_TCYCLE(1) | REGISTER_0_ANZ_FAKE(0) | REGISTER_0_CALIBRATE(1) | REGISTER_0_NO_CAL_AUTO(0) | REGISTER_0_MESSB2(0) | REGISTER_0_SEL_ECLK_TEMP(1) )

#define CONTENT_REGISTER_1 (uint32_t)( REGISTER_1_HIT2(1) | REGISTER_1_HIT1(9) | REGISTER_1_HITIN2(1) | REGISTER_1_HITIN1(1) | REGISTER_1_EN_FAST_INIT(0) | REGISTER_1_CURR32K(0) | REGISTER_1_SEL_START_FIRE(1) | REGISTER_1_SEL_TSTO2(6) | REGISTER_1_SEL_TSTO1(0) )

#define CONTENT_REGISTER_2 (uint32_t)( REGISTER_2_EN_INT(0b101) | REGISTER_2_RFEDGE2(0) | REGISTER_2_RFEDGE1(0))

#define CONTENT_REGISTER_3 (uint32_t)( REGISTER_3_EN_ERR_VAL(0) | REGISTER_3_SEL_TIMO_MB2(2) | REGISTER_3_EN_FIRST_WAVE(0))

#define CONTENT_REGISTER_4 (uint32_t)( 0 ) //default Value DIS_PW = 1

#define CONTENT_REGISTER_5 (uint32_t)( REGISTER_5_CONF_FIRE(0b010) | REGISTER_5_DIS_PHASESHIFT(1) )

#define CONTENT_REGISTER_6 (uint32_t)( REGISTER_6_EN_ANALOG(0) | REGISTER_6_NEG_STOP_TEMP(1) | REGISTER_6_EN_INT(0b0101) | REGISTER_6_START_CLKHS(1) | REGISTER_6_DOUBLE_RES(1) )

GP22_Device::GP22_Device(){
        spi_speed = 20000000;
        chip_select_pin = 2;
        reset_pin = 3;
        interrupt_pin = 34;
        pinMode(chip_select_pin, OUTPUT);
        pinMode(interrupt_pin, INPUT);
        pinMode(reset_pin, OUTPUT);
        SPI.begin();
        
        digitalWrite(reset_pin, LOW);
        digitalWrite(reset_pin, HIGH);
        
        writeOpCode(Power_On_Reset);
        writeOpCode(Init);
        uint8_t calc = 0;
        calc = read8(READ_REG_1);
        if(calc != 0b01010101){
                device_state = init_failed;
                return;
        }
        to_set_registers[0] = CONTENT_REGISTER_0;
        to_set_registers[1] = CONTENT_REGISTER_1;
        to_set_registers[2] = CONTENT_REGISTER_2;
        to_set_registers[3] = CONTENT_REGISTER_3;
        to_set_registers[4] = CONTENT_REGISTER_4;
        to_set_registers[5] = CONTENT_REGISTER_5;
        to_set_registers[6] = CONTENT_REGISTER_6;
        writeRegisters();
                
        writeOpCode(Start_Cal_TDC);
        writeOpCode(Start_Cal_Resonator);
        while (digitalRead(interrupt_pin) == 1);
        device_state = running;
}

#endif // SOFTSTART_DEFINES

GP22_Device::GP22_Device(uint32_t spi_speed_, uint32_t chip_select_pin_, uint32_t interrupt_pin_, uint32_t reset_pin_){
        spi_speed = spi_speed_;
        chip_select_pin = chip_select_pin_;
        interrupt_pin = interrupt_pin_;
        reset_pin = reset_pin_;
        
        pinMode(chip_select_pin, OUTPUT);
        pinMode(interrupt_pin, INPUT);
        pinMode(reset_pin, OUTPUT);
        SPI.begin();
}

GP22_State GP22_Device::getDeviceState(){
        return device_state;
}

void GP22_Device::init(){
        digitalWrite(reset_pin, LOW);
        digitalWrite(reset_pin, HIGH);
        
        writeOpCode(Power_On_Reset);
        writeOpCode(Init);
        uint8_t calc = 0;
        calc = read8(READ_REG_1);
        if(calc != 0b01010101){
                device_state = init_failed;
                return;
        }
        writeRegisters();
                
        writeOpCode(Start_Cal_TDC);
        writeOpCode(Start_Cal_Resonator);
        while (digitalRead(interrupt_pin) == 1);
        device_state = running;
}

void GP22_Device::writeRegisters(){
        write32(WRITE_REGISTER_0, to_set_registers[0]);
        write32(WRITE_REGISTER_1, to_set_registers[1]);
        write32(WRITE_REGISTER_2, to_set_registers[2]);
        write32(WRITE_REGISTER_3, to_set_registers[3]);
        write32(WRITE_REGISTER_4, to_set_registers[4]);
        write32(WRITE_REGISTER_5, to_set_registers[5]);
        write32(WRITE_REGISTER_6, to_set_registers[6]);
        set_registers[0] = to_set_registers[0];
        set_registers[1] = to_set_registers[1];
        set_registers[2] = to_set_registers[2];
        set_registers[3] = to_set_registers[3];
        set_registers[4] = to_set_registers[4];
        set_registers[5] = to_set_registers[5];
        set_registers[6] = to_set_registers[6];
}

void GP22_Device::writeRegister(uint8_t num, uint32_t val){
        setRegister(num, val);
        writeRegister(num);
}

void GP22_Device::writeRegister(uint8_t num){
        uint8_t address = 0;
        switch(num){
        case 0:
                address = WRITE_REGISTER_0;
                break;
        case 1:
                address = WRITE_REGISTER_1;
                break;
        case 2:
                address = WRITE_REGISTER_2;
                break;
        case 3:
                address = WRITE_REGISTER_3;
                break;
        case 4:
                address = WRITE_REGISTER_4;
                break;
        case 5:
                address = WRITE_REGISTER_5;
                break;
        case 6:
                address = WRITE_REGISTER_6;
                break;
        default:
                return;
        }
        write32(address, to_set_registers[num]);
        set_registers[num] = to_set_registers[num];
}

void GP22_Device::setRegisters(uint32_t * regs){
        for(int i = 0; i<7; i++)
        {
                to_set_registers[i] = *(regs+i);
        }
}

void GP22_Device::setRegister(uint8_t num, uint32_t val){
        if(num < 0 || num > 6){
                return;
        }
        to_set_registers[num] = val;
}

void GP22_Device::write32(uint8_t address, uint32_t content, int n) {
  if (n < 1 | n > 4) {
    return;
  }
  byte b0 = (byte)((content >> 24) & 0xFF);
  byte b1 = (byte)((content >> 16) & 0xFF);
  byte b2 = (byte)((content >> 8) & 0xFF);
  byte b3 = (byte)((content) & 0xFF);

  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  SPI.transfer(b0, SPI_CONTINUE);
  if (n > 1) SPI.transfer(b1, SPI_CONTINUE);
  if (n > 2) SPI.transfer(b2, SPI_CONTINUE);
  if (n > 3) SPI.transfer(b3);
  SPI.endTransaction();
  digitalWrite(chip_select_pin, HIGH);

}

void GP22_Device::write8(uint8_t address, uint8_t content) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  SPI.transfer(content, SPI_CONTINUE);
  SPI.endTransaction();
  digitalWrite(chip_select_pin, HIGH);

}

void GP22_Device::writeOpCode(uint8_t code) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  SPI.transfer(code);
  SPI.endTransaction();
  digitalWrite(chip_select_pin, HIGH);

}


uint32_t GP22_Device::read32(uint8_t address, int n) {
  if (n < 1 | n > 4) {
    return 0;
  }
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  uint32_t result = 0;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
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
  digitalWrite(chip_select_pin, HIGH);

  return result;
}

uint8_t GP22_Device::read8(uint8_t address) {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  uint8_t result = 0;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
  SPI.transfer(address, SPI_CONTINUE);
  result = SPI.transfer(0, SPI_CONTINUE);
  SPI.endTransaction();
  digitalWrite(chip_select_pin, HIGH);

  return result;
}

uint64_t GP22_Device::readID() {
  AtomicBlock< Atomic_RestoreState, _Safe >     a_Block;
  //Serial.flush();
  uint64_t id = 0;
  digitalWrite(chip_select_pin, LOW);
  SPI.beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE1));
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
  digitalWrite(chip_select_pin, HIGH);

  return id;
}

uint32_t GP22_Device::softStartMeasure(int retrys){
        int retry_counter = 0;
        uint16_t result = 0;
        while(true)
        {
                writeOpCode(Start_TOF);
                long timeoutTimer = millis();
                while (digitalRead(interrupt_pin) == 1 && millis() - timeoutTimer < 3);
                //Read Result after Timeout (quick) or Hit (slow - processing takes a while -)
                result = (uint16_t) read32(READ_STAT, 2);
                if((result >> 9) & 0x1){
                        if(retry_counter >= retrys)
                                return 0xFFFFFFFE;
                } else
                {
                        break;
                }
                retry_counter ++;
        }
        int alu_ptr = result & (uint16_t)0b111;
        uint32_t msmnt = 0;
        if(alu_ptr == 1)
        {
                msmnt=read32(READ_RES_0);
        }
        else if(alu_ptr == 2)
        {
                msmnt = read32(READ_RES_1);
        }
        else if(alu_ptr == 3)
        {
                msmnt = read32(READ_RES_2);
        }
        else if(alu_ptr == 4)
        {
                msmnt = read32(READ_RES_3);
        } else
        {
                return 0xFFFFFFFE;
        }
        //double converted = resultToMicroSeconds(msmnt)*1000;

        return msmnt;
}

/*
  //INIT
  
  pinMode(ssn_pin, OUTPUT);
  pinMode(rst_pin, OUTPUT);
  pinMode(fin_pin, INPUT);
  pinMode(dis_pin, INPUT);
  pinMode(int_pin, INPUT);
  pinMode(flip_flop_pin, OUTPUT);
  
  SPI.begin();

  for(int i = 0; i<32; i++){
    powArray[i] = (double)pow(2,i-16);
  }

  digitalWrite(rst_pin, LOW);
  digitalWrite(rst_pin, HIGH);

  byte calc = 0;

  while (calc != 0b01010101) {
    Serial.println("STARTING");

    writeOpCode(Power_On_Reset);

    writeOpCode(Init);

    calc = read8(READ_REG_1);
    delay(100);
    Serial.println(calc, BIN);
  }

  while (calc != 0b01100110) {
    Serial.println("TESTING");
    write32(WRITE_REGISTER_1, ((uint32_t)0b01100110) << 24);
    calc = read8(READ_REG_1);
    delay(100);
    Serial.println(calc, BIN);
  }
  write32(WRITE_REGISTER_1, ((uint32_t)0b01010101) << 24);
  calc = read8(READ_REG_1);
  Serial.println(calc, BIN);
  Serial.println("DONE");

  Serial.println("SET REGS");
  delay(100);
  write32(WRITE_REGISTER_0, CONTENT_REGISTER_0, 4);
  write32(WRITE_REGISTER_1, CONTENT_REGISTER_1, 4);
  write32(WRITE_REGISTER_2, CONTENT_REGISTER_2, 4);
  write32(WRITE_REGISTER_3, CONTENT_REGISTER_3, 4);
  write32(WRITE_REGISTER_4, CONTENT_REGISTER_4, 4);
  write32(WRITE_REGISTER_5, CONTENT_REGISTER_5, 4);
  write32(WRITE_REGISTER_6, CONTENT_REGISTER_6, 4);
  delay(100);
  Serial.println("SET");

  uint64_t id = readID();
  for (int i = 63; i >= 0; i--) {
    if ((id >> i) & 1) {
      Serial.print("1");
    }
    else {
      Serial.print("0");
    }
  }
  Serial.println("");

  writeOpCode(Start_Cal_TDC);
  delay(1);
  uint16_t result = (uint16_t) read32(READ_STAT, 2);
  for (int i = 15; i >= 0; i--) {
    if ((result >> i) & 1) {
      Serial.print("1");
    }
    else {
      Serial.print("0");
    }
  }
  Serial.println("");
  int a = digitalRead(int_pin);
  int b = digitalRead(fin_pin);
  int c = digitalRead(dis_pin);
  Serial.println(a);

  Serial.println("Start Cal Resonator");
  writeOpCode(Start_Cal_Resonator);
  while (digitalRead(int_pin) != 0)  {
    //Serial.println(digitalRead(int_pin), DEC);
  }
  uint32_t readCalResult = read32(READ_RES_0);
  printValue(readCalResult);
  Serial.println("");
  Serial.println(resultToMicroSeconds(readCalResult));
  double calibration_Value = (double)61.035 / resultToMicroSeconds(readCalResult);
  Serial.println(calibration_Value, DEC);
  
  Serial.println("");
  
  //END INIT

//  writeOpCode(Init);
//  for(int i = 0; i<5; i++){
//    writeOpCode(Start_TOF);
//    while (digitalRead(int_pin) == a);
//    Serial.println("TOF_FINISHED");
//    //Read Result after Timeout
//    result = (uint16_t) read32(READ_STAT, 2);
//    for (int i = 15; i >= 0; i--) {
//      if ((result >> i) & 1) {
//        Serial.print("1");
//      }
//      else {
//        Serial.print("0");
//      }
//    }
//    Serial.println("");
//    int alu_ptr = result & (uint16_t)0b111;
//    Serial.print("ALU Write register: ");
//    Serial.println(alu_ptr, DEC);
//  
//    uint32_t result0 = read32(READ_RES_0);
//    uint32_t result1 = read32(READ_RES_1);
//    uint32_t result2 = read32(READ_RES_2);
//    uint32_t result3 = read32(READ_RES_3);
//    Serial.println("Results:");
//    Serial.println(resultToMicroSeconds(result0)*1000);
//    Serial.println(resultToMicroSeconds(result1)*1000);
//    Serial.println(resultToMicroSeconds(result2)*1000);
//    Serial.println(resultToMicroSeconds(result3)*1000);
//  
//    Serial.println("");
//    delay(50);
//  }
//  writeOpCode(Init);
//  while (digitalRead(int_pin) == a);
//  Serial.println("TOF_FINISHED");
//
//  //Read Result after Timeout
//  result = (uint16_t) read32(READ_STAT, 2);
//  for (int i = 15; i >= 0; i--) {
//    if ((result >> i) & 1) {
//      Serial.print("1");
//    }
//    else {
//      Serial.print("0");
//    }
//  }
//  Serial.println("");
//  alu_ptr = result & (uint16_t)0b111;
//  Serial.print("ALU Write register: ");
//  Serial.println(alu_ptr, DEC);
//
//  result0 = read32(READ_RES_0);
//  result1 = read32(READ_RES_1);
//  result2 = read32(READ_RES_2);
//  result3 = read32(READ_RES_3);
//  Serial.println("Results:");
//  Serial.println(resultToMicroSeconds(result0)*1000);
//  Serial.println(resultToMicroSeconds(result1)*1000);
//  Serial.println(resultToMicroSeconds(result2)*1000);
//  Serial.println(resultToMicroSeconds(result3)*1000);
*/
