#ifndef GP22DEVICE_H_
#define GP22DEVICE_H_

#include <Arduino.h>
#include <SPI.h>
#include <GP22_regs.h>
#include <AtomicBlock_121b.h>

#define SOFTSTART_DEFINES

enum GP22_State{
        uninitialized,
        init_failed,
        running
};

class GP22_Device{
        public:
                #ifdef SOFTSTART_DEFINES
                GP22_Device();
                #endif // SOFTSTART_DEFINES
                GP22_Device(uint32_t spi_speed, uint32_t chip_select_pin, uint32_t interrupt_pin, uint32_t reset_pin);
                ~GP22_Device();
                
                void init();
                void setRegister(uint8_t, uint32_t);
                void setRegisters(uint32_t *);
                
                void writeRegister(uint8_t);
                void writeRegister(uint8_t, uint32_t);
                void writeRegisters();
                void write32(uint8_t address, uint32_t content, int n = 4);
                void writeN(uint8_t address, uint32_t content, int n = 4);
                void write8(uint8_t address, uint8_t content);
                void writeOpCode(uint8_t code);
                
                uint32_t read32(uint8_t address, int n = 4);
                uint8_t read8(uint8_t address);
                uint64_t readID();
                
                void setSPIspeed(uint32_t);
                void setChipSelectPin(uint32_t);
                void setInterruptPin(uint32_t);
                void setResetPin(uint32_t);
                
                uint32_t softStartMeasure(int);
                
                GP22_State getDeviceState();
        private:
                GP22_State device_state = uninitialized;
                uint32_t spi_speed;
                uint32_t chip_select_pin;
                uint32_t interrupt_pin;
                uint32_t reset_pin;
                uint32_t set_registers[7];
                uint32_t to_set_registers[7];
};

#endif //GP22DEVICE_H_
