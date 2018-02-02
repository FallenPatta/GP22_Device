#ifndef GP22_REGS_H_
#define GP22_REGS_H_


//Settings Registers
#define WRITE_REGISTER_0   0b10000000
#define WRITE_REGISTER_1  0b10000001
#define WRITE_REGISTER_2  0b10000010
#define WRITE_REGISTER_3  0b10000011
#define WRITE_REGISTER_4  0b10000100
#define WRITE_REGISTER_5  0b10000101
#define WRITE_REGISTER_6  0b10000111

//REGISTER 0
#define REGISTER_0_ANZ_FIRE(x)      ((uint32_t)x)<<28
#define REGISTER_0_DIV_FIRE(x)      ((uint32_t)x)<<24
#define REGISTER_0_ANZ_PER_CALRES(x)  ((uint32_t)x)<<22
#define REGISTER_0_DIV_CLKHS(x)     ((uint32_t)x)<<20
#define REGISTER_0_START_CLKHS(x)     ((uint32_t)x)<<18
#define REGISTER_0_ANZ_PORT(x)    ((uint32_t)x)<<17
#define REGISTER_0_TCYCLE(x)      ((uint32_t)x)<<16
#define REGISTER_0_ANZ_FAKE(x)    ((uint32_t)x)<<15
#define REGISTER_0_SEL_ECLK_TEMP(x)   ((uint32_t)x)<<14
#define REGISTER_0_CALIBRATE(x)     ((uint32_t)x)<<13
#define REGISTER_0_NO_CAL_AUTO(x)     ((uint32_t)x)<<12
#define REGISTER_0_MESSB2(x)      ((uint32_t)x)<<11
#define REGISTER_0_NEG_STOP2(x)     ((uint32_t)x)<<10
#define REGISTER_0_NEG_STOP1(x)     ((uint32_t)x)<<9
#define REGISTER_0_NEG_START(x)     ((uint32_t)x)<<8

//REGISTER 1
#define REGISTER_1_HIT2(x)        ((uint32_t)x)<<28
#define REGISTER_1_HIT1(x)        ((uint32_t)x)<<24
#define REGISTER_1_EN_FAST_INIT(x)  ((uint32_t)x)<<23
#define REGISTER_1_HITIN2(x)      ((uint32_t)x)<<19
#define REGISTER_1_HITIN1(x)      ((uint32_t)x)<<16
#define REGISTER_1_CURR32K(x)       ((uint32_t)x)<<15
#define REGISTER_1_SEL_START_FIRE(x)  ((uint32_t)x)<<14
#define REGISTER_1_SEL_TSTO2(x)     ((uint32_t)x)<<11
#define REGISTER_1_SEL_TSTO1(x)     ((uint32_t)x)<<8

//REGISTER 2
#define REGISTER_2_EN_INT(x)      ((uint32_t)x)<<29
#define REGISTER_2_RFEDGE2(x)       ((uint32_t)x)<<28
#define REGISTER_2_RFEDGE1(x)       ((uint32_t)x)<<27
#define REGISTER_2_DELVAL1(x)       ((uint32_t)x)<<8

//REGISTER 3 - EN_FIRST_WAVE = 0
#define REGISTER_3_EN_AUTOCALC_MB2(x) ((uint32_t)x)<<31
#define REGISTER_3_EN_FIRST_WAVE(x)   ((uint32_t)x)<<30
#define REGISTER_3_EN_ERR_VAL(x)  ((uint32_t)x)<<29
#define REGISTER_3_SEL_TIMO_MB2(x)  ((uint32_t)x)<<27
#define REGISTER_3_DELVAL2(x)   ((uint32_t)x)<<8

//REGISTER 3 - EN_FIRST_WAVE = 1
//#define REGISTER_3_EN_AUTOCALC_MB2(x)  ((uint32_t)x)<<31
//#define REGISTER_3_EN_FIRST_WAVE(x)   ((uint32_t)x)<<30
//#define REGISTER_3_EN_ERR_VAL(x)  ((uint32_t)x)<<29
//#define REGISTER_3_SEL_TIMO_MB2(x)  ((uint32_t)x)<<27
#define REGISTER_3_DELREL3(x)       ((uint32_t)x)<<20
#define REGISTER_3_DELREL2(x)       ((uint32_t)x)<<14
#define REGISTER_3_DELREL1(x)       ((uint32_t)x)<<8

//REGISTER 4 - EN_FIRST_WAVE = 0
#define REGISTER_4_DELVAL3(x)       ((uint32_t)x)<<8
//REGISTER 4 - EN_FIRST_WAVE = 1
#define REGISTER_4_DIS_PW(x)      ((uint32_t)x)<<16
#define REGISTER_4_EDGE_PW(x)     ((uint32_t)x)<<15
#define REGISTER_4_OFFSRNG2(x)    ((uint32_t)x)<<14
#define REGISTER_4_OFFSRNG1(x)    ((uint32_t)x)<<13
#define REGISTER_4_OFFS(x)      ((uint32_t)x)<<8

//REGISTER 5
#define REGISTER_5_CONF_FIRE(x)   ((uint32_t)x)<<29
#define REGISTER_5_EN_STARTNOISE(x) ((uint32_t)x)<<28
#define REGISTER_5_DIS_PHASESHIFT(x)  ((uint32_t)x)<<27
#define REGISTER_5_REPEAT_FIRE(x)     ((uint32_t)x)<<24
#define REGISTER_5_PHFIRE(x)      (x & 0xEFFF)<<8

//REGISTER 6
#define REGISTER_6_EN_ANALOG(x)   ((uint32_t)x)<<31
#define REGISTER_6_NEG_STOP_TEMP(x) ((uint32_t)x)<<30
#define REGISTER_6_DA_KORR(x)       ((uint32_t)x)<<25
#define REGISTER_6_TW2(x)         ((uint32_t)x)<<22
#define REGISTER_6_EN_INT(x)      ((uint32_t)x)<<21
#define REGISTER_6_START_CLKHS(x)     ((uint32_t)x)<<20
#define REGISTER_6_CYCLE_TEMP(x)    ((uint32_t)x)<<18
#define REGISTER_6_CYCLE_TOF(x)     ((uint32_t)x)<<16
#define REGISTER_6_HZ60(x)      ((uint32_t)x)<<15
#define REGISTER_6_FIREO_DEF(x)     ((uint32_t)x)<<14 //Set to 1 if internal analog section is used
#define REGISTER_6_QUAD_RES(x)    ((uint32_t)x)<<13
#define REGISTER_6_DOUBLE_RES(x)    ((uint32_t)x)<<12
#define REGISTER_6_TEMP_PORTDIR(x)  ((uint32_t)x)<<11
#define REGISTER_6_ANZ_FIRE(x)    ((uint32_t)x)<<8


//Results Registers
#define READ_RES_0      0b10110000
#define READ_RES_1      0b10110001
#define READ_RES_2      0b10110010
#define READ_RES_3      0b10110011
#define READ_STAT       0b10110100
#define READ_REG_1      0b10110101
#define READ_ID       0b10110111
#define READ_PW1ST      0b10111000

//OP CODES
#define Init        0b01110000
#define Power_On_Reset    0b01010000
#define Start_TOF     0b00000001
#define Start_Temp      0b00000010
#define Start_Cal_Resonator 0b00000011
#define Start_Cal_TDC   0b00000100
#define Start_TOF_Restart 0b00000101
#define Start_Temp_Restart  0b00000110


#endif //GP22_REGS_H_



