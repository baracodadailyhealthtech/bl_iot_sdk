#ifndef __SCCB_H__
#define __SCCB_H__

#include "stdint.h"

typedef struct {
    uint16_t    addr;
    uint8_t     data;
} REG_16BIT_Type;

int SCCB_Init(void);
int SCCB_Scan(int first, int last);
int SCCB_Read(uint8_t slave_addr, uint8_t reg_addr, uint8_t* rdata, uint8_t rdsize);
int SCCB_Read_Reg16(uint8_t slave_addr, uint16_t reg_addr, uint8_t* rdata, uint8_t rdsize);
int SCCB_Write(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data);
int SCCB_Write_Reg16(uint8_t slave_addr, uint16_t reg_addr, uint8_t* data);

#endif
