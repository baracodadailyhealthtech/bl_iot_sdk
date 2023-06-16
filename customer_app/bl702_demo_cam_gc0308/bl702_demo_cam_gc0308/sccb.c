#include "bl702_glb.h"
#include "bl702_i2c.h"

int SCCB_Init(void)
{
    GLB_GPIO_Type pinList[2] = {16, 11};
    GLB_GPIO_Func_Init(GPIO_FUN_I2C, pinList, 2);

    GLB_AHB_Slave1_Clock_Gate(DISABLE,BL_AHB_SLAVE1_I2C);
    GLB_Set_I2C_CLK(1,0);

    return 0;
}

int SCCB_Read(uint8_t slave_addr, uint8_t reg_addr, uint8_t* rdata, uint8_t rdsize)
{
    I2C_Transfer_Cfg tranCfg;

    tranCfg.slaveAddr = slave_addr;
    tranCfg.stopEveryByte = DISABLE;
    tranCfg.subAddrSize = 1;
    tranCfg.subAddr = reg_addr;
    tranCfg.dataSize = rdsize;
    tranCfg.data = rdata;
    tranCfg.clk = 100000;

    if(SUCCESS != I2C_MasterReceiveBlocking(I2C0_ID, &tranCfg)) {
        return -1;
    } 

    return 0;
}

int SCCB_Write(uint8_t slave_addr, uint8_t reg_addr, uint8_t* data)
{
    I2C_Transfer_Cfg tranCfg;

    tranCfg.slaveAddr = slave_addr;
    tranCfg.stopEveryByte = DISABLE;
    tranCfg.subAddrSize = 1;
    tranCfg.subAddr = reg_addr;
    tranCfg.dataSize = 1;
    tranCfg.data = data;
    tranCfg.clk = 100000;

    if(SUCCESS != I2C_MasterSendBlocking(I2C0_ID, &tranCfg)) {
        return -1;
    }

    return 0;
}
