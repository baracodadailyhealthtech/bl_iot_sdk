#include "bl702l_ef_cfg.h"
#include "bl702l_ef_ctrl.h"
#include "ef_data_0_reg.h"


#define EF_CTRL_LOAD_BEFORE_READ_R0 EF_Ctrl_Load_Efuse_R0()


/****************************************************************************/ /**
 * @brief  Whether Capcode is empty
 *
 * @param  slot: Cap code slot
 * @param  reload: Whether reload
 *
 * @return 0 for all slots full,1 for others
 *
*******************************************************************************/
uint8_t EF_Ctrl_Is_CapCode_Empty(uint8_t slot, uint8_t reload)
{
    uint32_t tmp;

    /* Switch to AHB clock */
    EF_Ctrl_Sw_AHB_Clk_0();

    if (reload) {
        EF_CTRL_LOAD_BEFORE_READ_R0;
    }

    if (slot == 0) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        return (EF_Ctrl_Is_All_Bits_Zero(tmp, 0, 10));
    } else if (slot == 1) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        return (EF_Ctrl_Is_All_Bits_Zero(tmp, 10, 10));
    } else if (slot == 2) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        return (EF_Ctrl_Is_All_Bits_Zero(tmp, 20, 10));
    } else {
        return 0;
    }
}

/****************************************************************************/ /**
 * @brief  Efuse write Cap code
 *
 * @param  slot: Cap code slot
 * @param  code: Cap code value
 * @param  program: Whether program
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type EF_Ctrl_Write_CapCode_Opt(uint8_t slot, uint8_t code, uint8_t program)
{
    uint32_t tmp;
    uint32_t trim;

    if (slot >= 3) {
        return ERROR;
    }

    /* Switch to AHB clock */
    EF_Ctrl_Sw_AHB_Clk_0();
    EF_CTRL_LOAD_BEFORE_READ_R0;

    trim = code;
    trim |= EF_Ctrl_Get_Trim_Parity(code, 8) << 8;
    trim |= 1 << 9;

    if (slot == 0) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        BL_WR_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3, tmp | (trim << 0));
    } else if (slot == 1) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        BL_WR_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3, tmp | (trim << 10));
    } else if (slot == 2) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        BL_WR_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3, tmp | (trim << 20));
    }

    if (program) {
        EF_Ctrl_Program_Efuse_0();
    }

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Efuse read Cap code
 *
 * @param  slot: Cap code slot
 * @param  code: Cap code pointer
 * @param  reload: Whether reload
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type EF_Ctrl_Read_CapCode_Opt(uint8_t slot, uint8_t *code, uint8_t reload)
{
    uint32_t tmp;
    uint32_t trim;
    uint8_t en;
    uint8_t parity;

    if (slot >= 3) {
        return ERROR;
    }

    /* Switch to AHB clock */
    EF_Ctrl_Sw_AHB_Clk_0();

    if (reload) {
        EF_CTRL_LOAD_BEFORE_READ_R0;
    }

    if (slot == 0) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        trim = (tmp >> 0) & 0x3ff;
    } else if (slot == 1) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        trim = (tmp >> 10) & 0x3ff;
    } else if (slot == 2) {
        tmp = BL_RD_REG(EF_DATA_BASE, EF_DATA_0_EF_KEY_SLOT_3_W3);
        trim = (tmp >> 20) & 0x3ff;
    }

    en = trim >> 9;
    parity = (trim >> 8) & 0x01;
    if (en == 1 && parity == EF_Ctrl_Get_Trim_Parity(trim, 8)) {
        *code = trim & 0xff;
        return SUCCESS;
    }
    return ERROR;
}
