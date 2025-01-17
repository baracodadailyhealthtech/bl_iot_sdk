/**
  ******************************************************************************
  * @file    bl702l_aon.c
  * @version V1.0
  * @date
  * @brief   This file is the standard driver c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2020 Bouffalo Lab</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Bouffalo Lab nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "bl702l_aon.h"
#include "bl702l_ef_cfg.h"
#include "bl702l_ef_ctrl.h"

/** @addtogroup  BL702L_Peripheral_Driver
 *  @{
 */

/** @addtogroup  AON
 *  @{
 */

/** @defgroup  AON_Private_Macros
 *  @{
 */
#define AON_CLK_SET_DUMMY_WAIT \
    {                          \
        __NOP();               \
        __NOP();               \
        __NOP();               \
        __NOP();               \
        __NOP();               \
        __NOP();               \
        __NOP();               \
        __NOP();               \
    }

/*@} end of group AON_Private_Macros */

/** @defgroup  AON_Private_Types
 *  @{
 */

/*@} end of group AON_Private_Types */

/** @defgroup  AON_Private_Variables
 *  @{
 */

/*@} end of group AON_Private_Variables */

/** @defgroup  AON_Global_Variables
 *  @{
 */

/*@} end of group AON_Global_Variables */

/** @defgroup  AON_Private_Fun_Declaration
 *  @{
 */

/*@} end of group AON_Private_Fun_Declaration */

/** @defgroup  AON_Private_Functions
 *  @{
 */

/*@} end of group AON_Private_Functions */

/** @defgroup  AON_Public_Functions
 *  @{
 */

/****************************************************************************/ /**
 * @brief  Power on MXX band gap
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
#ifndef BFLB_USE_ROM_DRIVER
__WEAK BL_Err_Type ATTR_CLOCK_SECTION AON_Power_On_MBG(void)
{
    uint32_t tmpVal = 0;

    /* Power up RF for PLL to work */
    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_MBG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    BL702L_Delay_US(55);

    return SUCCESS;
}
#endif

/****************************************************************************/ /**
 * @brief  Power off MXX band gap
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
#ifndef BFLB_USE_ROM_DRIVER
__WEAK BL_Err_Type ATTR_CLOCK_SECTION AON_Power_Off_MBG(void)
{
    uint32_t tmpVal = 0;

    /* Power OFF */
    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_MBG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    return SUCCESS;
}
#endif

/****************************************************************************/ /**
 * @brief  Power on XTAL
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
#ifndef BFLB_USE_ROM_DRIVER
__WEAK BL_Err_Type ATTR_CLOCK_SECTION AON_Power_On_XTAL(void)
{
    uint32_t tmpVal = 0;
    uint32_t timeOut = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_XTAL_AON);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_XTAL_HF_RC32M_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    /* Polling for ready */
    do {
        BL702L_Delay_US(10);
        timeOut++;
        tmpVal = BL_RD_REG(AON_BASE, AON_TSEN);
    } while (!BL_IS_REG_BIT_SET(tmpVal, AON_XTAL_RDY) && timeOut < 120);

    if (timeOut >= 120) {
        return TIMEOUT;
    }

    return SUCCESS;
}
#endif

/****************************************************************************/ /**
 * @brief  Set XTAL cap code
 *
 * @param  capIn: Cap code in
 * @param  capOut: Cap code out
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
#ifndef BFLB_USE_ROM_DRIVER
__WEAK BL_Err_Type ATTR_CLOCK_SECTION AON_Set_Xtal_CapCode(uint8_t capIn, uint8_t capOut)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_XTAL_CFG2);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_XTAL_HF_CAPCODE_IN_AON, capIn);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_XTAL_HF_CAPCODE_OUT_AON, capOut);
    BL_WR_REG(AON_BASE, AON_XTAL_CFG2, tmpVal);

    BL702L_Delay_US(100);

    return SUCCESS;
}
#endif

/****************************************************************************/ /**
 * @brief  Get XTAL cap code
 *
 * @param  None
 *
 * @return Cap code
 *
*******************************************************************************/
uint8_t ATTR_CLOCK_SECTION AON_Get_Xtal_CapCode(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_XTAL_CFG2);

    return BL_GET_REG_BITS_VAL(tmpVal, AON_XTAL_HF_CAPCODE_IN_AON);
}

/****************************************************************************/ /**
 * @brief  Set XTAL cap code
 *
 * @param  extra: cap cpde extra aon
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_CLOCK_SECTION AON_Set_Xtal_CapCode_Extra(uint8_t extra)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_XTAL_CFG);
    if (extra) {
        tmpVal = BL_SET_REG_BIT(tmpVal, AON_XTAL_CAPCODE_EXTRA_AON);
    } else {
        tmpVal = BL_CLR_REG_BIT(tmpVal, AON_XTAL_CAPCODE_EXTRA_AON);
    }
    BL_WR_REG(AON_BASE, AON_XTAL_CFG, tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power off XTAL
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
#ifndef BFLB_USE_ROM_DRIVER
__WEAK BL_Err_Type ATTR_CLOCK_SECTION AON_Power_Off_XTAL(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_XTAL_AON);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_XTAL_HF_RC32M_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    return SUCCESS;
}
#endif

/****************************************************************************/ /**
 * @brief  Power on bandgap system
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_On_BG(void)
{
    uint32_t tmpVal = 0;

    /* power up RF for PLL to work */
    tmpVal = BL_RD_REG(AON_BASE, AON_BG_SYS_TOP);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_BG_SYS_AON);
    BL_WR_REG(AON_BASE, AON_BG_SYS_TOP, tmpVal);

    BL702L_Delay_US(55);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power off bandgap system
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_Off_BG(void)
{
    uint32_t tmpVal = 0;

    /* power up RF for PLL to work */
    tmpVal = BL_RD_REG(AON_BASE, AON_BG_SYS_TOP);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_BG_SYS_AON);
    BL_WR_REG(AON_BASE, AON_BG_SYS_TOP, tmpVal);

    BL702L_Delay_US(55);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power on LDO11
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_On_LDO11_SOC(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_LDO11SOC_TOP);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_LDO11SOC_AON);
    BL_WR_REG(AON_BASE, AON_LDO11SOC_TOP, tmpVal);

    BL702L_Delay_US(55);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power off LDO11
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_Off_LDO11_SOC(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_LDO11SOC_TOP);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_LDO11SOC_AON);
    BL_WR_REG(AON_BASE, AON_LDO11SOC_TOP, tmpVal);

    BL702L_Delay_US(55);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  power on source follow regular
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_On_SFReg(void)
{
    uint32_t tmpVal = 0;

    /* power on sfreg */
    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_SFREG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    BL702L_Delay_US(10);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  power off source follow regular
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Power_Off_SFReg(void)
{
    uint32_t tmpVal = 0;

    /* power off sfreg */
    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_SFREG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power off the power can be shut down in PDS0
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_LowPower_Enter_PDS0(void)
{
    uint32_t tmpVal = 0;

    /* power off bz */
    tmpVal = BL_RD_REG(AON_BASE, AON_MISC);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_SW_BZ_EN_AON);
    BL_WR_REG(AON_BASE, AON_MISC, tmpVal);

    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_SFREG_AON);
#if 0
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_LDO15RF_AON);
#endif
    tmpVal = BL_CLR_REG_BIT(tmpVal, AON_PU_MBG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    /* gating Clock, no more use */
    //tmpVal=BL_RD_REG(GLB_BASE,GLB_CGEN_CFG0);
    //tmpVal=tmpVal&(~(1<<6));
    //tmpVal=tmpVal&(~(1<<7));
    //BL_WR_REG(GLB_BASE,GLB_CGEN_CFG0,tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power on the power powered down in PDS0
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_LowPower_Exit_PDS0(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_RF_TOP_AON);

    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_MBG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    BL702L_Delay_US(20);

#if 0
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_LDO15RF_AON);
#endif
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    BL702L_Delay_US(60);

    tmpVal = BL_SET_REG_BIT(tmpVal, AON_PU_SFREG_AON);
    BL_WR_REG(AON_BASE, AON_RF_TOP_AON, tmpVal);

    BL702L_Delay_US(20);

    /* power on bz */
    tmpVal = BL_RD_REG(AON_BASE, AON_MISC);
    tmpVal = BL_SET_REG_BIT(tmpVal, AON_SW_BZ_EN_AON);
    BL_WR_REG(AON_BASE, AON_MISC, tmpVal);

    /* ungating Clock, no more use */
    //tmpVal=BL_RD_REG(GLB_BASE,GLB_CGEN_CFG0);
    //tmpVal=tmpVal|((1<<6));
    //tmpVal=tmpVal|((1<<7));
    //BL_WR_REG(GLB_BASE,GLB_CGEN_CFG0,tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  Power on the power powered down in PDS0
 *
 * @param  delay: None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Set_LDO11_SOC_Sstart_Delay(uint8_t delay)
{
    uint32_t tmpVal = 0;

    CHECK_PARAM((delay <= 0x3));

    /* config ldo11soc_sstart_delay_aon */
    tmpVal = BL_RD_REG(AON_BASE, AON_LDO11SOC_TOP);
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_LDO11SOC_SSTART_DELAY_AON, delay);
    BL_WR_REG(AON_BASE, AON_LDO11SOC_TOP, tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief set DCDC14 voltage and vpfm
 *
 * @param voutSel output voltage selection
 * @param vpfm pfm mode threshold
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type AON_Set_DCDC14_Top_0(uint8_t voutSel, uint8_t vpfm)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(AON_BASE, AON_DCDC18_TOP_0);
    //dcdc18_vout_sel_aon, 1.425V*1.05=1.5V
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_DCDC_VOUT_SEL_AON, voutSel);
    //dcdc18_vpfm_aon
    tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_DCDC_VPFM_AON, vpfm);
    BL_WR_REG(AON_BASE, AON_DCDC18_TOP_0, tmpVal);

    return SUCCESS;
}

/****************************************************************************/ /**
 * @brief  trim LDO11SOC vout
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Trim_Ldo11socVoutTrim(void)
{
    Efuse_Common_Trim_Type trim;
    int32_t tmpVal = 0;

    EF_Ctrl_Read_Common_Trim("ldo11_trim", &trim);
    if (trim.en) {
        if (trim.parity == EF_Ctrl_Get_Trim_Parity(trim.value, trim.len)) {
            tmpVal = BL_RD_REG(AON_BASE, AON_LDO11SOC_TOP);
            tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_LDO11SOC_VOUT_TRIM_AON, trim.value);
            BL_WR_REG(AON_BASE, AON_LDO11SOC_TOP, tmpVal);
            return SUCCESS;
        }
    }

    return ERROR;
}

/****************************************************************************/ /**
 * @brief  trim LDO14 vout
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Trim_Ldo14VoutTrim(void)
{
    Efuse_Common_Trim_Type trim;
    int32_t tmpVal = 0;

    EF_Ctrl_Read_Common_Trim("ldo14_trim", &trim);
    if (trim.en) {
        if (trim.parity == EF_Ctrl_Get_Trim_Parity(trim.value, trim.len)) {
            tmpVal = BL_RD_REG(AON_BASE, AON_LDO14_TOP);
            tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_LDO14_VOUT_TRIM_AON, trim.value);
            BL_WR_REG(AON_BASE, AON_LDO14_TOP, tmpVal);
            return SUCCESS;
        }
    }

    return ERROR;
}

/****************************************************************************/ /**
 * @brief  trim LDO14 vout
 *
 * @param  None
 *
 * @return SUCCESS or ERROR
 *
*******************************************************************************/
BL_Err_Type ATTR_TCM_SECTION AON_Trim_Dcdc14VoutTrim(void)
{
    Efuse_Common_Trim_Type trim;
    int32_t tmpVal = 0;

    EF_Ctrl_Read_Common_Trim("ldo14_trim", &trim);
    if (trim.en) {
        if (trim.parity == EF_Ctrl_Get_Trim_Parity(trim.value, trim.len)) {
            tmpVal = BL_RD_REG(AON_BASE, AON_DCDC_TOP_2);
            tmpVal = BL_SET_REG_BITS_VAL(tmpVal, AON_DCDC_VOUT_TRIM_AON, trim.value);
            BL_WR_REG(AON_BASE, AON_DCDC_TOP_2, tmpVal);
            return SUCCESS;
        }
    }

    return ERROR;
}
/*@} end of group AON_Public_Functions */

/*@} end of group AON */

/*@} end of group BL702L_Peripheral_Driver */
