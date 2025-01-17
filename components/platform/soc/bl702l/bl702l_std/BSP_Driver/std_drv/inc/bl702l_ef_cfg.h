/**
  ******************************************************************************
  * @file    bl702l_ef_cfg.h
  * @version V1.0
  * @date
  * @brief   This file is the standard driver header file
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
#ifndef __BL702L_EF_CFG_H__
#define __BL702L_EF_CFG_H__

#include "ef_ctrl_reg.h"
#include "bl702l_common.h"

/** @addtogroup  BL702L_Peripheral_Driver
 *  @{
 */

/** @addtogroup  EF_CFG
 *  @{
 */

/** @defgroup  EF_CFG_Public_Types
 *  @{
 */

/**
 *  @brief Efuse common trim config definition
 */
typedef struct
{
    char *name;           /*!< trim anme */
    uint16_t en_addr;     /*!< enable address */
    uint16_t parity_addr; /*!< parity address */
    uint16_t value_addr;  /*!< value address */
    uint16_t value_len;   /*!< value length */
} Efuse_Common_Trim_Cfg;

/**
 *  @brief Efuse common trim type definition
 */
typedef struct
{
    uint8_t en;     /*!< Enable status */
    uint8_t parity; /*!< Trim parity */
    uint8_t empty;  /*!< Trim empty */
    uint8_t len;    /*!< Trim value len in bit */
    uint32_t value; /*!< Trim value */
} Efuse_Common_Trim_Type;

/*@} end of group EF_CFG_Public_Types */

/** @defgroup  EF_CFG_Public_Constants
 *  @{
 */

/*@} end of group EF_CFG_Public_Constants */

/** @defgroup  EF_CFG_Public_Macros
 *  @{
 */

/*@} end of group EF_CFG_Public_Macros */

/** @defgroup  EF_CFG_Public_Functions
 *  @{
 */
uint8_t EF_Ctrl_Get_Trim_Parity(uint32_t val, uint8_t len);
uint32_t EF_Ctrl_Get_Common_Trim_List(const Efuse_Common_Trim_Cfg **trim_list);
void EF_Ctrl_Read_Common_Trim(char *name, Efuse_Common_Trim_Type *trim);
void EF_Ctrl_Write_Common_Trim(char *name, uint8_t trim_en, uint32_t trim_value);
uint8_t EF_Ctrl_Is_MAC_Address_Slot_Empty(uint8_t slot, uint8_t reload);
BL_Err_Type EF_Ctrl_Write_MAC_Address_Opt(uint8_t slot, uint8_t mac[8], uint8_t program);
BL_Err_Type EF_Ctrl_Read_MAC_Address_Opt(uint8_t slot, uint8_t mac[8], uint8_t reload);
/*@} end of group EF_CFG_Public_Functions */

/*@} end of group EF_CFG */

/*@} end of group BL702L_Peripheral_Driver */

#endif /* __BL702L_EF_CFG_H__ */
