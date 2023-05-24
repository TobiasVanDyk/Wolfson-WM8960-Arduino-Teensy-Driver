/******************************************************************************
 * File Name:   mtb_wm8960.c
 *
 * Description: This file contains the implementation of the WM8960 driver.
 *
 * Related Document: See README.md
 *
 *
 *******************************************************************************
 * $ Copyright 2021-YEAR Cypress Semiconductor $
 *******************************************************************************/
#include "mtb_wm8960.h"
#include "cyhal.h"

#define WM8960_I2C_ADDRESS          (0x1Au)
#define WM8960_TIMEOUT_MS           (100u)
#define REGISTER_MAP_SIZE           (56u)

typedef cy_rslt_t (* _mtb_wm8960_update_data_t)(mtb_wm8960_reg_t reg, uint16_t value);

typedef enum
{
    _WM8960_SYSCLK_FREQ_12288000_HZ = 12288000,
    _WM8960_SYSCLK_FREQ_11289600_HZ = 11289600
} _mtb_wm8960_sysclk_freq_t;

typedef struct
{
    mtb_wm8960_features_t features;
    mtb_wm8960_reg_t reg;
    uint16_t value;
} _mtb_wm8960_operation_t;

static cyhal_i2c_t* i2c_ptr = NULL;
static mtb_wm8960_features_t enabled_features;
static bool pll_enabled = false;

/* The WM8960 audio codec does not allow reading registers from the device so we
 * store a cached copy with default of the register map in the driver and is updated
 * on every write.
 */
static uint16_t wm8960_register_map[REGISTER_MAP_SIZE];

//--------------------------------------------------------------------------------------------------
// _mtb_wm8960_config_default
//--------------------------------------------------------------------------------------------------
static cy_rslt_t _mtb_wm8960_config_default(mtb_wm8960_features_t features)
{
    cy_rslt_t result;
    uint16_t value;

    /* Enable VREF and set VMID=50K */
    value = (WM8960_PWR_MGMT1_VREF_UP | WM8960_PWR_MGMT1_VMIDSEL_50K);
    if ((features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        /* AINL, AINR, ADCL, ADCR and MICB */
        value |= (WM8960_PWR_MGMT1_AINL_UP | WM8960_PWR_MGMT1_AINR_UP |
                  WM8960_PWR_MGMT1_ADCL_UP | WM8960_PWR_MGMT1_ADCR_UP |
                  WM8960_PWR_MGMT1_MICB_UP);
    }
    result = mtb_wm8960_write(WM8960_REG_PWR_MGMT1, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Enable DACL, DACR, LOUT1 and ROUT1 */
    if ((features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        result = mtb_wm8960_write(WM8960_REG_PWR_MGMT2, WM8960_PWR_MGMT2_DACL_UP  |
                                  WM8960_PWR_MGMT2_DACR_UP  |
                                  WM8960_PWR_MGMT2_LOUT1_UP |
                                  WM8960_PWR_MGMT2_ROUT1_UP);
        if (result != CY_RSLT_SUCCESS)
        {
            return result;
        }
    }

    value = 0;
    /* Enable left output mixer and right output mixer */
    if ((features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        value |= (WM8960_PWR_MGMT3_LOMIX_UP | WM8960_PWR_MGMT3_ROMIX_UP);
    }
    /* Enable left and right channel input PGA */
    if ((features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        value |= (WM8960_PWR_MGMT3_RMIC_UP | WM8960_PWR_MGMT3_LMIC_UP);
    }
    result = mtb_wm8960_write(WM8960_REG_PWR_MGMT3, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    static const _mtb_wm8960_operation_t operations[] =
    {
        /* LINPUT1 to PGA (LMN1), Connect left input PGA to left input boost (LMIC2B), Left PGA
           Boost = 0dB */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_ADCL_SIG_PTH,
          .value = (WM8960_ADCL_ADCR_SIG_PTH_MN1_CON | WM8960_ADCL_ADCR_SIG_PTH_MIC2B_CON) },
        /* RINPUT1 to PGA (RMN1), Connect right input PGA to right input boost (RMIC2B), Right PGA
           Boost = 0dB */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_ADCR_SIG_PTH,
          .value = (WM8960_ADCL_ADCR_SIG_PTH_MN1_CON | WM8960_ADCL_ADCR_SIG_PTH_MIC2B_CON) },
        /* Unmute left input PGA (LINMUTE), Left Input PGA Vol = 0dB, Volume Update */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_LEFT_IN_VOL,
          .value = (WM8960_LEFT_RIGHT_IN_VOL_IPVU | WM8960_LEFT_RIGHT_IN_VOL_INVOL_0dB) },
        /* Unmute right input PGA (RINMUTE), Right Input PGA Vol = 0dB, Volume Update */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_RIGHT_IN_VOL,
          .value = (WM8960_LEFT_RIGHT_IN_VOL_IPVU | WM8960_LEFT_RIGHT_IN_VOL_INVOL_0dB) },
        /* Left ADC Vol = 0dB, Volume Update */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_LEFT_ADC_VOL,
          .value = (WM8960_LEFT_RIGHT_ADC_VOL_ADCVU_UP | WM8960_LEFT_RIGHT_ADC_VOL_ADCVOL_0dB) },
        /* Right ADC Vol = 0dB, Volume Update */
        { .features = WM8960_FEATURE_MICROPHONE, .reg   = WM8960_REG_RIGHT_ADC_VOL,
          .value = (WM8960_LEFT_RIGHT_ADC_VOL_ADCVU_UP | WM8960_LEFT_RIGHT_ADC_VOL_ADCVOL_0dB) },

        /* Left DAC to left output mixed enabled (LD2LO), 0dB */
        { .features = WM8960_FEATURE_HEADPHONE,  .reg   = WM8960_REG_LEFT_OUT_MIX,
          .value = WM8960_LEFT_OUT_MIX_LD2LO_EN },
        /* Right DAC to right output mixed enabled (RD2RO), 0dB */
        { .features = WM8960_FEATURE_HEADPHONE,  .reg   = WM8960_REG_RIGHT_OUT_MIX,
          .value = WM8960_RIGHT_OUT_MIX_RD2RO_EN },
        /* LOUT1 Volume = 0dB, volume updated */
        { .features = WM8960_FEATURE_HEADPHONE,  .reg   = WM8960_REG_LOUT1_VOL,
          .value = (WM8960_LOUT1_ROUT1_VOL_OUT1VU | WM8960_LOUT1_ROUT1_VOL_OUT1VOL_0dB) },
        /* ROUT1 Volume = 0dB, volume updated */
        { .features = WM8960_FEATURE_HEADPHONE,  .reg   = WM8960_REG_ROUT1_VOL,
          .value = (WM8960_LOUT1_ROUT1_VOL_OUT1VU | WM8960_LOUT1_ROUT1_VOL_OUT1VOL_0dB) },
        /* Unmute DAC digital soft mute */
        { .features = WM8960_FEATURE_HEADPHONE,  .reg   = WM8960_REG_CTR1,
          .value = WM8960_CTR1_DACMU_NO },
    };

    for (uint32_t i = 0; i < sizeof(operations) / sizeof(_mtb_wm8960_operation_t); i++)
    {
        _mtb_wm8960_operation_t operation = operations[i];
        if ((features & operation.features) == operation.features)
        {
            result = mtb_wm8960_write(operation.reg, operation.value);
            if (result != CY_RSLT_SUCCESS)
            {
                return result;
            }
        }
    }

    enabled_features = features;

    return result;
}


//--------------------------------------------------------------------------------------------------
// _mtb_wm8960_setup_pll
//--------------------------------------------------------------------------------------------------
static cy_rslt_t _mtb_wm8960_setup_pll(uint32_t mclk_hz,
                                       mtb_wm8960_adc_dac_sample_rate_t sample_rate)
{
    cy_rslt_t result;
    uint8_t PLLN;
    uint32_t PLLK;
    bool use_prescale = false;
    uint32_t sys_clk_hz;

    /* Based on Table 40 on pg 61 in the WM8960 datasheet we infer
     * the SysClk frequency based on the selected sample rate.
     */
    switch (sample_rate)
    {
        case WM8960_ADC_DAC_SAMPLE_RATE_48_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_32_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_24_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_16_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_12_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_8_KHZ:
        default:
            sys_clk_hz = _WM8960_SYSCLK_FREQ_12288000_HZ;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_44_1_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_22_05_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_11_025_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_8_018_KHZ:
            sys_clk_hz = _WM8960_SYSCLK_FREQ_11289600_HZ;
            break;
    }

    /* Based on the PLL section on pg 63/64 the Sysclk divider
     * after the PLL must be 2. See Figure 36.
     *   f2 = 4 (Fixed divider in PLL) * 2 (SYSCLKDIV) * sysclk_hz
     *   f1 = mclk_hz
     *   R = f2 / f1
     *   PLLN = int (R)
     *   PLLK = int (2^24 (R - PLLN))
     */
    uint32_t f2 = (4 * 2 * sys_clk_hz);
    uint32_t f1 = mclk_hz;
    float R = (float)f2 / (float)f1;
    /* As per description of R52 the PLLN value must be between
     * 5 and 13. If the R value is less than 5 we use a prescale
     * divder (PLLPRESCALE) that divides the mclk_hz frequency by
     * 2 before its goes into the PLL.
     *
     * The datasheet also documents the peak stability is at
     * PLLN = 8 (pg 64) so if the R value is close to 5 and if using
     * the prescale divider gets us closer to 8 we enable the prescale
     * divider.
     */
    if ((R <= 5) || ((R*2 - 8) < (8 - R)))
    {
        use_prescale = true;
        R *= (float)2;
    }

    /* If the value of R is not within the permitted range then
     * mclk_hz freq cannot be used to generate a valid sysclk.
     */
    if ((R <= 5) || (R >= 13))
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    PLLN = (uint8_t)R;
    PLLK = (uint32_t)((float)0x1000000 * (R - (float)PLLN));
    uint16_t prescale_mask = (use_prescale)
                            ? WM8960_PLL_N_PLLPRESCALE_EN
                            : WM8960_PLL_N_PLLPRESCALE_DI;

    /* Update the PLLN register */
    result = mtb_wm8960_write(WM8960_REG_PLL_N, PLLN | prescale_mask | WM8960_PLL_N_SDM_FRAC);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Update PLLK1 with PLLK[23:16] */
    result = mtb_wm8960_write(WM8960_REG_PLL_K1, PLLK >> 16 & 0xFF);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Update PLLK2 with PLLK[15:8] */
    result = mtb_wm8960_write(WM8960_REG_PLL_K2, PLLK >> 8 & 0xFF);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Update PLLK3 with PLLK[7:0] */
    result = mtb_wm8960_write(WM8960_REG_PLL_K3, PLLK & 0xFF);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Enable the PLL in the power management register. */
    result = mtb_wm8960_set(WM8960_REG_PWR_MGMT2, WM8960_PWR_MGMT2_PLL_EN_UP);

    return result;
}


//--------------------------------------------------------------------------------------------------
// _mtb_wm8960_adjust_volume
//--------------------------------------------------------------------------------------------------
static cy_rslt_t _mtb_wm8960_adjust_volume(uint8_t volume, mtb_wm8960_reg_t left_vol_reg,
                                           mtb_wm8960_reg_t right_vol_reg,
                                           uint16_t update_bit, uint16_t volume_bits_mask)
{
    /* Volume adjustment is done based on documentation datasheet(rev 4.4) pg 45 */
    cy_rslt_t result;
    uint16_t data;

    result = mtb_wm8960_read(left_vol_reg, &data);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Clear the update and volume bits */
    data &= (~update_bit & ~volume_bits_mask);
    /* Set the new volume bits */
    data |= volume;

    result = mtb_wm8960_write(left_vol_reg, data);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = mtb_wm8960_read(right_vol_reg, &data);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Clear old volume bits */
    data &= ~volume_bits_mask;
    /* Set the new volume bits */
    data |= (volume | update_bit);
    /* Set right volume and update bits */
    result = mtb_wm8960_write(right_vol_reg, data);

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_init
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_init(cyhal_i2c_t* i2c_inst, mtb_wm8960_features_t features)
{
    /* Check I2C object if NULL */
    if (i2c_inst == NULL)
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    /* Assign I2C hal object */
    i2c_ptr = i2c_inst;

    cy_rslt_t result = mtb_wm8960_write(WM8960_REG_RESET, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /* Call default configuration */
    if (features != WM8960_FEATURE_NONE)
    {
        result = _mtb_wm8960_config_default(features);
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_free
//--------------------------------------------------------------------------------------------------
void mtb_wm8960_free(void)
{
    i2c_ptr = NULL;
    pll_enabled = false;
    enabled_features = WM8960_FEATURE_NONE;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_activate
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_activate(void)
{
    cy_rslt_t result;
    uint16_t value;

    /******************** PWR_MGMT1 *******************/
    result = mtb_wm8960_read(WM8960_REG_PWR_MGMT1, &value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    /* Clear VMIDSEL */
    value &= ~WM8960_PWR_MGMT1_VMIDSEL_5K;
    /* Set VMID=50K for playback and recording */
    value |= (WM8960_PWR_MGMT1_VMIDSEL_50K);
    if ((enabled_features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        /* AINL, AINR, ADCL, ADCR and MICB */
        value |= (WM8960_PWR_MGMT1_AINL_UP | WM8960_PWR_MGMT1_AINR_UP |
                  WM8960_PWR_MGMT1_ADCL_UP | WM8960_PWR_MGMT1_ADCR_UP |
                  WM8960_PWR_MGMT1_MICB_UP);
    }
    result = mtb_wm8960_write(WM8960_REG_PWR_MGMT1, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /******************** PWR_MGMT2 *******************/
    value = 0;
    if (pll_enabled)
    {
        value = WM8960_PWR_MGMT2_PLL_EN_UP;
    }
    /* Enable DACL, DACR, LOUT1 and ROUT1 */
    if ((enabled_features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        value |= (WM8960_PWR_MGMT2_DACL_UP  | WM8960_PWR_MGMT2_DACR_UP |
                  WM8960_PWR_MGMT2_LOUT1_UP | WM8960_PWR_MGMT2_ROUT1_UP);
    }
    result = mtb_wm8960_set(WM8960_REG_PWR_MGMT2, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /******************** PWR_MGMT3 *******************/
    value = 0;
    /* Enable left output mixer and right output mixer */
    if ((enabled_features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        value |= (WM8960_PWR_MGMT3_LOMIX_UP | WM8960_PWR_MGMT3_ROMIX_UP);
    }
    /* Enable left and right channel input PGA */
    if ((enabled_features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        value |= (WM8960_PWR_MGMT3_RMIC_UP | WM8960_PWR_MGMT3_LMIC_UP);
    }
    result = mtb_wm8960_set(WM8960_REG_PWR_MGMT3, value);

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_deactivate
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_deactivate(void)
{
    cy_rslt_t result;
    uint16_t value = 0;

    /******************** PWR_MGMT3 *******************/
    /* Disable left output mixer and right output mixer */
    if ((enabled_features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        value |= (WM8960_PWR_MGMT3_LOMIX_UP | WM8960_PWR_MGMT3_ROMIX_UP);
    }
    /* Disable left and right channel input PGA */
    if ((enabled_features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        value |= (WM8960_PWR_MGMT3_RMIC_UP | WM8960_PWR_MGMT3_LMIC_UP);
    }
    result = mtb_wm8960_clear(WM8960_REG_PWR_MGMT3, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /******************** PWR_MGMT2 *******************/
    result = mtb_wm8960_read(WM8960_REG_PWR_MGMT2, &value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    /* If PLL is enabled, disable the PLL. */
    if (pll_enabled)
    {
        value &= ~(WM8960_PWR_MGMT2_PLL_EN_UP);
    }
    if ((enabled_features & WM8960_FEATURE_HEADPHONE) == WM8960_FEATURE_HEADPHONE)
    {
        /* Disable DACL, DACR, LOUT1 and ROUT1 */
        value &= ~(WM8960_PWR_MGMT2_DACL_UP  | WM8960_PWR_MGMT2_DACR_UP |
                   WM8960_PWR_MGMT2_LOUT1_UP | WM8960_PWR_MGMT2_ROUT1_UP);
    }
    result = mtb_wm8960_write(WM8960_REG_PWR_MGMT2, value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    /******************** PWR_MGMT1 *******************/
    result = mtb_wm8960_read(WM8960_REG_PWR_MGMT1, &value);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }
    /* Clear VMIDSEL */
    value &= ~WM8960_PWR_MGMT1_VMIDSEL_5K;
    /* Set VMID=250K for low power standby */
    value |= WM8960_PWR_MGMT1_VMIDSEL_250K;
    if ((enabled_features & WM8960_FEATURE_MICROPHONE) == WM8960_FEATURE_MICROPHONE)
    {
        /* Disable AINL, AINR, ADCL, ADCR and MICB */
        value &= ~(WM8960_PWR_MGMT1_AINL_UP | WM8960_PWR_MGMT1_AINR_UP |
                   WM8960_PWR_MGMT1_ADCL_UP | WM8960_PWR_MGMT1_ADCR_UP |
                   WM8960_PWR_MGMT1_MICB_UP);
    }
    result = mtb_wm8960_write(WM8960_REG_PWR_MGMT1, value);

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_write
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_write(mtb_wm8960_reg_t reg, uint16_t value)
{
    if (reg >= REGISTER_MAP_SIZE)
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    cy_rslt_t result;

    uint8_t buf[] = { (reg << 1) | (CY_HI8(value) & 0x1), CY_LO8(value) };
    result = cyhal_i2c_master_write(i2c_ptr, WM8960_I2C_ADDRESS, buf, sizeof(buf),
                                    WM8960_TIMEOUT_MS, true);

    if (result == CY_RSLT_SUCCESS)
    {
        if (reg == WM8960_REG_RESET)
        {
            static const uint16_t wm8960_default_register_map[REGISTER_MAP_SIZE] =
            {
                0x0097, 0x0097, 0x0000, 0x0000, 0x0000, 0x0008, 0x0000, 0x000a, // R0~R7
                0x01c0, 0x0000, 0x00ff, 0x00ff, 0x0000, 0x0000, 0x0000, 0x0000, // R8~R15
                0x0000, 0x007b, 0x0100, 0x0032, 0x0000, 0x00c3, 0x00c3, 0x01c0, // R16~R23
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, // R24~R31
                0x0100, 0x0100, 0x0050, 0x0050, 0x0050, 0x0050, 0x0000, 0x0000, // R32~R39
                0x0000, 0x0000, 0x0040, 0x0000, 0x0000, 0x0050, 0x0050, 0x0000, // R40~R47
                0x0000, 0x0037, 0x004d, 0x0080, 0x0008, 0x0031, 0x0026, 0x00e9, // R48~R55
            };

            memcpy(&wm8960_register_map, &wm8960_default_register_map, REGISTER_MAP_SIZE);
        }
        else
        {
            /* Update register map */
            wm8960_register_map[reg] = value;
        }
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_read
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_read(mtb_wm8960_reg_t reg, uint16_t* value)
{
    if (reg >= REGISTER_MAP_SIZE)
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    *value = wm8960_register_map[reg];

    return CY_RSLT_SUCCESS;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_set
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_set(mtb_wm8960_reg_t reg, uint16_t mask)
{
    cy_rslt_t result;
    uint16_t data;

    result = mtb_wm8960_read(reg, &data);
    if (result == CY_RSLT_SUCCESS)
    {
        result = mtb_wm8960_write(reg, data | mask);
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_clear
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_clear(mtb_wm8960_reg_t reg, uint16_t mask)
{
    cy_rslt_t result;
    uint16_t data;

    result = mtb_wm8960_read(reg, &data);
    if (result == CY_RSLT_SUCCESS)
    {
        result = mtb_wm8960_write(reg, data & ~mask);
    }

    return result;
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_adjust_input_volume
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_adjust_input_volume(uint8_t volume)
{
    if (volume > WM8960_LEFT_RIGHT_IN_VOL_INVOL_30dB)
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    return _mtb_wm8960_adjust_volume(volume, WM8960_REG_LEFT_IN_VOL, WM8960_REG_RIGHT_IN_VOL,
                                     WM8960_LEFT_RIGHT_IN_VOL_IPVU,
                                     WM8960_LEFT_RIGHT_IN_VOL_INVOL_30dB);
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_adjust_heaphone_output_volume
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_adjust_heaphone_output_volume(uint8_t volume)
{
    if (volume > WM8960_LOUT1_ROUT1_VOL_OUT1VOL_6dB)
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    return _mtb_wm8960_adjust_volume(volume, WM8960_REG_LOUT1_VOL, WM8960_REG_ROUT1_VOL,
                                     WM8960_LOUT1_ROUT1_VOL_OUT1VU,
                                     WM8960_LOUT1_ROUT1_VOL_OUT1VOL_6dB);
}


//--------------------------------------------------------------------------------------------------
// mtb_wm8960_configure_clocking
//--------------------------------------------------------------------------------------------------
cy_rslt_t mtb_wm8960_configure_clocking(uint32_t mclk_hz, bool enable_pll,
                                        mtb_wm8960_adc_dac_sample_rate_t sample_rate,
                                        mtb_wm8960_word_length_t word_length,
                                        mtb_wm8960_mode_t mode)
{
    cy_rslt_t result;
    uint16_t dac_div_mask;
    uint16_t adc_div_mask;
    /* By default the mclk is used as sysclk (no pll) */
    uint16_t clk_sel_mask = WM8960_CLK1_CLKSEL_MCLK;
    /* By default the sysclk div is set to 1 */
    uint16_t sysclk_div_mask = WM8960_CLK1_SYSCLKDIV_BY_1;

    /* Only two clock frequencies are supported for mclk if PLL is not being used*/
    if (!enable_pll && (mclk_hz != _WM8960_SYSCLK_FREQ_12288000_HZ) &&
        (mclk_hz != _WM8960_SYSCLK_FREQ_11289600_HZ))
    {
        return MTB_RSLT_WM8960_BAD_ARG;
    }

    /* Check if using internal PLL */
    if (enable_pll)
    {
        result = _mtb_wm8960_setup_pll(mclk_hz, sample_rate);
        if (result != CY_RSLT_SUCCESS)
        {
            return result;
        }

        clk_sel_mask = WM8960_CLK1_CLKSEL_PLL;
        sysclk_div_mask = WM8960_CLK1_SYSCLKDIV_BY_2;
    }

    switch (sample_rate)
    {
        case WM8960_ADC_DAC_SAMPLE_RATE_48_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_44_1_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_1;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_1;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_32_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_1_5;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_1_5;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_24_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_22_05_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_2;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_2;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_16_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_3;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_3;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_11_025_KHZ:
        case WM8960_ADC_DAC_SAMPLE_RATE_12_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_4;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_4;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_8_018_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_5_5;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_5_5;
            break;

        case WM8960_ADC_DAC_SAMPLE_RATE_8_KHZ:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_6;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_6;
            break;

        default:
            dac_div_mask = WM8960_CLK1_DACDIV_BY_1;
            adc_div_mask = WM8960_CLK1_ADCDIV_BY_1;
            break;
    }

    pll_enabled = enable_pll;

    /* Set Clocking 1 */
    result = mtb_wm8960_write(WM8960_REG_CLK1,
                              adc_div_mask     |
                              dac_div_mask     |
                              sysclk_div_mask  |
                              clk_sel_mask);
    if (result != CY_RSLT_SUCCESS)
    {
        return result;
    }

    result = mtb_wm8960_write(WM8960_REG_AUDIO_INTF0,
                              (uint16_t)mode        |
                              (uint16_t)word_length |
                              WM8960_AUDIO_INTF0_FORMAT_I2S_MODE);

    return result;
}
