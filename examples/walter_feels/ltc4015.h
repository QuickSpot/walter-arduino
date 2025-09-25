/**
 * @file ltc4015.h
 * @author Daan Pape <daan@dptechnics.com> Arnoud Devoogdt <arnoud@dptechnics.com>
 * @date 25 September 2025
 * @copyright DPTechnics bv
 * @brief Walter Modem library examples
 *
 * @section LICENSE
 *
 * Copyright (C) 2023, DPTechnics bv
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *   3. Neither the name of DPTechnics bv nor the names of its contributors may
 *      be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 *   4. This software, with or without modification, must only be used with a
 *      Walter board from DPTechnics bv.
 *
 *   5. Any software provided in binary form under this license must not be
 *      reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY DPTECHNICS BV “AS IS” AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL DPTECHNICS BV OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * This file contains drivers for the Walter Feels carrier board
 */

#ifndef _LTC4015_H_
#define _LTC4015_H_

#include <inttypes.h>

/**
 * @defgroup The addresses of the I2C devices on the Walter Feels board.
 * @{
 */
#define WFEELS_ADDR_LTC4015 0xD0
/**
 * @}
 */

/**
 * @defgroup LTC4015_REG Analog Devices LTC4015 register map.
 * @{
 */
#define LTC4015_REG_VBAT_LO_ALERT_LIMIT 0x01
#define LTC4015_REG_VBAT_HI_ALERT_LIMIT 0x02
#define LTC4015_REG_VIN_LO_ALERT_LIMIT 0x03
#define LTC4015_REG_VIN_HI_ALERT_LIMIT 0x04
#define LTC4015_REG_VSYS_LO_ALERT_LIMIT 0x05
#define LTC4015_REG_VSYS_HI_ALERT_LIMIT 0x06
#define LTC4015_REG_IIN_HI_ALERT_LIMIT 0x07
#define LTC4015_REG_IBAT_LO_ALERT_LIMIT 0x08
#define LTC4015_REG_DIE_TEMP_HI_ALERT_LIMIT 0x09
#define LTC4015_REG_BSR_HI_ALERT_LIMIT 0x0A
#define LTC4015_REG_NTC_RATIO_HI_ALERT_LIMIT 0x0B
#define LTC4015_REG_NTC_RATIO_LO_ALERT_LIMIT 0x0C
#define LTC4015_REG_EN_LIMIT_ALERTS 0x0D
#define LTC4015_REG_EN_CHARGER_STATE_ALERTS 0x0E
#define LTC4015_REG_EN_CHARGE_STATUS_ALERTS 0x0F
#define LTC4015_REG_QCOUNT_LO_ALERT_LIMIT 0x10
#define LTC4015_REG_QCOUNT_HI_ALERT_LIMIT 0x11
#define LTC4015_REG_QCOUNT_PRESCALE_FACTOR 0x12
#define LTC4015_REG_QCOUNT 0x13
#define LTC4015_REG_CONFIG_BITS 0x14
#define LTC4015_REG_IIN_LIMIT_SETTING 0x15
#define LTC4015_REG_VIN_UVCL_SETTING 0x16
#define LTC4015_REG_ARM_SHIP_MODE 0x19
#define LTC4015_REG_ICHARGE_TARGET 0x1A
#define LTC4015_REG_VCHARGE_SETTING 0x1B
#define LTC4015_REG_C_OVER_X_THRESHOLD 0x1C
#define LTC4015_REG_MAX_CV_TIME 0x1D
#define LTC4015_REG_MAX_CHARGE_TIME 0x1E
#define LTC4015_REG_JEITA_T1 0x1F
#define LTC4015_REG_JEITA_T2 0x20
#define LTC4015_REG_JEITA_T3 0x21
#define LTC4015_REG_JEITA_T4 0x22
#define LTC4015_REG_JEITA_T5 0x23
#define LTC4015_REG_JEITA_T6 0x24
#define LTC4015_REG_VCHARGE_JEITA_6_5 0x25
#define LTC4015_REG_VCHARGE_JEITA_4_3_2 0x26
#define LTC4015_REG_ICHARGE_JEITA_6_5 0x27
#define LTC4015_REG_ICHARGE_JEITA_4_3_2 0x28
#define LTC4015_REG_CHARGER_CONFIG_BITS 0x29
#define LTC4015_REG_VABSORB_DELTA 0x2A
#define LTC4015_REG_MAX_ABSORB_TIME 0x2B
#define LTC4015_REG_VEQUALIZE_DELTA 0x2C
#define LTC4015_REG_EQUALIZE_TIME 0x2D
#define LTC4015_REG_LIFEP04_RECHARGE_THRESHOLD 0x2E
#define LTC4015_REG_MAX_CHARGE_TIMER 0x30
#define LTC4015_REG_CV_TIMER 0x31
#define LTC4015_REG_ABSORB_TIMER 0x32
#define LTC4015_REG_EQUALIZE_TIMER 0x33
#define LTC4015_REG_CHARGER_STATE 0x34
#define LTC4015_REG_CHARGE_STATUS 0x35
#define LTC4015_REG_LIMIT_ALERTS 0x36
#define LTC4015_REG_CHARGER_STATE_ALERTS 0x37
#define LTC4015_REG_CHARGE_STATUS_ALERTS 0x38
#define LTC4015_REG_SYSTEM_STATUS 0x39
#define LTC4015_REG_VBAT 0x3A
#define LTC4015_REG_VIN 0x3B
#define LTC4015_REG_VSYS 0x3C
#define LTC4015_REG_IBAT 0x3D
#define LTC4015_REG_IIN 0x3E
#define LTC4015_REG_DIE_TEMP 0x3F
#define LTC4015_REG_NTC_RATIO 0x40
#define LTC4015_REG_BSR 0x41
#define LTC4015_REG_JEITA_REGION 0x42
#define LTC4015_REG_CHEM_CELLS 0x43
#define LTC4015_REG_ICHARGE_DAC 0x44
#define LTC4015_REG_VCHARGE_DAC 0x45
#define LTC4015_REG_IIN_LIMIT_DAC 0x46
#define LTC4015_REG_VBAT_FILT 0x47
#define LTC4015_REG_ICHARGE_BSR 0x48
#define LTC4015_REG_MEAS_SYS_VALID 0x4A
/**
 * @}
 */

/* Charger control bits */
#define suspend_charger (1 << 8)   // Suspend battery charger operation
#define run_bsr (1 << 5)           // Perform a battery series resistance measurement
#define force_meas_sys_on (1 << 4) // Force measurement system to operate
#define mppt_en_i2c (1 << 3)       // Enable maximum power point tracking
#define en_qcount (1 << 2)         // Enable coulomb counter

/* JEITA profile voltage and current settings */
#define vcharge_jeita_6 5
#define vcharge_jeita_5 0
#define vcharge_jeita_4 10
#define vcharge_jeita_3 5
#define vcharge_jeita_2 0
#define icharge_jeita_6 5
#define icharge_jeita_5 0
#define icharge_jeita_4 10
#define icharge_jeita_3 5
#define icharge_jeita_2 0

/* Charger configuration bits */
#define en_c_over_x_term (1 << 2)       // Enable C/x termination
#define en_lead_acid_temp_comp (1 << 1) // Enable lead-acid charge voltage temperature compensation
#define en_jeita (1 << 0)               // Enable JEITA temperature profile

/* Charger state bits */
#define equalize_charge (1 << 10)      // Battery charger is in lead-acid equalization charge state
#define absorb_charge (1 << 9)         // Battery charger is in absorb charge state
#define charger_suspended (1 << 8)     // Battery charger is in suspended state
#define precharge (1 << 7)             // Battery charger is in precondition charge state
#define cc_cv_charge (1 << 6)          // Battery charger is in CC-CV state
#define ntc_pause (1 << 5)             // Battery charger is in thermistor pause state
#define timer_term (1 << 4)            // Battery charger is in timer termination state
#define c_over_x_term (1 << 3)         // Battery charger is in C/x termination state
#define max_charge_time_fault (1 << 2) // Battery charger is in max charge time fault state
#define bat_missing_fault (1 << 1)     // Battery charger is in missing battery fault state
#define bat_short_fault (1 << 0)       // Battery charger is in shorted battery fault state

/* Charge status bits */
#define vin_uvcl_active (1 << 3)  // Input undervoltage control loop is active
#define iin_limit_active (1 << 2) // Input current limit control loop is active
#define constant_current (1 << 1) // Charge current control loop is active
#define constant_voltage (1 << 0) // Battery voltage control loop is active

/* Limit alerts */
#define meas_sys_valid (1 << 15) // Measurement system results are valid
#define qcount_lo (1 << 13)      // QCOUNT has fallen below QCOUNT_LO_ALERT_LIMIT
#define qcount_hi (1 << 12)      // QCOUNT has exceeded QCOUNT_HI_ALERT_LIMIT
#define vbat_lo (1 << 11)        // VBAT has fallen below VBAT_LO_ALERT_LIMIT
#define vbat_hi (1 << 10)        // VBAT has exceeded VBAT_HI_ALERT_LIMIT
#define vin_lo (1 << 9)          // VIN has fallen below VIN_LO_ALERT_LIMIT
#define vin_hi (1 << 8)          // VIN has exceeded VIN_HI_ALERT_LIMIT
#define vsys_lo (1 << 7)         // VSYS has fallen below VSYS_LO_ALERT_LIMIT
#define vsys_hi (1 << 6)         // VSYS has exceeded VSYS_HI_ALERT_LIMIT
#define iin_hi (1 << 5)          // IIN has exceeded IIN_HI_ALERT_LIMIT
#define ibat_lo (1 << 4)         // IBAT has fallen below IBAT_LO_ALERT_LIMIT
#define die_temp_hi (1 << 3)     // DIE_TEMP has exceeded DIE_TEMP_HI_ALERT_LIMIT
#define bsr_hi (1 << 2)          // BSR has exceeded BSR_HI_ALERT_LIMIT
#define ntc_ratio_hi (1 << 1)    // NTC_RATIO has exceeded NTC_RATIO_HI_ALERT_LIMIT
#define ntc_ratio_lo (1 << 0)    // NTC_RATIO has fallen below NTC_RATIO_LO_ALERT_LIMIT

/* System status bits */
#define charger_enabled (1 << 13) // Battery charger is active
#define mppt_en_pin (1 << 11)     // MPPT_EN pin is set to enable maximum power point tracking
#define equalize_req (1 << 10)    // Lead-acid equalize charge is queued
#define drvcc_good (1 << 9)       // DRVCC voltage is above undervoltage lockout level
#define cell_count_error (1 << 8) // Invalid combination of CELLS pin settings
#define ok_to_charge (1 << 6)     // All system conditions are met for battery charger operation
#define no_rt (1 << 5)            // No resistor detected at the RT pin
#define thermal_shutdown (1 << 4) // Die temperature exceeds thermal shutdown level
#define vin_ovlo (1 << 3)         // VIN voltage exceeds overvoltage lockout level
#define vin_gt_vbat (1 << 2)      // VIN voltage is sufficiently greater than BATSENS
#define intvcc_gt_4p3v (1 << 1)   // INTVCC voltage is above 4.3V
#define intvcc_gt_2p8v (1 << 0)   // INTVCC voltage is above 2.8V

/* Battery configuration */
#define chem 8            // Programmed battery chemistry
#define cell_count_pins 0 // Cell count as set by CELLS pins

enum bat_chem { LI_ION, LIFEPO4, LEAD_ACID, INVALID };
// const char *bat_chem_str[] = {"Li Ion", "LiFePO4", "Lead Acid"};

class LTC4015
{
public:
  static void initialize(unsigned char _Rsnsi, unsigned char _Rsnsb);

  static void printinfo();

  static void set_vin_uvcl_setting(uint8_t code);

  static void set_input_voltage_limit(float hi, float lo);

  static void set_system_voltage_limit(float hi, float lo);

  static void set_battery_voltage_limit(float hi, float lo);

  static void set_input_current_limit(float hi);

  static void set_battery_current_limit(float lo);

  static void set_die_temp_limit(float hi);

  static void set_bsr_limit(float hi);

  static void set_ntc_limit(float hi, float lo);

  static void set_qcount(uint16_t qcount);

  static uint16_t get_qcount();

  static void set_coulomb_counter(uint16_t qcount_init, uint16_t prescale, uint16_t lo_alert = 0,
                                  uint16_t hi_alert = 0);

  static void en_meas_sys_valid_alert();
  static void dis_meas_sys_valid_alert();
  static void en_qcount_lo_alert();
  static void dis_qcount_lo_alert();
  static void en_qcount_hi_alert();
  static void dis_qcount_hi_alert();
  static void en_vbat_lo_alert();
  static void dis_vbat_lo_alert();
  static void en_vbat_hi_alert();
  static void dis_vbat_hi_alert();
  static void en_vin_lo_alert();
  static void dis_vin_lo_alert();
  static void en_vin_hi_alert();
  static void dis_vin_hi_alert();
  static void en_vsys_lo_alert();
  static void dis_vsys_lo_alert();
  static void en_vsys_hi_alert();
  static void dis_vsys_hi_alert();
  static void en_iin_hi_alert();
  static void dis_iin_hi_alert();
  static void en_ibat_lo_alert();
  static void dis_ibat_lo_alert();
  static void en_die_temp_hi_alert();
  static void dis_die_temp_hi_alert();
  static void en_bsr_hi_alert();
  static void dis_bsr_hi_alert();
  static void en_ntc_ratio_hi_alert();
  static void dis_ntc_ratio_hi_alert();
  static void en_ntc_ratio_lo_alert();
  static void dis_ntc_ratio_lo_alert();
  static void enable_limit_alerts(unsigned short alert);

  static void enable_charger_state_alerts(unsigned short alert);

  static void en_uvcl_limit_alert();
  static void dis_uvcl_limit_alert();
  static void en_iin_limit_alert();
  static void dis_iin_limit_alert();
  static void en_cc_alert();
  static void dis_cc_alert();
  static void en_cv_alert();
  static void dis_cv_alert();
  static void enable_charger_status_alerts(unsigned short alert);

  static void suspend_charging();
  static void start_charging();
  static void enable_mppt();
  static void disable_mppt();
  static void enable_force_telemetry();
  static void disable_force_telemetry();
  static void enable_coulomb_counter();
  static void disable_coulomb_counter();

  static void config(unsigned short setting);

  static void set_input_current_max(float current);

  static void set_input_voltage_min(float voltage);

  static void set_charge_current(float current);

  static void set_charge_voltage(float voltage);

  static void arm_ship_mode();

  static void charger_status();

  static void charger_state();

  static void charger_limit_alert();

  static void get_battery_info();

  static float get_input_voltage();

  static float get_input_current();

  static float get_system_voltage();

  static float get_battery_voltage();

  static float get_charge_current();
  static float get_die_temp();

  static unsigned short read_word(uint8_t sub_address);

private:
  static inline bat_chem _chemistry;
  static inline unsigned char _cell_count;
  static inline unsigned char _rsnsi;
  static inline unsigned char _rsnsb;
  static inline unsigned char _vcharge;
  static inline unsigned char _icharge;

  static inline bool _is_programmable;

  static void _write_word(unsigned char sub_address, unsigned short word);

  static void _clr_bit(unsigned char sub_address, unsigned short new_word);
  static void _set_bit(unsigned char sub_address, unsigned short new_word);
};
;

#endif