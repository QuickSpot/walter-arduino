/**
 * @file WalterFeels.cpp
 * @author Daan Pape <daan@dptechnics.com>,
 *         Dries Vandenbussche <dries@dptechnics.com>
 * @date 11 Jul 2023
 * @copyright DPTechnics bv
 * @brief Walter Modem library
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
 * This file contains the Walter Feels expansion board library implementation.
 */

#include "WalterFeels.h"

#include <Wire.h>
#include <Arduino.h>

#include "LTC4015Registers.h"

#define I2C_BUS_POWER_PIN 1
#define I2C_SDA_PIN 42
#define I2C_SCL_PIN 2

#define LTC4015_ADDR 0xD0

const char *bat_chem_str[] = {"Li Ion", "LiFePO4", "Lead Acid"};
LTC4015::LTC4015(unsigned char _Rsnsi, unsigned char _Rsnsb):
    is_programmable(false),
    Rsnsi(_Rsnsi),
    Rsnsb(_Rsnsb)
{
}

void LTC4015::initialize() 
{
    suspend_charging();
    get_battery_info();
    Serial.println("Battery : " + (String)bat_chem_str[chemistry]);
    Serial.println("Cells : " + (String)cell_count);
}

void LTC4015::printinfo()
{
    get_battery_info();
    Serial.println("Battery                  : " + (String)bat_chem_str[chemistry]);
    Serial.println("Cells                    : " + (String)cell_count);

    int vcharge = read_word(VCHARGE_SETTING);
    Serial.printf("Charge voltage target:   : %.2fV\n", ((float) vcharge / 80) + 3.4125);

    int icharge = read_word(ICHARGE_TARGET);
    Serial.printf("Charge current target:   : %.2fA\n", ((float) icharge + 1) * 1.0/Rsnsb);
}

void LTC4015::set_input_voltage_limit(float hi, float lo) {
    write_word(VIN_LO_ALERT_LIMIT, 0);
    write_word(VIN_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_system_voltage_limit(float hi, float lo) {
    write_word(VSYS_LO_ALERT_LIMIT, 0);
    write_word(VSYS_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_battery_voltage_limit(float hi, float lo) {
    write_word(VBAT_LO_ALERT_LIMIT, 0);
    write_word(VBAT_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_input_current_limit(float hi) {
    write_word(IIN_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_battery_current_limit(float lo) {
    write_word(IBAT_LO_ALERT_LIMIT, 0);
}

void LTC4015::set_die_temp_limit(float hi) {
    write_word(DIE_TEMP_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_bsr_limit(float hi) {
    write_word(BSR_HI_ALERT_LIMIT, 0);
}

void LTC4015::set_ntc_limit(float hi, float lo) {
    write_word(NTC_RATIO_HI_ALERT_LIMIT, 0);
    write_word(NTC_RATIO_LO_ALERT_LIMIT, 0);
}

void LTC4015::set_qcount(u16_t qcount_val) {
    write_word(QCOUNT, qcount_val);
}

u16_t LTC4015::get_qcount() {
    return (u16_t) read_word(QCOUNT);
}

void LTC4015::set_coulomb_counter(u16_t qcount_init, u16_t prescale_factor, u16_t lo_alert_lim, u16_t hi_alert_lim) {
    write_word(QCOUNT, qcount_init);
    write_word(QCOUNT_PRESCALE_FACTOR, prescale_factor);
    write_word(QCOUNT_LO_ALERT_LIMIT, lo_alert_lim);
    write_word(QCOUNT_HI_ALERT_LIMIT, hi_alert_lim);
}

void LTC4015::en_meas_sys_valid_alert ()  {_set_bit(EN_LIMIT_ALERTS, meas_sys_valid);}
void LTC4015::dis_meas_sys_valid_alert () {_clr_bit(EN_LIMIT_ALERTS, meas_sys_valid);}
void LTC4015::en_qcount_lo_alert()        {_set_bit(EN_LIMIT_ALERTS, qcount_lo);}
void LTC4015::dis_qcount_lo_alert()       {_clr_bit(EN_LIMIT_ALERTS, qcount_lo);}
void LTC4015::en_qcount_hi_alert()        {_set_bit(EN_LIMIT_ALERTS, qcount_hi);}
void LTC4015::dis_qcount_hi_alert()       {_clr_bit(EN_LIMIT_ALERTS, qcount_hi);}
void LTC4015::en_vbat_lo_alert()          {_set_bit(EN_LIMIT_ALERTS, vbat_lo);}
void LTC4015::dis_vbat_lo_alert(){_clr_bit(EN_LIMIT_ALERTS, vbat_lo);}
void LTC4015::en_vbat_hi_alert(){_set_bit(EN_LIMIT_ALERTS, vbat_hi);}
void LTC4015::dis_vbat_hi_alert(){_clr_bit(EN_LIMIT_ALERTS, vbat_hi);}
void LTC4015::en_vin_lo_alert()    {_set_bit(EN_LIMIT_ALERTS, vin_lo);}
void LTC4015::dis_vin_lo_alert(){_clr_bit(EN_LIMIT_ALERTS, vin_lo);}
void LTC4015::en_vin_hi_alert(){_set_bit(EN_LIMIT_ALERTS, vin_hi);}
void LTC4015::dis_vin_hi_alert(){_clr_bit(EN_LIMIT_ALERTS, vin_hi);}
void LTC4015::en_vsys_lo_alert(){_set_bit(EN_LIMIT_ALERTS, vsys_lo);}
void LTC4015::dis_vsys_lo_alert(){_clr_bit(EN_LIMIT_ALERTS, vsys_lo);}
void LTC4015::en_vsys_hi_alert(){_set_bit(EN_LIMIT_ALERTS, vsys_hi);}
void LTC4015::dis_vsys_hi_alert(){_clr_bit(EN_LIMIT_ALERTS, vsys_hi);}
void LTC4015::en_iin_hi_alert(){_set_bit(EN_LIMIT_ALERTS, iin_hi);}
void LTC4015::dis_iin_hi_alert(){_clr_bit(EN_LIMIT_ALERTS, iin_hi);}
void LTC4015::en_ibat_lo_alert(){_set_bit(EN_LIMIT_ALERTS, ibat_lo);}
void LTC4015::dis_ibat_lo_alert(){_clr_bit(EN_LIMIT_ALERTS, ibat_lo);}
void LTC4015::en_die_temp_hi_alert(){_set_bit(EN_LIMIT_ALERTS,die_temp_hi);}
void LTC4015::dis_die_temp_hi_alert(){_clr_bit(EN_LIMIT_ALERTS,die_temp_hi);}
void LTC4015::en_bsr_hi_alert(){_set_bit(EN_LIMIT_ALERTS, bsr_hi);}
void LTC4015::dis_bsr_hi_alert(){_clr_bit(EN_LIMIT_ALERTS, bsr_hi);}
void LTC4015::en_ntc_ratio_hi_alert()     {}
void LTC4015::dis_ntc_ratio_hi_alert()    {}
void LTC4015::en_ntc_ratio_lo_alert()     {}
void LTC4015::dis_ntc_ratio_lo_alert()     {}
void LTC4015::enable_limit_alerts(unsigned short alert) {
    write_word(EN_LIMIT_ALERTS, alert);
}

void LTC4015::enable_charger_state_alerts (unsigned short alert) {
    write_word(EN_CHARGER_STATE_ALERTS, alert);
}

void LTC4015::en_uvcl_limit_alert() { _set_bit(EN_CHARGE_STATUS_ALERTS, vin_uvcl_active);  }
void LTC4015::dis_uvcl_limit_alert(){ _clr_bit(EN_CHARGE_STATUS_ALERTS, vin_uvcl_active);  }
void LTC4015::en_iin_limit_alert()  { _set_bit(EN_CHARGE_STATUS_ALERTS, iin_limit_active); }
void LTC4015::dis_iin_limit_alert() { _clr_bit(EN_CHARGE_STATUS_ALERTS, iin_limit_active); }
void LTC4015::en_cc_alert()         { _set_bit(EN_CHARGE_STATUS_ALERTS, constant_current); }
void LTC4015::dis_cc_alert()        { _clr_bit(EN_CHARGE_STATUS_ALERTS, constant_current); }
void LTC4015::en_cv_alert()         { _set_bit(EN_CHARGE_STATUS_ALERTS, constant_voltage); }
void LTC4015::dis_cv_alert()        { _clr_bit(EN_CHARGE_STATUS_ALERTS, constant_voltage); }
void LTC4015::enable_charger_status_alerts (unsigned short alert) {
    write_word(EN_CHARGE_STATUS_ALERTS, alert);
}

void LTC4015::suspend_charging()  { _set_bit(CONFIG_BITS, suspend_charger); }
void LTC4015::start_charging()    { _clr_bit(CONFIG_BITS, suspend_charger); }
void LTC4015::enable_mppt()       { _set_bit(CONFIG_BITS, mppt_en_i2c); }
void LTC4015::disable_mppt()      { _clr_bit(CONFIG_BITS, mppt_en_i2c); }
void LTC4015::enable_force_telemetry()       { _set_bit(CONFIG_BITS, force_meas_sys_on); }
void LTC4015::disable_force_telemetry()      { _clr_bit(CONFIG_BITS, force_meas_sys_on); }
void LTC4015::enable_coulomb_counter()       { _set_bit(CONFIG_BITS, en_qcount); }
void LTC4015::disable_coulomb_counter()      { _clr_bit(CONFIG_BITS, en_qcount); }
void LTC4015::config(unsigned short setting) {
    write_word(CONFIG_BITS, setting);
}

void LTC4015::set_input_current_max(float current) {
    unsigned short iin = (unsigned short) ((2 * current * Rsnsi) - 1);
    if(iin > 63) {
        return;
    }
    write_word(ICHARGE_TARGET, iin);
}

void LTC4015::set_input_voltage_min(float voltage) {


}

void LTC4015::set_charge_current(float current) {
    unsigned short icharge = (unsigned short) ((current * Rsnsb) - 1);
    if(icharge > 31) {
        return;
    }
    write_word(ICHARGE_TARGET, icharge);
}

void LTC4015::set_charge_voltage(float voltage) {
    float v_cell;

    v_cell = (voltage / cell_count);
    if (chemistry == LI_ION) {
        v_cell -= 3.8125;
        vcharge = (unsigned char)(v_cell * 80);
        if(vcharge > 31) {
            return;
        }
    }
    
    else if (chemistry == LIFEPO4) {
        v_cell -= 3.4125;
        vcharge = (unsigned char)(v_cell * 80);
        if(vcharge > 31) {
            return;
        }
    }
    
    else if(chemistry == LEAD_ACID) {
        v_cell -= 2.0;
        vcharge = (unsigned char)(v_cell * 105.0);
        if(vcharge > 63) {
            return;
        }
    }
    write_word(VCHARGE_SETTING, vcharge);
}

void set_equalize_voltage(float voltage) {
    unsigned short vequlize;
    float v_equalize_delta;
}

void LTC4015::arm_ship_mode() {

}

void LTC4015::charger_status() {

}

void LTC4015::charger_state() {

}

void LTC4015::charger_limit_alert() {

}

void LTC4015::get_battery_info() {
    unsigned short tmp = read_word(CHEM_CELLS);
    unsigned char t_chem = (tmp >> chem) & 0x0F;
    cell_count = tmp & 0x0F;
    if(t_chem < 4) { chemistry = LI_ION; }
    else if (t_chem < 7) { chemistry = LIFEPO4; }
    else if (t_chem < 9) { chemistry = LEAD_ACID; }
    else {chemistry = INVALID; }
    if((t_chem == 0) || (t_chem == 4) || (t_chem == 8)) {
        is_programmable = true;
    }
}

float LTC4015::get_input_voltage() {
    return (float)((signed short)read_word(VIN) * 0.001648);
}

float LTC4015::get_input_current() {
    return (float)((signed short)read_word(IIN) * 0.00146487 / Rsnsi); 
}

float LTC4015::get_system_voltage() {
    return (float)((signed short)read_word(VSYS) * 0.001648);
}

float LTC4015::get_battery_voltage() {
    short v_bat_cell = read_word(VBAT);
    float v_bat = 0.0;
    if(chemistry == LEAD_ACID) {
    v_bat = v_bat_cell * 0.000128176;
    }
    else if (chemistry != INVALID) {
    v_bat = v_bat_cell * 0.0001922;
    }
    else {
    
    }
    return (v_bat * cell_count);
}

float LTC4015::get_charge_current() {
    return (float)((signed short) read_word(IBAT) * 0.00146487 / Rsnsb);
}

float LTC4015::get_die_temp() {
    short rawdie = read_word(DIE_TEMP);
    return (float)((rawdie - 12010) / 45.6);
}


void LTC4015::write_word(unsigned char sub_address, unsigned short word) {
    Wire.beginTransmission(LTC4015_ADDR >> 1);
    Wire.write(sub_address);
    Wire.write(word & 0xFF);
    Wire.write((word >> 8) & 0xFF);
    Wire.endTransmission();
}

unsigned short LTC4015::read_word(unsigned char sub_address) {
    unsigned short word = 0;
    Wire.beginTransmission(LTC4015_ADDR >> 1);
    Wire.write(sub_address);
    Wire.endTransmission();

    Wire.requestFrom(LTC4015_ADDR >> 1, 2);
    word = (Wire.read() | (Wire.read() << 8));
    return word;
}

void LTC4015::_clr_bit(unsigned char sub_address, unsigned short new_word) {
    unsigned short old_word = read_word(sub_address);
    old_word &= ~new_word;
    write_word(sub_address, old_word);
}

void LTC4015::_set_bit(unsigned char sub_address, unsigned short new_word) {
    unsigned short old_word = read_word(sub_address);
    old_word |= new_word;
    write_word(sub_address, old_word);
}