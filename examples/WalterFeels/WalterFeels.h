/**
 * @file WalterFeels.h
 * @author Daan Pape <daan@dptechnics.com>
 *         Dries Vandenbussche <dries@dptechnics.com>
 * @date 16 Jan 2024
 * @copyright DPTechnics bv
 * @brief Walter Feels library
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
 * This file contains the headers of the Walter Feels expansion board library.
 */

#ifndef WALTER_FEELS_H
#define WALTER_FEELS_H

#include <Arduino.h>

enum bat_chem {
    LI_ION,
    LIFEPO4,
    LEAD_ACID,
    INVALID };
// const char *bat_chem_str[] = {"Li Ion", "LiFePO4", "Lead Acid"};

class LTC4015 {
    public:
        LTC4015 (unsigned char _Rsnsi, unsigned char _Rsnsb);

        void initialize();

        void printinfo();

        void set_input_voltage_limit(float hi, float lo);

        void set_system_voltage_limit(float hi, float lo);

        void set_battery_voltage_limit(float hi, float lo);

        void set_input_current_limit(float hi);

        void set_battery_current_limit(float lo);

        void set_die_temp_limit(float hi);

        void set_bsr_limit(float hi);

        void set_ntc_limit(float hi, float lo);

        void set_qcount(u16_t qcount);

        u16_t get_qcount();

        void set_coulomb_counter(u16_t qcount_init, u16_t prescale, u16_t lo_alert = 0, u16_t hi_alert = 0);

        void en_meas_sys_valid_alert ();
        void dis_meas_sys_valid_alert ();
        void en_qcount_lo_alert();
        void dis_qcount_lo_alert();
        void en_qcount_hi_alert();
        void dis_qcount_hi_alert();
        void en_vbat_lo_alert();
        void dis_vbat_lo_alert();
        void en_vbat_hi_alert();
        void dis_vbat_hi_alert();
        void en_vin_lo_alert();
        void dis_vin_lo_alert();
        void en_vin_hi_alert();
        void dis_vin_hi_alert();
        void en_vsys_lo_alert();
        void dis_vsys_lo_alert();
        void en_vsys_hi_alert();
        void dis_vsys_hi_alert();
        void en_iin_hi_alert();
        void dis_iin_hi_alert();
        void en_ibat_lo_alert();
        void dis_ibat_lo_alert();
        void en_die_temp_hi_alert();
        void dis_die_temp_hi_alert();
        void en_bsr_hi_alert();
        void dis_bsr_hi_alert();
        void en_ntc_ratio_hi_alert();
        void dis_ntc_ratio_hi_alert();
        void en_ntc_ratio_lo_alert();
        void dis_ntc_ratio_lo_alert();
        void enable_limit_alerts(unsigned short alert);

        void enable_charger_state_alerts (unsigned short alert);

        void en_uvcl_limit_alert();
        void dis_uvcl_limit_alert();
        void en_iin_limit_alert();
        void dis_iin_limit_alert();
        void en_cc_alert();
        void dis_cc_alert();
        void en_cv_alert();
        void dis_cv_alert();
        void enable_charger_status_alerts (unsigned short alert);

        void suspend_charging();
        void start_charging();
        void enable_mppt();
        void disable_mppt();
        void enable_force_telemetry();
        void disable_force_telemetry();
        void enable_coulomb_counter();
        void disable_coulomb_counter();

        void config(unsigned short setting);

        void set_input_current_max(float current);

        void set_input_voltage_min(float voltage);

        void set_charge_current(float current);

        void set_charge_voltage(float voltage);

        void arm_ship_mode();

        void charger_status();

        void charger_state();

        void charger_limit_alert();

        void get_battery_info();

        float get_input_voltage();

        float get_input_current();

        float get_system_voltage();

        float get_battery_voltage();

        float get_charge_current();
        float get_die_temp();

        unsigned short read_word(unsigned char sub_address);

    private:
        bat_chem chemistry;
        unsigned char cell_count;
        const unsigned char Rsnsi;
        const unsigned char Rsnsb;
        unsigned char vcharge;
        unsigned char icharge;

        bool is_programmable;

        void write_word(unsigned char sub_address, unsigned short word);

        void _clr_bit(unsigned char sub_address, unsigned short new_word);
        void _set_bit(unsigned char sub_address, unsigned short new_word);
};

// /**
//  * @brief Setup and configure the expansion board.
//  * 
//  * This function will initialize the peripherals on Walter feels and set up
//  * the charger.
//  * 
//  * @return True on success, false on error. 
//  */
// bool walterFeelsSetup();

#endif