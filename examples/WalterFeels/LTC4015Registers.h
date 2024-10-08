#ifndef _LTC4015_REGISTERS_H_
#define _LTC4015_REGISTERS_H_

#define VBAT_LO_ALERT_LIMIT 0x01 // Battery voltage low alert limit
#define VBAT_HI_ALERT_LIMIT 0x02 // Battery voltage high alert limit
#define VIN_LO_ALERT_LIMIT 0x03 // Input voltage low alert limit
#define VIN_HI_ALERT_LIMIT 0x04 // Input voltage high alert limit
#define VSYS_LO_ALERT_LIMIT 0x05 // Output voltage low alert limit
#define VSYS_HI_ALERT_LIMIT 0x06 // Output voltage high alert limit
#define IIN_HI_ALERT_LIMIT 0x07 // Input current high alert limit
#define IBAT_LO_ALERT_LIMIT 0x08 // Charge current low alert limit
#define DIE_TEMP_HI_ALERT_LIMIT 0x09 // Die temperature high alert limit
#define BSR_HI_ALERT_LIMIT  0x0A // Battery series resistance high alert limit
#define NTC_RATIO_HI_ALERT_LIMIT 0x0B // Thermistor ratio high (cold battery) alert limit
#define NTC_RATIO_LO_ALERT_LIMIT 0x0C // Thermistor ratio low (hot battery) alert limit

#define EN_LIMIT_ALERTS 0x0D // Enable limit monitoring and alert notification via SMBALERT 
#define EN_CHARGER_STATE_ALERTS 0x0E // Enable charger state alert notification via SMBALERT 
#define EN_CHARGE_STATUS_ALERTS 0x0F // Enable charge status alert notification via SMBALERT 

#define QCOUNT_LO_ALERT_LIMIT 0x10 // Coulomb counter QCOUNT low alert limit
#define QCOUNT_HI_ALERT_LIMIT 0x11 // Coulomb counter QCOUNT high alert limit
#define QCOUNT_PRESCALE_FACTOR 0x12 // Coulomb counter prescale factor 0x0200 
#define QCOUNT 0x13 // Coulomb counter value 0x8000 

#define CONFIG_BITS 0x14 // Configuration Settings  

#define IIN_LIMIT_SETTING 0x15 // Input current limit setting = (IIN_LIMIT_SETTING + 1)x500uV / Rsnsi
#define VIN_UVCL_SETTING 0x16 // UVCLFB input undervoltage limit = (VIN_UVCL_SETTING + 1)x4.75mV
#define ARM_SHIP_MODE 0x19 // Write 0x534D to arm ship mode. Once armed, ship mode cannot be disarmed.
#define ICHARGE_TARGET 0x1A // Maximum charge current target = (ICHARGE_TARGET + 1)x1mV/RSNSB
#define VCHARGE_SETTING 0x1B // Charge voltage target.
#define C_OVER_X_THRESHOLD 0x1C // Two's complement Low IBAT threshold for C/x termination
#define MAX_CV_TIME 0x1D // Time in seconds with battery charger in the CV state before timer termination occurs (lithium chemistries only)
#define MAX_CHARGE_TIME 0x1E // Time in seconds before a max_charge_time fault is declared. Set to zero to disable max_charge_time fault
#define JEITA_T1 0x1F // Value of NTC_RATIO for transition between JEITA regions 2 and 1 (off) 0x3F00
#define JEITA_T2 0x20 // Value of NTC_RATIO for transition between JEITA regions 3 and 2 0x372A
#define JEITA_T3 0x21 // Value of NTC_RATIO for transition between JEITA regions 4 and 3 0x1F27
#define JEITA_T4 0x22 // Value of NTC_RATIO for transition between JEITA regions 5 and 4 0x1BCC
#define JEITA_T5 0x23 // Value of NTC_RATIO for transition between JEITA regions 6 and 5 0x18B9
#define JEITA_T6 0x24 // Value of NTC_RATIO for transition between JEITA regions 7 (off) and 6 0x136D
#define VCHARGE_JEITA_6_5 0x25 // VCHARGE values for JEITA temperature regions 6 and 5
#define VCHARGE_JEITA_4_3_2 0x26 // VCHARGE values for JEITA temperature regions 4, 3, and 2
#define ICHARGE_JEITA_6_5 0x27 // ICHARGE_TARGET values for JEITA temperature regions 6 and 5 0x01EF
#define ICHARGE_JEITA_4_3_2 0x28 // ICHARGE_TARGET value for JEITA temperature regions 4, 3, and 20x7FEF
#define CHARGER_CONFIG_BITS 0x29 // Battery charger configuration settings, bits 15:3 are reserved.
#define VABSORB_DELTA 0x2A // LiFePO4/lead-acid absorb voltage adder, bits 15:6 are reserved.
#define MAX_ABSORB_TIME 0x2B // Maximum time for LiFePO4/lead-acid absorb charge
#define VEQUALIZE_DELTA 0x2C // Lead-acid equalize charge voltage adder, bits 15:6 are reserved. 0x002A
#define EQUALIZE_TIME 0x2D // Lead-acid equalization time 0x0E10 
#define LIFEP04_RECHARGE_THRESHOLD 0x2E // LiFeP04 recharge threshold0x4410 

#define MAX_CHARGE_TIMER 0x30 // For lithium chemistries, indicates the time (in sec) that the battery has been charging
#define CV_TIMER 0x31 // For lithium chemistries, indicates the time (in sec) that the battery has been in constant-voltage regulation
#define ABSORB_TIMER 0x32 // For LiFePO4 and lead-acid batteries, indicates the time (in sec) that the battery has been in absorb phase
#define EQUALIZE_TIMER 0x33 // For lead-acid batteries, indicates the time (in sec) that the battery has been in EQUALIZE phase
#define CHARGER_STATE 0x34 // Real time battery charger state indicator. Individual bits are mutually exclusive. Bits 15:11 are reserved.
#define CHARGE_STATUS 0x35 // Charge status indicator. Individual bits are mutually exclusive. Only active in charging states.
#define LIMIT_ALERTS 0x36 // Limit alert register.
#define CHARGER_STATE_ALERTS 0x37 // Charger state alert register.
#define CHARGE_STATUS_ALERTS 0x38 // Alerts that CHARGE_STATUS indicators have occurred
#define SYSTEM_STATUS 0x39 // Real time system status indicator bits
#define VBAT 0x3A // Two's complement ADC measurement result for the BATSENS pin.
     // VBATSENS/cellcount = [VBAT] x 192.2uV for lithium chemistries.
     // VBATSENS/cellcount = [VBAT] x 128.176uV for lead-acid.
#define VIN 0x3B // Two's complement ADC measurement result for VIN.
 // VIN = [VIN] x 1.648mV 
#define VSYS 0x3C // Two's complement ADC measurement result for VSYS.
 // VSYS = [VSYS] x 1.648mV
#define IBAT 0x3D // Two's complement ADC measurement result for (VCSP -VCSN).
 // Charge current (into the battery) is represented as a positive number.
 // Battery current = [IBAT] x 1.46487uV/Rsnsb
#define IIN 0x3E // Two's complement ADC measurement result for (VCLP -VCLN).
 // Input current = [IIN] x 1.46487uV/Rsnsi
#define DIE_TEMP 0x3F // Two's complement ADC measurement result for die temperature.
 // Temperature = (DIE_TEMP -12010)/45.6 °C
#define NTC_RATIO 0x40 // Two's complement ADC measurement result for NTC thermistor ratio.
 // Rntc = NTC_RATIO x Rntcbias/(21845.0 -NTC_RATIO)
#define BSR 0x41 // Calculated battery series resistance.
 // For lithium chemistries, series resistance/cellcount = BSR x Rsnsb/500.0
 // For lead-acid chemistries, series resistance/cellcount = BSR x RSNSB/750.0
#define JEITA_REGION 0x42 // JEITA temperature region of the NTC thermistor (Li Only). Active only when EN_JEITA=1
#define CHEM_CELLS 0x43 // Readout of CHEM and CELLS pin settings
#define ICHARGE_DAC 0x44 // Charge current control DAC control bits
#define VCHARGE_DAC 0x45 // Charge voltage control DAC control bits
#define IIN_LIMIT_DAC 0x46 // Input current limit control DAC control word
#define VBAT_FILT 0x47 // Digitally filtered two's complement ADC measurement result for battery voltage
#define ICHARGE_BSR 0x48 // This 16-bit two's complement word is the value of IBAT (0x3D) used in calculating BSR.
#define MEAS_SYS_VALID 0x4A // Measurement valid bit, bit 0 is a 1 when the telemetry(ADC) system is ready
 
#define suspend_charger (1 << 8)  // suspend battery charger operation
#define run_bsr (1 << 5)  // perform a battery series resistance measurement
#define force_meas_sys_on (1 << 4)  // force measurement system to operate
#define mppt_en_i2c (1 << 3)  // enable maximum power point tracking
#define en_qcount (1 << 2)  // enable coulomb counter

#define vcharge_jeita_6 5 
#define vcharge_jeita_5 0
#define vcharge_jeita_4     10 
#define vcharge_jeita_3 5 
#define vcharge_jeita_2 0 
#define icharge_jeita_6 5 
#define icharge_jeita_5 0
#define icharge_jeita_4 10 
#define icharge_jeita_3 5 
#define icharge_jeita_2 0 

#define en_c_over_x_term (1 << 2)  // enable C/x termination
#define en_lead_acid_temp_comp (1 << 1)  // enable lead-acid charge voltage temperature compensation
#define en_jeita (1 << 0)  // enable jeita temperature profile

/* Charger state */
#define equalize_charge (1 << 10) // indicates battery charger is in lead-acid equalization charge state
#define absorb_charge (1 << 9)  // indicates battery charger is in absorb charge state
#define charger_suspended (1 << 8)  // indicates battery charger is in charger suspended state
#define precharge (1 << 7)  // indicates battery charger is in precondition charge state
#define cc_cv_charge (1 << 6)  // indicates battery charger is in cc-cv state
#define ntc_pause (1 << 5)  // indicates battery charger is in thermistor pause state
#define timer_term (1 << 4)  // indicates battery charger is in timer termination state
#define c_over_x_term (1 << 3)  // indicates battery charger is in C/x termination state
#define max_charge_time_fault   (1 << 2)  // indicates battery charger is in max_charge_time_fault state
#define bat_missing_fault (1 << 1)  // indicates battery charger is in missing battery fault state
#define bat_short_fault (1 << 0)  // indicates battery charger is in shorted battery fault state

/* Charge status */
#define vin_uvcl_active (1 << 3)  // indicates the input undervoltage control loop is actively controlling power delivery based on VIN_UVCL_SETTING
#define iin_limit_active (1 << 2)  // indicates the input current limit control loop is actively controlling power delivery based on IIN_LIMIT_DAC
#define constant_current (1 << 1)  // indicates the charge current control loop is actively controlling power delivery based on ICHARGE_DAC
#define constant_voltage (1 << 0)  // indicates the battery voltage control loop is actively controlling power delivery based on VCHARGE_DAC

/* Limit alerts */
#define meas_sys_valid (1 << 15) // indicates that measurement system results have become valid.
#define qcount_lo (1 << 13) // indicates QCOUNT has fallen below QCOUNT_LO_ALERT_LIMIT
#define qcount_hi (1 << 12) // indicates QCOUNT has exceeded QCOUNT_HI_ALERT_LIMIT
#define vbat_lo (1 << 11) // indicates VBAT has fallen below VBAT_LO_ALERT_LIMIT
#define vbat_hi (1 << 10) // indicates VBAT has exceeded VBAT_HI_ALERT_LIMIT
#define vin_lo (1 << 9)  // indicates VIN has fallen below VIN_LO_ALERT_LIMIT
#define vin_hi (1 << 8)  // indicates VIN has exceeded VIN_HI_ALERT_LIMIT
#define vsys_lo (1 << 7)  // indicates VSYS has fallen below VSYS_LO_ALERT_LIMIT
#define vsys_hi (1 << 6)  // indicates VSYS has exceeded VIN_HI_ALERT_LIMIT
#define iin_hi (1 << 5)  // indicates IIN has exceeded IIN_HI_ALERT_LIMIT
#define ibat_lo (1 << 4)  // indicates IBAT has fallen below IBAT_LO_ALERT_LIMIT
#define die_temp_hi (1 << 3)  // indicates DIE_TEMP has exceeded DIE_TEMP_HI_ALERT_LIMIT
#define bsr_hi (1 << 2)  // indicates BSR has exceeded BSR_HI_ALERT_LIMIT
#define ntc_ratio_hi (1 << 1)  // indicates NTC_RATIO has exceeded NTC_RATIO_HI_ALERT_LIMIT
#define ntc_ratio_lo (1 << 0)  // indicates NTC_RATIO has exceeded NTC_RATIO_LO_ALERT_LIMIT
 
/* System status bits */
#define charger_enabled (1 << 13) // indicates that the battery charger is active
#define mppt_en_pin (1 << 11) // indicates the mppt_en pin is set to enable maximum power point tracking
#define equalize_req (1 << 10) // indicates a rising edge has been detected at the EQ pin, and an lead-acid equalize charge is queued
#define drvcc_good (1 << 9)  // indicates DRVCC voltage is above switching charger undervoltage lockout level (4.3V typical)
#define cell_count_error (1 << 8)  // indicates an invalid combination of CELLS pin settings
#define ok_to_charge (1 << 6)  // indicates all system conditions are met to allow battery charger operation
#define no_rt (1 << 5)  // indicates no resistor has been detected at the rt pin
#define thermal_shutdown (1 << 4)  // indicates die temperature is greater than thermal shutdown level (160Â°C typical)
#define vin_ovlo (1 << 3)  // indicates vin voltage is greater than overvoltage lockout level (38.6V typical)
#define vin_gt_vbat (1 << 2)  // indicates vin voltage is sufficiently greater than batsens for switching charger operation (200mV typical)
#define intvcc_gt_4p3v (1 << 1)  // indicates INTVCC voltage is above switching charger undervoltage lockout level (4.3V typ)
#define intvcc_gt_2p8v (1 << 0)  // indicates INTVCC voltage is greater than measurement system lockout level (2.8V typical)

#define chem 8 // programmed battery chemistry
#define cell_count_pins 0 // cell count as set by CELLS pins

#endif
