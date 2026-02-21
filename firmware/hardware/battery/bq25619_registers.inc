/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_BATTERY_MANAGEMENT_BQ25619_REGISTERS_H
#define HARDWARE_BATTERY_MANAGEMENT_BQ25619_REGISTERS_H
#include <stdint.h>

enum {
    kStatus_struct_align = 32U,
};

enum {
    // General values
    kDEFAULT_SLAVEADDR = 0x6AU,  ///< Default I²C slave address
    kNB_STATUS_BYTES = 3U,       ///< Number of status bytes

    // Input current limit (0x00) values
    kENABLE_HIZ = 0x80U,          ///< Bit value to enable HIZ (High impedance) mode. Disconnects the input source
    kDISABLE_HIZ = 0x00U,         ///< Bit value to disable HIZ (High impedance) mode. Reconnects the input source
    kTEMPERATURE_IGNORE = 0x40U,  ///< Bit value to always consider the thermistor value as good
    kINPUT_LIMIT_1_2A = 0x0BU,    ///< Bits value to limit the input current to 1.2A
    kENABLE_BATSNS = 0x00,        ///< Bit value to enable BATSNS in CV regulation

    // Charger control 0 (0x01) values
    kDISABLE_WATCHDOG = 0x00U,  ///< Bit value to disable the internal watchdog
    kRESET_WATCHDOG = 0x40U,    ///< Bit value to reset the internal watchdog
    kDISABLE_PMID = 0x00U,      ///< Bit value to disable the boost mode (output to PMID)
    kENABLE_CHARGE = 0x10U,     ///< Bit value to enable the buck mode (battery charge)
    kVSYS_MIN_3_5V = 0x00U,     ///< Bits value to set the minimum system voltage to 3.5V

    // Charge current limit register (0x02) values
    kREGULATE_LOW_CURRENT = 0x00U,  ///< Bit value to let Q1 regulated when IINDPM < 700mA
    kFASTCHARGE_1180MA = 0x3BU,     ///< Bits value to set the fast charge current to 1180mA

    // Precharge and Termination Current Limit Register (0x03) values
    kPRECHARGE_260MA = 0xC0U,    ///< Bits value to set the precharge current to 260mA
    kTERMINATION_260MA = 0x0CU,  ///< Bits value to set the termination current to 260mA

    // Battery Voltage Limit Register (0x04) values
    kBATTERY_LIMIT_4_2V = 0x40U,          ///< Bits value to set the battery voltage limit to 4.2V
    kNO_TOPOFF_TIMER = 0x00U,             ///< Bits value to disable the topoff timer
    kBATT_RECHG_THRESHOLD_120MV = 0x00U,  ///< Bits value to set the battery recharge threshold to 120mV

    // Charger Control 1 Register (0x05) values
    kENABLE_CHG_TERMINATION = 0x80U,   ///< Bit value to enable charge termination
    kDISABLE_WATCHDOG_TIMER = 0x00U,   ///< Bit value to disable the watchdog timer
    kENABLE_CHG_SAFETY_TIMER = 0x08U,  ///< Bit value to enable precharge and fast charge safety timer
    kSAFETY_10H = 0x04U,               ///< Bit value to set the safety timer to 10h

    // Charger Control 2 Register (0x06) values
    kOVERVOLTAGE_THRESHOLD_6_4V = 0x40U,  ///< Bit value to set the input overvoltage threshold to 6.4V
    kINPUT_VOLTAGE_DPM_4_5V = 0x06U,      ///< Bits value to set the input Dynamic Power Management voltage to 4.5V

    // Part Information (0x0B) values
    kPN_MASK = 0x78U,    ///< Mask used to isolate the charger Part Number
    kPN_VALUE = 0x28U,   ///< Part Number value
    kREG_RESET = 0x80U,  ///< Bit value to reset the registers to their default value
};

/**
 * Enumeration of the general charge faults
 */
typedef enum {
    kChgFaultNormal = 0x00U,       ///< Normal state
    kChgFaultInput = 0x01U,        ///< Input fault
    kChgFaultThermal = 0x02U,      ///< Thermal shutdown
    kChgFaultSafetyTimer = 0x03U,  ///< Charge safety timer expiration
} ChargeFault;

/**
 * Enumeration of the NTC states
 */
typedef enum {
    kNTCnormal = 0x00U,  ///< Thermistor is OK
    kNTCwarm = 0x02U,    ///< Thermistor is warm
    kNTCcool = 0x03U,    ///< Thermistor is cool
    kNTCcold = 0x05U,    ///< Thermistor is cold
    kNTChot = 0x06U,     ///< Thermistor is hot
} NTCfault;

/**
 * Enumeration of the VBUS statuses
 */
typedef enum {
    kSTATnoInput = 0x00U,      ///< No input
    kSTATnoUSBhost = 0x01U,    ///<  USB host SDP (500 mA) -> PSEL pin HIGH
    kSTATadapter2_4A = 0x03U,  ///< Adapter 2.4 A -> PSEL pin LOW
    kSTATboostMode = 0x07U,    ///< Boost mode
} VBUSstat;

/**
 * Enumeration of the charge statuses
 */
typedef enum {
    kChargeNotCharging = 0x00U,  ///< Not charging
    kChargeTrickle = 0x01U,      ///< Precharge or trickle charge (< VBATLOWV)
    kChargeFast = 0x02U,         ///< Fast charging
    kChargeTermination = 0x03U,  ///< Charge termination
} ChargeStatus;

/**
 * Enumeration of the device modes due to watchdog status
 */
typedef enum {
    kCHGmodeHost = 0x00U,     ///< Normal, device is in host mode
    kCHGmodeDefault = 0x01U,  ///< Watchdog timer expiration, device is in default mode
} DeviceMode;

/**
 * All the bits in the Charger Status retisters
 */
typedef struct {
    //Charger Status 0 Register (Address = 08h)
    uint8_t vbatt_below_minimum : 1;  ///< VSYS_STAT bit : Whether in SYS_MIN regulation (VBAT < VSYS_MIN)
    uint8_t thermal_regulation : 1;   ///< THERM_STAT bit : Whether in thermal regulation
    uint8_t power_good : 1;           ///< PG_STAT bit : Whether Power Good status is reached
    uint8_t chrg_status : 2;          ///< CHRG_STAT bits @see ChargeStatus
    uint8_t vbus_status : 3;          ///< VBUS_STAT bits @see VBUSstat

    //Charger Status 1 Register (Address = 09h)
    uint8_t thermal_fault : 3;    ///< NTC_FAULT bits @see NTCfault
    uint8_t bat_overvoltage : 1;  ///< BAT_FAULT bit : Whether battery is in overvoltage
    uint8_t charge_fault : 2;     ///< CHRG_FAULT bits @see ChargeFault
    uint8_t boostmode_fault : 1;  ///< BOOST_FAULT bit : Whether a fault is detected in boost mode
    uint8_t device_mode : 1;      ///< WATCHDOG_FAULT bit : Whether the watchdog expired @see DeviceMode

    //Charger Status 2 Register (Address = 0Ah)
    uint8_t cur_dpm_int_disabled : 1;   ///< IINDPM_INT_ MASK MASK bit : Whether ~INT is asserted to host during IINDPM
    uint8_t volt_dpm_int_disabled : 1;  ///< VINDPM_INT_ MASK bit : Whether ~INT is asserted to host during VINDPM
    uint8_t input_overvoltage : 1;      ///< ACOV_STAT bit : Whether In ACOV
    uint8_t topoff_timer_active : 1;    ///< TOPOFF_ACTIVE bit : Whether Top off timer is counting
    uint8_t batsns_bad_connection : 1;  ///< BATSNS_STAT bit : Whether BATSNS pin is open/short or in good connection.
    uint8_t input_current_high : 1;  ///< IINDPM_STAT : Whether in IINDPM (input current above the input current limit)
    uint8_t input_voltage_low : 1;   ///< VINDPM_STAT : Whether in VINDPM (input voltage below the input voltage limit)
    uint8_t poor_source_passed : 1;  ///< VBUS_GD bit : Whether VBUS passes poor source detection
} __attribute__((aligned(kStatus_struct_align))) ChargerStatusBits;

typedef union {
    ChargerStatusBits bits;
    uint8_t bytes[kNB_STATUS_BYTES];
} ChargerStatus;

/**
 * Enumeration of the BQ25619 registers
 */
typedef enum {
    kINPUT_CUR_LIMIT = 0x00U,      ///< rw - REG00 : Input Current Limit
    kCHG_CONTROL0 = 0x01U,         ///< rw - REG01 : Charger Control 0
    kCHG_CUR_LIMIT = 0x02U,        ///< rw - REG02 : Charge Current Limit
    kPCHG_TERM_CUR_LIMIT = 0x03U,  ///< rw - REG03 : Precharge and Termination Current Limit
    kBATT_VOLT_LIMIT = 0x04U,      ///< rw - REG04 : Battery Voltage Limit
    kCHG_CONTROL1 = 0x05U,         ///< rw - REG05 : Charger Control 1
    kCHG_CONTROL2 = 0x06U,         ///< rw - REG06 : Charger Control 2
    kCHG_CONTROL3 = 0x07U,         ///< rw - REG07 : Charger Control 3
    kCHG_STATUS0 = 0x08U,          ///< ro - REG08 : Charger Status 0
    kCHG_STATUS1 = 0x09U,          ///< ro - REG09 : Charger Status 1
    kCHG_STATUS2 = 0x0AU,          ///< ro - REG0A : Charger Status 2
    kPART_INFO = 0x0BU,            ///< ro - REG0B : Part Information
    kCHG_CONTROL4 = 0x0CU,         ///< rw - REG0C : Charger Control 4
} BQ25619register;
#endif
