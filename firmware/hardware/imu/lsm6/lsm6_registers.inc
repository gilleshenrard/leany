/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */
#ifndef HARDWARE_IMU_LSM6_REGISTERS_H_INCLUDED
#define HARDWARE_IMU_LSM6_REGISTERS_H_INCLUDED

enum {
    kLSM6_WRITE = 0x00U,  ///< Address byte value for a write operation
    kLSM6_READ = 0x80U,   ///< Address byte value for a read operation

    // FIFO control register 4 (0x0A) values
    kFIFO_MODE_BYPASS = 0x00U,  ///< Bit value to disable the FIFO

    // INT1 pin control register (0x0D) values
    kINT1_AXL_DATA_RDY = 0x01U,  ///< Bit value to enable the accelerometer data-ready interrupt on INT1

    // WhoAmI register (0x0F) values
    kLSM6_WHOAMI = 0x6CU,  ///< Who Am I constant value

    // Accelerometer Control register 1 (0x10) values
    kAXL_POWER_DOWN = 0x00U,        ///< Accelerometer ODR value for power-down
    kAXL_ODR_416HZ = 0x60U,         ///< Accelerometer ODR value for 416Hz High-Performance
    kODR_XL_52HZ = 0x30U,           ///< Bit value for an accelerometer 52Hz ODR
    kLSM6_AXL_LPF2_ENABLE = 0x02U,  ///< Bit value to enable the accelerometers LP filter 2
    kLPF2_XL_DISABLE = 0x00U,       ///< Bit value for accelerometer LPF2 disable
    kFS_XL_4G = 0x08U,              ///< Bit value for +-4G accelerometer range
    kFS_XL_2G = 0x00U,              ///< Bit value for +-2G accelerometer range

    // Gyroscope Control register 2 (0x11) values
    kGYR_POWER_DOWN = 0x00U,   ///< Gyroscope ODR value for power-down
    kGYR_FS_125_DPS = 0x02U,   ///< Bit value used to force gyroscope sensitivity to 125 °/s
    kGYR_FS_2000_DPS = 0x0CU,  ///< Bit value used to force gyroscope sensitivity to 2000 °/s
    kGYR_ODR_208HZ = 0x50U,    ///< Gyroscope ODR value for 208Hz High-Performance

    // Control register 3 (0x12) values
    kLSM6_REBOOT_MEMORY = 0x80U,    ///< Bit value to reboot the LSM6DSR memory
    kLSM6_REBOOT_NORMAL = 0x00U,    ///< Bit value to boot the LSM6DSR in normal mode
    kLSM6_BDU_WAITBLOCK = 0x40U,    ///< output registers are not updated until MSB and LSB have been read
    kLSM6_INT_ACTIVE_LOW = 0x20U,   ///< Bit value to set interrupts as active low (falling-edge)
    kLSM6_INT_ACTIVE_HIGH = 0x00U,  ///< Bit value to set interrupts as active high (rising-edge)
    kLSM6_INT_PUSHPULL = 0x00U,     ///< Bit value to set interrupts as active high (rising-edge)
    kLSM6_SOFTWARE_RESET = 0x01U,   ///< Bit value to reset the LSM6DSR software
    kLSM6_SPI_4WIRE = 0x00U,        ///< Bit value to set SPI to 4-wire mode
    kLSM6_F_INC_AUTO = 0x04U,       ///< Bit value to automatically increment registers during burst-read

    // Control register 4 (0x13) values
    kGYR_LPF1_ENABLE = 0x02U,    ///< Bit value to enable gyroscope LP1 filter
    kGYR_LPF1_DISABLE = 0x00U,   ///< Bit value to disable gyroscope LP1 filter
    kGYR_SLEEP_DISABLE = 0x00U,  ///< Bit value to disable gyroscope sleep mode
    kLSM6_DRDY_DISABLE = 0x00U,  ///< Bit value to disable Data Available signal
    kLSM6_I2C_DISABLE = 0x04U,   ///< Bit value to disable I²C on the LSM6

    // Control register 5 (0x14) values
    kGYR_SELFTEST_DISABLE = 0x00U,     ///< Bit value to disable the Gyroscope self-test
    kGYR_SELFTEST_ENABLE_POS = 0x04U,  ///< Bit value to enable the Gyroscope self-test in positive polarity
    kAXL_SELFTEST_DISABLE = 0x00U,     ///< Bit value to disable the Accelerometer self-test
    kAXL_SELFTEST_ENABLE_POS = 0x01U,  ///< Bit value to enable the Accelerometer self-test in positive polarity

    // Control register 6 (0x15) values (valid with gyroscope 416Hz ODR)
    kGYR_LPF1_CUTOFF_136_6HZ = 0x00U,  ///< Bit value to set gyroscope LPF1 cutoff freq. to 136.6Hz
    kGYR_LPF1_CUTOFF_130_5HZ = 0x01U,  ///< Bit value to set gyroscope LPF1 cutoff freq. to 130.5Hz
    kGYR_LPF1_CUTOFF_120_3HZ = 0x02U,  ///< Bit value to set gyroscope LPF1 cutoff freq. to 120.3Hz
    kGYR_LPF1_CUTOFF_137_1HZ = 0x03U,  ///< Bit value to set gyroscope LPF1 cutoff freq. to 137.1Hz
    kGYR_LPF1_CUTOFF_86_7HZ = 0x04U,   ///< Bit value to set gyroscope LPF1 cutoff freq. to 86.7Hz
    kGYR_LPF1_CUTOFF_48HZ = 0x05U,     ///< Bit value to set gyroscope LPF1 cutoff freq. to 48Hz
    kGYR_LPF1_CUTOFF_24_6HZ = 0x06U,   ///< Bit value to set gyroscope LPF1 cutoff freq. to 24.6Hz
    kGYR_LPF1_CUTOFF_12_4HZ = 0x07U,   ///< Bit value to set gyroscope LPF1 cutoff freq. to 12.4Hz
    kLSM6_DEN_DISABLE = 0x00U,         ///< Bit value to disable DEN signal
    kAXL_HM_MODE = 0x00,               ///< Bit value to enable Accelerometer high-perf. mode

    // Control register 7 (0x16) values
    kGYR_HPF_ENABLE = 0x40U,          ///< Bit value to enable gyroscope HPF
    kGYR_HPF_DISABLE = 0x00U,         ///< Bit value to disable gyroscope HPF
    kGYR_HM_DISABLE = 0x80U,          ///< Bit value to disable gyroscope High-perf. mode
    kGYR_HM_ENABLE = 0x00U,           ///< Bit value to enable gyroscope HPF
    kGYR_HPF_CUTOFF_16MHZ = 0x00U,    ///< Bit value to set gyroscope HPF cutoff freq. to 16mHz
    kGYR_HPF_CUTOFF_65MHZ = 0x10U,    ///< Bit value to set gyroscope HPF cutoff freq. to 65mHz
    kGYR_HPF_CUTOFF_260MHZ = 0x20U,   ///< Bit value to set gyroscope HPF cutoff freq. to 260mHz
    kGYR_HPF_CUTOFF_1040MHZ = 0x30U,  ///< Bit value to set gyroscope HPF cutoff freq. to 1040mHz
    kLSM6_OIS_DISABLE = 0x00U,        ///< Bit value to disable OIS on the device

    // Control register 8 (0x17) values
    kAXL_NO_HP_FILTER = 0x00U,  ///< Bit value to disable the accelerometers HP filter
    k6D_NO_LP_FILTER = 0x00U,   ///< Bit value to disable the 6D detection LP filter
    kAXL_LPF2_ODR_4 = 0x00U,    ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/4
    kAXL_LPF2_ODR_10 = 0x20U,   ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/10
    kAXL_LPF2_ODR_20 = 0x40U,   ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/20
    kAXL_LPF2_ODR_45 = 0x60U,   ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/45
    kAXL_LPF2_ODR_100 = 0x80U,  ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/100
    kAXL_LPF2_ODR_200 = 0xA0U,  ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/200
    kAXL_LPF2_ODR_400 = 0xC0U,  ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/400
    kAXL_LPF2_ODR_800 = 0xE0U,  ///< Bit value to set the accelerometer's LP filter 2 cutoff value to ODR/800

    // Control register 9 (0x18) values
    kAXL_DEN_DISABLE = 0x00U,   ///< Bit value to disable accelerometer's DEN
    kLSM6_I3C_ENABLE = 0x00U,   ///< Bit value to enable I3C interface
    kLSM6_I3C_DISABLE = 0x02U,  ///< Bit value to enable I3C interface

    // Control register 10 (0x19) values
    kLSM6_TIMESTAMP_ENABLE = 0x20U,   ///< Enables timestamp counter
    kLSM6_TIMESTAMP_DISABLE = 0x00U,  ///< Disables timestamp counter

    // Status register (0x1E) values
    kLSM6_AXL_DATA_AVAIL = 0x01U,  ///< Bit value indicating new accelerometer reading is available
    kLSM6_GYR_DATA_AVAIL = 0x02U,  ///< Bit value indicating new gyroscope reading is available
    kLSM6_TMP_DATA_AVAIL = 0x04U,  ///< Bit value indicating new temperature reading is available

    // Embedded Function register (0x01) values
    kLSM6_ENABLE_EMB_FUNCT = 0x80U,   ///< Enable the embedded function register access
    kLSM6_DISABLE_FUNCTIONS = 0x00U,  ///< Disable the embedded function and sensor hub registers access

    // Page Read/Write register (0x17) values
    kLSM6_ENABLE_PG_WRITE = 0x40U,  ///< Enable write operations to an embedded function page
    kLSM6_ENABLE_PG_READ = 0x20U,   ///< Enable read operations to an embedded function page
    kLSM6_DISABLE_PG_RDWR = 0x00U,  ///< Disable read/write operations on embedded functions

    // Page selection register (0x02) values
    kLSM6_PG_SELECT_DEFAULT = 0x01U  ///< Value of the 4 lower bits needed for correct operation
};

/**
 * @brief Enumeration of the LSM6 registers table
 */
typedef enum {
    kFUNC_CFG_ACCESS = 0x01U,           ///< RW - Enable embedded functions register
    kPIN_CTRL = 0x02U,                  ///< RW - SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register
    kFIFO_CTRL1 = 0x07U,                ///< RW - FIFO control register 1
    kFIFO_CTRL2 = 0x08U,                ///< RW - FIFO control register 2
    kFIFO_CTRL3 = 0x09U,                ///< RW - FIFO control register 3
    kFIFO_CTRL4 = 0x0AU,                ///< RW - FIFO control register 4
    kCOUNTER_BDR_REG1 = 0x0BU,          ///< RW - Counter batch data rate register 1
    kCOUNTER_BDR_REG2 = 0x0CU,          ///< RW - Counter batch data rate register 2
    kINT1_CTRL = 0x0DU,                 ///< RW - INT1 pin control register
    kINT2_CTRL = 0x0EU,                 ///< RW - INT2 pin control register
    kWHO_AM_I = 0x0FU,                  ///< RO - WHO_AM_I register
    kCTRL1_XL = 0x10U,                  ///< RW - Accelerometer control register 1
    kCTRL2_G = 0x11U,                   ///< RW - Gyroscope control register 2
    kCTRL3_C = 0x12U,                   ///< RW - Control register 3
    kCTRL4_C = 0x13U,                   ///< RW - Control register 4
    kCTRL5_C = 0x14U,                   ///< RW - Control register 5
    kCTRL6_C = 0x15U,                   ///< RW - Control register 6
    kCTRL7_G = 0x16U,                   ///< RW - Control register 7
    kCTRL8_XL = 0x17U,                  ///< RW - Control register 8
    kCTRL9_XL = 0x18U,                  ///< RW - Control register 9
    kCTRL10_C = 0x19U,                  ///< RW - Control register 10
    kALL_INT_SRC = 0x1AU,               ///< RO - Source register for all interrupts
    kWAKE_UP_SRC = 0x1BU,               ///< RO - Wake-up interrupt source register
    kTAP_SRC = 0x1CU,                   ///< RO - Tap source register
    kD6D_SRC = 0x1DU,                   ///< RO - Portrait, landscape, face-up and face-down source register
    kSTATUS_REG = 0x1EU,                ///< RO - Data ready status register
    kOUT_TEMP_L = 0x20U,                ///< RO - Temperature data output register LSB
    kOUT_TEMP_H = 0x21U,                ///< RO - Temperature data output register MSB
    kOUTX_L_G = 0x22U,                  ///< RO - Angular rate sensor pitch axis (X) angular rate output register LSB
    kOUTX_H_G = 0x23U,                  ///< RO - Angular rate sensor pitch axis (X) angular rate output register MSB
    kOUTY_L_G = 0x24U,                  ///< RO - Angular rate sensor pitch axis (Y) angular rate output register LSB
    kOUTY_H_G = 0x25U,                  ///< RO - Angular rate sensor pitch axis (Y) angular rate output register MSB
    kOUTZ_L_G = 0x26U,                  ///< RO - Angular rate sensor pitch axis (Z) angular rate output register LSB
    kOUTZ_H_G = 0x27U,                  ///< RO - Angular rate sensor pitch axis (Z) angular rate output register MSB
    kOUTX_L_A = 0x28U,                  ///< RO - Linear acceleration sensor X-axis output register LSB
    kOUTX_H_A = 0x29U,                  ///< RO - Linear acceleration sensor X-axis output register MSB
    kOUTY_L_A = 0x2AU,                  ///< RO - Linear acceleration sensor Y-axis output register LSB
    kOUTY_H_A = 0x2BU,                  ///< RO - Linear acceleration sensor Y-axis output register MSB
    kOUTZ_L_A = 0x2CU,                  ///< RO - Linear acceleration sensor Z-axis output register LSB
    kOUTZ_H_A = 0x2DU,                  ///< RO - Linear acceleration sensor Z-axis output register MSB
    kEMB_FUNC_STATUS_MAINPAGE = 0x35U,  ///< RO - Embedded function status register
    kFSM_STATUS_A_MAINPAGE = 0x36U,     ///< RO - Finite State Machines 1 to 8 status register
    kFSM_STATUS_B_MAINPAGE = 0x37U,     ///< RO - Finite State Machines 9 to 16 status register
    kSTATUS_MASTER_MAINPAGE = 0x39U,    ///< RO - Sensor hub source register
    kFIFO_STATUS1 = 0x3AU,              ///< RO - FIFO status register 1
    kFIFO_STATUS2 = 0x3BU,              ///< RO - FIFO status register 2
    kTIMESTAMP0 = 0x40U,                ///< RO - Timestamp first data output register, bits [31-24]
    kTIMESTAMP1 = 0x41U,                ///< RO - Timestamp first data output register, bits [23-16]
    kTIMESTAMP2 = 0x42U,                ///< RO - Timestamp first data output register, bits [15-8]
    kTIMESTAMP3 = 0x43U,                ///< RO - Timestamp first data output register, bits [7-0]
    kTAP_CFG0 = 0x56U,                  ///< RW - Activity/inact., filtering config., and tap recognition functions
    kTAP_CFG1 = 0x57U,                  ///< RW - Tap configuration register
    kTAP_CFG2 = 0x58U,                  ///< RW - Enables interrupt and inact. functions, and tap recognition functions
    kTAP_THS_6D = 0x59U,                ///< RW - Portrait/landscape position and tap function threshold register
    kINT_DUR2 = 0x5AU,                  ///< RW - Tap recognition function setting register
    kWAKE_UP_THS = 0x5BU,               ///< RW - Single/double-tap selection and wake-up configuration
    kWAKE_UP_DUR = 0x5CU,               ///< RW - Free-fall, wakeup and sleep mode functions duration setting register
    kFREE_FALL = 0x5DU,                 ///< RW - Free-fall function duration setting register
    kMD1_CFG = 0x5EU,                   ///< RW - Functions routing on INT1 register
    kMD2_CFG = 0x5FU,                   ///< RW - Functions routing on INT2 register
    kI3C_BUS_AVB = 0x62U,               ///< RW - I3C_BUS_AVB register
    kINTERNAL_FREQ_FINE = 0x63U,        ///< RO - Internal frequency register
    kINT_OIS = 0x6FU,                   ///< RO - OIS interrupt config. register and accel. self-test enable setting
    kCTRL1_OIS = 0x70U,                 ///< RO - OIS configuration register
    kCTRL2_OIS = 0x71U,                 ///< RO - OIS configuration register
    kCTRL3_OIS = 0x72U,                 ///< RO - OIS configuration register
    kX_OFS_USR = 0x73U,                 ///< RW - Accelerometer X-axis user offset correction
    kY_OFS_USR = 0x74U,                 ///< RW - Accelerometer Y-axis user offset correction
    kZ_OFS_USR = 0x75U,                 ///< RW - Accelerometer Z-axis user offset correction
    kFIFO_DATA_OUT_TAG = 0x78U,         ///< RO - FIFO tag register
    kFIFO_DATA_OUT_X_L = 0x79U,         ///< RO - FIFO data output X LSB
    kFIFO_DATA_OUT_X_H = 0x7AU,         ///< RO - FIFO data output X MSB
    kFIFO_DATA_OUT_Y_L = 0x7BU,         ///< RO - FIFO data output Y LSB
    kFIFO_DATA_OUT_Y_H = 0x7CU,         ///< RO - FIFO data output Y MSB
    kFIFO_DATA_OUT_Z_L = 0x7DU,         ///< RO - FIFO data output Z LSB
    kFIFO_DATA_OUT_Z_H = 0x7EU,         ///< RO - FIFO data output Z MSB
    kMAX_REGISTER = 0x7FU               ///< Maximum Number of LSM6 registers
} LSM6register;

/**
 * @brief Enumeration of all the embedded functions
 */
typedef enum {
    kPAGE_SEL = 0x02U,                ///< RW - Enable advanced features dedicated page
    kEMB_FUNC_EN_A = 0x04U,           ///< RW - Embedded functions enable register
    kEMB_FUNC_EN_B = 0x05U,           ///< RW - Embedded functions enable register
    kPAGE_ADDRESS = 0x08U,            ///< RW - Page address register
    kPAGE_VALUE = 0x09U,              ///< RW - Page value register
    kEMB_FUNC_INT1 = 0x0AU,           ///< RW - INT1 pin control register
    kFSM_INT1_A = 0x0BU,              ///< RW - INT1 pin control register, FSM 1 to 8
    kFSM_INT1_B = 0x0CU,              ///< RW - INT1 pin control register, FSM 9 to 16
    kEMB_FUNC_INT2 = 0x0EU,           ///< RW - INT2 pin control register
    kFSM_INT2_A = 0x0FU,              ///< RW - INT2 pin control register, FSM 1 to 8
    kFSM_INT2_B = 0x10U,              ///< RW - INT2 pin control register, FSM 9 to 16
    kEMB_FUNC_STATUS = 0x12U,         ///< RO - Embedded function status register
    kFSM_STATUS_A = 0x13U,            ///< RO - Finite State Machine status register, FSM 1 to 8
    kFSM_STATUS_B = 0x14U,            ///< RO - Finite State Machine status register, FSM 9 to 16
    kPAGE_RW = 0x17U,                 ///< RW - Enable read and write mode of advanced features dedicated page
    kEMB_FUNC_FIFO_CFG = 0x44U,       ///< RW - Embedded functions batching configuration register
    kFSM_ENABLE_A = 0x46U,            ///< RW - FSM enable register, FSM 1 to 8
    kFSM_ENABLE_B = 0x47U,            ///< RW - FSM enable register, FSM 9 to 16
    kFSM_LONG_COUNTER_L = 0x48U,      ///< RW - FSM long counter status register LSB
    kFSM_LONG_COUNTER_H = 0x49U,      ///< RW - FSM long counter status register MSB
    kFSM_LONG_COUNTER_CLEAR = 0x4AU,  ///< RW - FSM long counter reset register
    kFSM_OUTS1 = 0x4CU,               ///< RO - FSM1 output register
    kFSM_OUTS2 = 0x4DU,               ///< RO - FSM2 output register
    kFSM_OUTS3 = 0x4EU,               ///< RO - FSM3 output register
    kFSM_OUTS4 = 0x4FU,               ///< RO - FSM4 output register
    kFSM_OUTS5 = 0x50U,               ///< RO - FSM5 output register
    kFSM_OUTS6 = 0x51U,               ///< RO - FSM6 output register
    kFSM_OUTS7 = 0x52U,               ///< RO - FSM7 output register
    kFSM_OUTS8 = 0x53U,               ///< RO - FSM8 output register
    kFSM_OUTS9 = 0x54U,               ///< RO - FSM9 output register
    kFSM_OUTS10 = 0x55U,              ///< RO - FSM10 output register
    kFSM_OUTS11 = 0x56U,              ///< RO - FSM11 output register
    kFSM_OUTS12 = 0x57U,              ///< RO - FSM12 output register
    kFSM_OUTS13 = 0x58U,              ///< RO - FSM13 output register
    kFSM_OUTS14 = 0x59U,              ///< RO - FSM14 output register
    kFSM_OUTS15 = 0x5AU,              ///< RO - FSM15 output register
    kFSM_OUTS16 = 0x5BU,              ///< RO - FSM16 output register
    kEMB_FUNC_ODR_CFG_B = 0x5FU,      ///< RW - Finite State Machine output data rate configuration register
    kSTEP_COUNTER_L = 0x62U,          ///< RO - Step counter output register LSB
    kSTEP_COUNTER_H = 0x63U,          ///< RO - Step counter output register MSB
    kEMB_FUNC_SRC = 0x64U,            ///< RW - Embedded function source register
    kEMB_FUNC_INIT_A = 0x66U,         ///< RW - Embedded functions initialization register
    kEMB_FUNC_INIT_B = 0x67U,         ///< RW - Embedded functions initialization register
    kMAX_FUNCTION = 0x68U             ///< Maximum number of LSM6DSR Embedded function registers
} LSM6embeddedFunction;

#endif
