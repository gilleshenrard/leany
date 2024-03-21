#ifndef LSM6DSO_REGISTERS_H_INCLUDED
#define LSM6DSO_REGISTERS_H_INCLUDED

#define LSM6_WRITE              0x00U   ///< Address byte value for a write operation
#define LSM6_READ               0x80U   ///< Address byte value for a read operation
#define LSM6_NB_OUT_REGISTERS   6U      ///< Number of output data registers for the accelerometer and the gyroscope

//WhoAmI register (0x0F) values
#define LSM6_WHOAMI             0x6CU   ///< Who Am I constant value

//Accelerometer Control register (0x10) values
#define LSM6_POWER_DOWN         0x00U   ///< Accelerometer/gyroscope ODR value for power-down
#define LSM6_ODR_416HZ          0x60U   ///< Accelerometer/gyroscope ODR value for 416Hz High-Performance

//Status register (0x1E) values
#define LSM6_AXL_DATA_AVAIL     0x01U   ///< Bit value indicating new accelerometer reading is available
#define LSM6_GYR_DATA_AVAIL     0x02U   ///< Bit value indicating new gyroscope reading is available
#define LSM6_TMP_DATA_AVAIL     0x04U   ///< Bit value indicating new temperature reading is available

/**
 * @brief Enumeration of the LSM6DSO registers table
 */
typedef enum{
    FUNC_CFG_ACCESS         = 0x01U,    ///< 0x01 - RW : Enable embedded functions register
    PIN_CTRL,                           ///< 0x02 - RW : SDO, OCS_AUX, SDO_AUX pins pull-up enable/disable register
    FIFO_CTRL1              = 0x07U,    ///< 0x07 - RW : FIFO control register 1
    FIFO_CTRL2,                         ///< 0x08 - RW : FIFO control register 2
    FIFO_CTRL3,                         ///< 0x09 - RW : FIFO control register 3
    FIFO_CTRL4,                         ///< 0x0A - RW : FIFO control register 4
    COUNTER_BDR_REG1,                   ///< 0x0B - RW : Counter batch data rate register 1
    COUNTER_BDR_REG2,                   ///< 0x0C - RW : Counter batch data rate register 2
    INT1_CTRL,                          ///< 0x0D - RW : INT1 pin control register
    INT2_CTRL,                          ///< 0x0E - RW : INT2 pin control register
    WHO_AM_I,                           ///< 0x0F - RO : WHO_AM_I register
    CTRL1_XL,                           ///< 0x10 - RW : Accelerometer control register 1
    CTRL2_G,                            ///< 0x11 - RW : Gyroscope control register 2
    CTRL3_C,                            ///< 0x12 - RW : Control register 3
    CTRL4_C,                            ///< 0x13 - RW : Control register 4
    CTRL5_C,                            ///< 0x14 - RW : Control register 5
    CTRL6_C,                            ///< 0x15 - RW : Control register 6
    CTRL7_G,                            ///< 0x16 - RW : Control register 7
    CTRL8_XL,                           ///< 0x17 - RW : Control register 8
    CTRL9_XL,                           ///< 0x18 - RW : Control register 9
    CTRL10_C,                           ///< 0x19 - RW : Control register 10
    ALL_INT_SRC,                        ///< 0x1A - RO : Source register for all interrupts
    WAKE_UP_SRC,                        ///< 0x1B - RO : Wake-up interrupt source register
    TAP_SRC,                            ///< 0x1C - RO : Tap source register
    D6D_SRC,                            ///< 0x1D - RO : Portrait, landscape, face-up and face-down source register
    STATUS_REG,                         ///< 0x1E - RO : Data ready status register
    OUT_TEMP_L              = 0x20U,    ///< 0x20 - RO : Temperature data output register LSB
    OUT_TEMP_H,                         ///< 0x21 - RO : Temperature data output register MSB
    OUTX_L_G,                           ///< 0x22 - RO : Angular rate sensor pitch axis (X) angular rate output register LSB
    OUTX_H_G,                           ///< 0x23 - RO : Angular rate sensor pitch axis (X) angular rate output register MSB
    OUTY_L_G,                           ///< 0x24 - RO : Angular rate sensor pitch axis (Y) angular rate output register LSB
    OUTY_H_G,                           ///< 0x25 - RO : Angular rate sensor pitch axis (Y) angular rate output register MSB
    OUTZ_L_G,                           ///< 0x26 - RO : Angular rate sensor pitch axis (Z) angular rate output register LSB
    OUTZ_H_G,                           ///< 0x27 - RO : Angular rate sensor pitch axis (Z) angular rate output register MSB
    OUTX_L_A,                           ///< 0x28 - RO : Linear acceleration sensor X-axis output register LSB
    OUTX_H_A,                           ///< 0x29 - RO : Linear acceleration sensor X-axis output register MSB
    OUTY_L_A,                           ///< 0x2A - RO : Linear acceleration sensor Y-axis output register LSB
    OUTY_H_A,                           ///< 0x2B - RO : Linear acceleration sensor Y-axis output register MSB
    OUTZ_L_A,                           ///< 0x2C - RO : Linear acceleration sensor Z-axis output register LSB
    OUTZ_H_A,                           ///< 0x2D - RO : Linear acceleration sensor Z-axis output register MSB
    EMB_FUNC_STATUS_MAINPAGE= 0x35U,    ///< 0x35 - RO : Embedded function status register
    FSM_STATUS_A_MAINPAGE,              ///< 0x36 - RO : Finite State Machines 1 to 8 status register
    FSM_STATUS_B_MAINPAGE,              ///< 0x37 - RO : Finite State Machines 9 to 16 status register
    STATUS_MASTER_MAINPAGE  = 0x39U,    ///< 0x39 - RO : Sensor hub source register
    FIFO_STATUS1,                       ///< 0x3A - RO : FIFO status register 1
    FIFO_STATUS2,                       ///< 0x3B - RO : FIFO status register 2
    TIMESTAMP0              = 0x40U,    ///< 0x40 - RO : Timestamp first data output register, bits [31-24]
    TIMESTAMP1,                         ///< 0x41 - RO : Timestamp first data output register, bits [23-16]
    TIMESTAMP2,                         ///< 0x42 - RO : Timestamp first data output register, bits [15-8]
    TIMESTAMP3,                         ///< 0x43 - RO : Timestamp first data output register, bits [7-0]
    TAP_CFG0                = 0x56U,    ///< 0x56 - RW : Activity/inactivity functions, configuration of filtering, and tap recognition functions
    TAP_CFG1,                           ///< 0x57 - RW : Tap configuration register
    TAP_CFG2,                           ///< 0x58 - RW : Enables interrupt and inactivity functions, and tap recognition functions
    TAP_THS_6D,                         ///< 0x59 - RW : Portrait/landscape position and tap function threshold register
    INT_DUR2,                           ///< 0x5A - RW : Tap recognition function setting register
    WAKE_UP_THS,                        ///< 0x5B - RW : Single/double-tap selection and wake-up configuration
    WAKE_UP_DUR,                        ///< 0x5C - RW : Free-fall, wakeup and sleep mode functions duration setting register
    FREE_FALL,                          ///< 0x5D - RW : Free-fall function duration setting register
    MD1_CFG,                            ///< 0x5E - RW : Functions routing on INT1 register
    MD2_CFG,                            ///< 0x5F - RW : Functions routing on INT2 register
    I3C_BUS_AVB             = 0x62U,    ///< 0x62 - RW : I3C_BUS_AVB register
    INTERNAL_FREQ_FINE,                 ///< 0x63 - RO : Internal frequency register
    INT_OIS                 = 0x6FU,    ///< 0x6F - RO : OIS interrupt configuration register and accelerometer self-test enable setting
    CTRL1_OIS,                          ///< 0x70 - RO : OIS configuration register
    CTRL2_OIS,                          ///< 0x71 - RO : OIS configuration register
    CTRL3_OIS,                          ///< 0x72 - RO : OIS configuration register
    X_OFS_USR,                          ///< 0x73 - RW : Accelerometer X-axis user offset correction
    Y_OFS_USR,                          ///< 0x74 - RW : Accelerometer Y-axis user offset correction
    Z_OFS_USR,                          ///< 0x75 - RW : Accelerometer Z-axis user offset correction
    FIFO_DATA_OUT_TAG       = 0x78U,    ///< 0x78 - RO : FIFO tag register
    FIFO_DATA_OUT_X_L,                  ///< 0x79 - RO : FIFO data output X LSB
    FIFO_DATA_OUT_X_H,                  ///< 0x7A - RO : FIFO data output X MSB
    FIFO_DATA_OUT_Y_L,                  ///< 0x7B - RO : FIFO data output Y LSB
    FIFO_DATA_OUT_Y_H,                  ///< 0x7C - RO : FIFO data output Y MSB
    FIFO_DATA_OUT_Z_L,                  ///< 0x7D - RO : FIFO data output Z LSB
    FIFO_DATA_OUT_Z_H,                  ///< 0x7E - RO : FIFO data output Z MSB
    MAX_REGISTER
}LSM6DSOregister_e;

/**
 * @brief Enumeration of all the embedded functions
 */
typedef enum{
    PAGE_SEL            = 0x02U,    ///< 0x02 - RW : Enable advanced features dedicated page
    EMB_FUNC_EN_A       = 0x04U,    ///< 0x04 - RW : Embedded functions enable register
    EMB_FUNC_EN_B,                  ///< 0x05 - RW : Embedded functions enable register
    PAGE_ADDRESS        = 0x08U,    ///< 0x08 - RW : Page address register
    PAGE_VALUE,                     ///< 0x09 - RW : Page value register
    EMB_FUNC_INT1,                  ///< 0x0A - RW : INT1 pin control register
    FSM_INT1_A,                     ///< 0x0B - RW : INT1 pin control register, FSM 1 to 8
    FSM_INT1_B,                     ///< 0x0C - RW : INT1 pin control register, FSM 9 to 16
    EMB_FUNC_INT2       = 0x0EU,    ///< 0x0E - RW : INT2 pin control register
    FSM_INT2_A,                     ///< 0x0F - RW : INT2 pin control register, FSM 1 to 8
    FSM_INT2_B,                     ///< 0x10 - RW : INT2 pin control register, FSM 9 to 16
    EMB_FUNC_STATUS     = 0x12U,    ///< 0x12 - RO : Embedded function status register
    FSM_STATUS_A,                   ///< 0x13 - RO : Finite State Machine status register, FSM 1 to 8
    FSM_STATUS_B,                   ///< 0x14 - RO : Finite State Machine status register, FSM 9 to 16
    PAGE_RW             = 0x17U,    ///< 0x17 - RW : Enable read and write mode of advanced features dedicated page
    EMB_FUNC_FIFO_CFG   = 0x44U,    ///< 0x44 - RW : Embedded functions batching configuration register
    FSM_ENABLE_A        = 0x46U,    ///< 0x46 - RW : FSM enable register, FSM 1 to 8
    FSM_ENABLE_B,                   ///< 0x47 - RW : FSM enable register, FSM 9 to 16
    FSM_LONG_COUNTER_L,             ///< 0x48 - RW : FSM long counter status register LSB
    FSM_LONG_COUNTER_H,             ///< 0x49 - RW : FSM long counter status register MSB
    FSM_LONG_COUNTER_CLEAR,         ///< 0x4A - RW : FSM long counter reset register
    FSM_OUTS1           = 0x4CU,    ///< 0x4C - RO : FSM1 output register
    FSM_OUTS2,                      ///< 0x4D - RO : FSM2 output register
    FSM_OUTS3,                      ///< 0x4E - RO : FSM3 output register
    FSM_OUTS4,                      ///< 0x4F - RO : FSM4 output register
    FSM_OUTS5,                      ///< 0x50 - RO : FSM5 output register
    FSM_OUTS6,                      ///< 0x51 - RO : FSM6 output register
    FSM_OUTS7,                      ///< 0x52 - RO : FSM7 output register
    FSM_OUTS8,                      ///< 0x53 - RO : FSM8 output register
    FSM_OUTS9,                      ///< 0x54 - RO : FSM9 output register
    FSM_OUTS10,                     ///< 0x55 - RO : FSM10 output register
    FSM_OUTS11,                     ///< 0x56 - RO : FSM11 output register
    FSM_OUTS12,                     ///< 0x57 - RO : FSM12 output register
    FSM_OUTS13,                     ///< 0x58 - RO : FSM13 output register
    FSM_OUTS14,                     ///< 0x59 - RO : FSM14 output register
    FSM_OUTS15,                     ///< 0x5A - RO : FSM15 output register
    FSM_OUTS16,                     ///< 0x5B - RO : FSM16 output register
    EMB_FUNC_ODR_CFG_B  = 0x5FU,    ///< 0x5F - RW : Finite State Machine output data rate configuration register
    STEP_COUNTER_L      = 0x62U,    ///< 0x62 - RO : Step counter output register LSB
    STEP_COUNTER_H,                 ///< 0x63 - RO : Step counter output register MSB
    EMB_FUNC_SRC,                   ///< 0x64 - RW : Embedded function source register
    EMB_FUNC_INIT_A     = 0x66U,    ///< 0x66 - RW : Embedded functions initialization register
    EMB_FUNC_INIT_B,                ///< 0x67 - RW : Embedded functions initialization register
    MAX_FUNCTION
}LSM6DSOembeddedFunction_e;

#endif
