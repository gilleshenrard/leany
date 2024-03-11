#ifndef LSM6DSO_REGISTERS_H_INCLUDED
#define LSM6DSO_REGISTERS_H_INCLUDED

#define LSM6_WRITE  0x00U   ///< Address byte value for a write operation
#define LSM6_READ   0x80U   ///< Address byte value for a read operation

//WhoAmI register (0x0F) values
#define LSM6_WHOAMI 0x6CU   ///< Who Am I constant value

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



#endif
