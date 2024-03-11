#ifndef LSM6DSO_REGISTERS_H_INCLUDED
#define LSM6DSO_REGISTERS_H_INCLUDED

#define LSM6_WRITE  0x00U   ///< Address byte value for a write operation
#define LSM6_READ   0x80U   ///< Address byte value for a read operation

//WhoAmI register (0x0F) values
#define LSM6_WHOAMI 0x6CU   ///< Who Am I constant value

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
    STATUS_MASTER_MAINPAGE  = 0x39U,    ///< 0x37 - RO : Sensor hub source register
    NB_REGISTERS
}LSM6DSOregister_e;

#endif
