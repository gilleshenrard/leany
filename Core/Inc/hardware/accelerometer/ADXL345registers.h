#ifndef INC_ADXL345REGISTERS_H_
#define INC_ADXL345REGISTERS_H_

// Device ID
#define ADXL_DEVICE_ID		0xE5		///< Device ID value

// Base serial communication
#define ADXL_WRITE			0x00		///< MSB value for a register write operation
#define ADXL_READ			0x80		///< MSB value for a register read operation
#define ADXL_SINGLE			0x00		///< Bit 6 value for a single register operation
#define ADXL_MULTIPLE		0x40		///< Bit 6 value for a multiple registers operation

// FIFO Control (register 0x38) configuration value
#define ADXL_MODE_BYPASS	0x00		///< Bits 7 and 6 values for the FIFO Bypass mode
#define ADXL_MODE_FIFO		0x40		///< Bits 7 and 6 values for the FIFO mode
#define ADXL_MODE_STREAM	0x80		///< Bits 7 and 6 values for the FIFO Stream mode
#define ADXL_MODE_TRIGGER	0xC0		///< Bits 7 and 6 values for the FIFO Trigger mode
#define ADXL_INT_MAP_INT1	0x00		///< Bit  5 value to map trigger events to the INT1 pin
#define ADXL_INT_MAP_INT2	0x20		///< Bit  5 value to map trigger events to the INT2 pin
#define ADXL_SAMPLES_32		0x20		///< Request 32 samples before triggering an interrupt (+1 to reuse at other places)
#define ADXL_SAMPLES_16		0x10		///< Request 16 samples before triggering an interrupt (+1 to reuse at other places)
#define ADXL_SAMPLES_8		0x08		///< Request 08 samples before triggering an interrupt (+1 to reuse at other places)

// Interrupt Enable (register 0x2E) configuration value
#define ADXL_INT_DATARDY	0x80		///< Bit to set to enable the Data Ready interrupt
#define ADXL_INT_SINGLETAP	0x40		///< Bit to set to enable the Single Tap interrupt
#define ADXL_INT_DOUBLETAP	0x20		///< Bit to set to enable the Double Tap interrupt
#define ADXL_INT_ACTIVITY	0x10		///< Bit to set to enable the Activity interrupt
#define ADXL_INT_INACTIVITY	0x08		///< Bit to set to enable the Inactivity interrupt
#define ADXL_INT_FREEFALL	0x04		///< Bit to set to enable the Free Fall interrupt
#define ADXL_INT_WATERMARK	0x02		///< Bit to set to enable the Watermark interrupt
#define ADXL_INT_OVERRUN	0x01		///< Bit to set to enable the Overrun interrupt
#define ADXL_INT_DISABLED	0x00		///< Bit to set to disable interrupts

#define ADXL_SELF_TEST		0x80
#define ADXL_NO_SELF_TEST	0x00
#define ADXL_SPI_3WIRE		0x40
#define ADXL_SPI_4WIRE		0x00
#define ADXL_INT_ACTIV_HIGH 0x00
#define ADXL_INT_ACTIV_LOW	0x20
#define ADXL_10BIT_RESOL	0x00
#define ADXL_13BIT_RESOL	0x08
#define ADXL_LEFT_JUSTIFY	0x04
#define ADXL_RIGHT_JUSTIFY	0x00
#define ADXL_RANGE_2G		0x00
#define ADXL_RANGE_4G		0x01
#define ADXL_RANGE_8G		0x02
#define ADXL_RANGE_16G		0x03

#define ADXL_POWER_NORMAL	0x00
#define ADXL_POWER_LOW		0x10
#define ADXL_RATE_200HZ		0x0B
#define ADXL_RATE_100HZ		0x0A
#define ADXL_RATE_50HZ		0x09

#define ADXL_STANDBY_MODE	0x00	///< Power control bit 3 configuration for standby mode
#define ADXL_MEASURE_MODE	0x08	///< Power control bit 3 configuration for measurement mode

#define ADXL_HIGH_RESERVED_REG	0x1C	///< Number of the last reserved register
#define ADXL_NB_DATA_REGISTERS	6U		///< Number of the last reserved register

/**
 * @brief Enumeration of the available registers of the ADXL345
 */
typedef enum{
	DEVICE_ID 		= 0x00,		///< 0x00 - RO : Device ID
	TAP_THRESHOLD	= 0x1D,		///< 0x1D - RW : Tap threshold   @note Registers 0x01 to 0x1C are reserved
	OFFSET_X,					///< 0x1E - RW : X-axis offset
	OFFSET_Y,					///< 0x1F - RW : Y-axis offset
	OFFSET_Z,					///< 0x20 - RW : Z-axis offset
	TAP_DURATION,				///< 0x21 - RW : Tap duration
	TAP_LATENCY,				///< 0x22 - RW : Tap latency
	TAP_WINDOW,					///< 0x23 - RW : Tap window
	ACTIVITY_THRESHOLD,			///< 0x24 - RW : Activity threshold
	INACTIVITY_THRESHOLD,		///< 0x25 - RW : Inactivity threshold
	INACTIVITY_TIME,			///< 0x26 - RW : Inactivity time
	ACTIVITY_CONTROL,			///< 0x27 - RW : Axis enable control for activity and inactivity detection
	FREEFALL_THRESHOLD,			///< 0x28 - RW : Free-fall threshold
	FREEFALL_TIME,				///< 0x29 - RW : Free-fall time
	TAP_AXES,					///< 0x2A - RW : Axis control for single tap/double tap
	TAP_ACTIVITY_SOURCE,		///< 0x2B - RO : Source of single tap/double tap
	BANDWIDTH_POWERMODE,		///< 0x2C - RW : Data rate and power mode control
	POWER_CONTROL,				///< 0x2D - RW : Power-saving features control
	INTERRUPT_ENABLE,			///< 0x2E - RW : Interrupt enable control
	INTERRUPT_MAPPING,			///< 0x2F - RW : Interrupt mapping control
	INTERRUPT_SOURCE,			///< 0x30 - RO : Source of interrupts
	DATA_FORMAT,				///< 0x31 - RW : Data format control
	DATA_X0,					///< 0x32 - RO : X-Axis Data 0
	DATA_X1,					///< 0x33 - RO : X-Axis Data 1
	DATA_Y0,					///< 0x34 - RO : Y-Axis Data 0
	DATA_Y1,					///< 0x35 - RO : Y-Axis Data 1
	DATA_Z0,					///< 0x36 - RO : Z-Axis Data 0
	DATA_Z1,					///< 0x37 - RO : Z-Axis Data 1
	FIFO_CONTROL,				///< 0x38 - RW : FIFO control
	FIFO_STATUS,				///< 0x39 - RO : FIFO status
	ADXL_REGISTER_MAXNB			///< Maximum register number
}adxl345Registers_e;


#endif /* INC_ADXL345REGISTERS_H_ */
