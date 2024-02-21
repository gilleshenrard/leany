#ifndef INC_ADXL345REGISTERS_H_
#define INC_ADXL345REGISTERS_H_

// Base serial communication
#define ADXL_WRITE			0x00		///< Register write operation
#define ADXL_READ			0x80		///< Register read operation
#define ADXL_SINGLE			0x00		///< Single register operation
#define ADXL_MULTIPLE		0x40		///< Multiple registers operation

// FIFO Control (register 0x38) configuration values
#define ADXL_MODE_BYPASS	0x00		///< FIFO Bypass mode
#define ADXL_MODE_FIFO		0x40		///< FIFO mode
#define ADXL_MODE_STREAM	0x80		///< FIFO Stream mode
#define ADXL_MODE_TRIGGER	0xC0		///< FIFO Trigger mode
#define ADXL_INT_MAP_INT1	0x00		///< Map trigger events to the INT1 pin
#define ADXL_INT_MAP_INT2	0x20		///< Map trigger events to the INT2 pin
#define ADXL_SAMPLES_32		0x20		///< 32 samples before triggering an interrupt (+1 to reuse at other places)
#define ADXL_SAMPLES_16		0x10		///< 16 samples before triggering an interrupt (+1 to reuse at other places)
#define ADXL_SAMPLES_8		0x08		///< 08 samples before triggering an interrupt (+1 to reuse at other places)

// Interrupt Enable (register 0x2E) configuration values
#define ADXL_INT_DATARDY	0x80		///< Enable the Data Ready interrupt
#define ADXL_INT_SINGLETAP	0x40		///< Enable the Single Tap interrupt
#define ADXL_INT_DOUBLETAP	0x20		///< Enable the Double Tap interrupt
#define ADXL_INT_ACTIVITY	0x10		///< Enable the Activity interrupt
#define ADXL_INT_INACTIVITY	0x08		///< Enable the Inactivity interrupt
#define ADXL_INT_FREEFALL	0x04		///< Enable the Free Fall interrupt
#define ADXL_INT_WATERMARK	0x02		///< Enable the Watermark interrupt
#define ADXL_INT_OVERRUN	0x01		///< Enable the Overrun interrupt
#define ADXL_INT_DISABLED	0x00		///< Disable interrupts

// Data Format (register 0x31) configuration values
#define ADXL_SELF_TEST		0x80		///< Enable Self-test functionality
#define ADXL_NO_SELF_TEST	0x00		///< Disable Self-test functionality
#define ADXL_SPI_3WIRE		0x40		///< Half-duplex SPI
#define ADXL_SPI_4WIRE		0x00		///< Full-duplex SPI
#define ADXL_INT_ACTIV_HIGH 0x00		///< Rising edge interrupt pin
#define ADXL_INT_ACTIV_LOW	0x20		///< Falling edge interrupt pin
#define ADXL_13BIT_RESOL	0x08		///< 13-bit resolution
#define ADXL_10BIT_RESOL	0x00		///< 10-bit resolution
#define ADXL_LEFT_JUSTIFY	0x04		///< Left-justified values (LSB bits set to 0)
#define ADXL_RIGHT_JUSTIFY	0x00		///< Right-justified values (MSB bits set to 0)
#define ADXL_RANGE_16G		0x03		///< 16G range
#define ADXL_RANGE_8G		0x02		///< 8G range
#define ADXL_RANGE_4G		0x01		///< 4G range
#define ADXL_RANGE_2G		0x00		///< 2G range

// Data rate and power mode control (register 0x2C) configuration values
#define ADXL_POWER_NORMAL	0x00		///< Disable low-power mode
#define ADXL_POWER_LOW		0x10		///< Enable low-power mode (noisier)
#define ADXL_RATE_200HZ		0x0B		///< Set the output data rate to 200Hz 
#define ADXL_RATE_100HZ		0x0A		///< Set the output data rate to 100Hz
#define ADXL_RATE_50HZ		0x09		///< Set the output data rate to 50Hz

// Power Control (register 0x2D) configuration values
#define ADXL_STANDBY_MODE	0x00		///< Set the Standby mode
#define ADXL_MEASURE_MODE	0x08		///< Set the Measurement mode

//constants
#define ADXL_DEVICE_ID			0xE5	///< Device ID value
#define ADXL_HIGH_RESERVED_REG	0x1C	///< Number of the last reserved register
#define ADXL_NB_DATA_REGISTERS	6U		///< Number of output data registers to read from the FIFO

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
