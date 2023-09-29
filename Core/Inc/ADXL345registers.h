#ifndef INC_ADXL345REGISTERS_H_
#define INC_ADXL345REGISTERS_H_

#define ADXL_WRITE			0x00		///< MSB configuration for write operations
#define ADXL_READ			0x80		///< MSB configuration for read operations
#define ADXL_SINGLE			0x00		///< Bit 6 configuration for single register operations
#define ADXL_MULTIPLE		0x40		///< Bit 6 configuration for multiple register operations

#define ADXL_MODE_BYPASS	0x00
#define ADXL_MODE_FIFO		0x40
#define ADXL_MODE_STREAM	0x80
#define ADXL_MODE_TRIGGER	0xC0
#define ADXL_TRIGGER_INT1	0x00
#define ADXL_TRIGGER_INT2	0x20
#define ADXL_SAMPLES_32		0x20
#define ADXL_SAMPLES_16		0x10
#define ADXL_SAMPLES_8		0x08

#define ADXL_INT_DATARDY	0x80
#define ADXL_INT_SINGLETAP	0x40
#define ADXL_INT_DOUBLETAP	0x20
#define ADXL_INT_ACTIVITY	0x10
#define ADXL_INT_INACTIVITY	0x08
#define ADXL_INT_FREEFALL	0x04
#define ADXL_INT_WATERMARK	0x02
#define ADXL_INT_OVERRUN	0x01
#define ADXL_INT_MAP_INT1	0x00

#define ADXL_SELF_TEST		0x80
#define ADXL_NO_SELF_TEST	0x00
#define ADXL_SPI_3WIRE		0x40
#define ADXL_SPI_4WIRE		0x00
#define ADXL_INT_ACTIV_HIGH 0x00
#define ADXL_INT_ACTIV_LOW	0x20
#define ADXL_FULL_RESOL		0x08
#define ADXL_LEFT_JUSTIFY	0x04
#define ADXL_RIGHT_JUSTIFY	0x00
#define ADXL_RANGE_2G		0x00
#define ADXL_RANGE_4G		0x01
#define ADXL_RANGE_8G		0x02
#define ADXL_RANGE_16G		0x03

#define ADXL_STANDBY_MODE	0x00	///< Power control bit 3 configuration for standby mode
#define ADXL_MEASURE_MODE	0x08	///< Power control bit 3 configuration for measurement mode


#define ADXL_HIGH_RESERVED_REG	0x1C	///< Number of the last reserved register
#define ADXL_NB_DATA_REGISTERS	6
typedef enum{
	DEVICE_ID 		= 0x00,  ///< DEVICE_ID
	TAP_THRESHOLD	= 0x1D,		//Registers 0x01 to 0x1C are reserved
	OFFSET_X,            ///< OFFSET_X
	OFFSET_Y,            ///< OFFSET_Y
	OFFSET_Z,            ///< OFFSET_Z
	TAP_DURATION,        ///< TAP_DURATION
	TAP_LATENCY,         ///< TAP_LATENCY
	TAP_WINDOW,          ///< TAP_WINDOW
	ACTIVITY_THRESHOLD,  ///< ACTIVITY_THRESHOLD
	INACTIVITY_THRESHOLD,///< INACTIVITY_THRESHOLD
	INACTIVITY_TIME,     ///< INACTIVITY_TIME
	ACTIVITY_CONTROL,    ///< ACTIVITY_CONTROL
	FREEFALL_THRESHOLD,  ///< FREEFALL_THRESHOLD
	FREEFALL_TIME,       ///< FREEFALL_TIME
	TAP_AXES,            ///< TAP_AXES
	TAP_ACTIVITY_SOURCE, ///< TAP_ACTIVITY_SOURCE
	BANDWIDTH_POWERMODE, ///< BANDWIDTH_POWERMODE
	POWER_CONTROL,       ///< POWER_CONTROL
	INTERRUPT_ENABLE,    ///< INTERRUPT_ENABLE
	INTERRUPT_MAPPING,   ///< INTERRUPT_MAPPING
	INTERRUPT_SOURCE,    ///< INTERRUPT_SOURCE
	DATA_FORMAT,         ///< DATA_FORMAT
	DATA_X0,             ///< DATA_X0
	DATA_X1,             ///< DATA_X1
	DATA_Y0,             ///< DATA_Y0
	DATA_Y1,             ///< DATA_Y1
	DATA_Z0,             ///< DATA_Z0
	DATA_Z1,             ///< DATA_Z1
	FIFO_CONTROL,        ///< FIFO_CONTROL
	FIFO_STATUS,         ///< FIFO_STATUS
	ADXL_NB_REGISTERS    ///< ADXL_NB_REGISTERS
}adxl345Registers_e;


#endif /* INC_ADXL345REGISTERS_H_ */
