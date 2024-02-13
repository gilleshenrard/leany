#ifndef INC_ADXL345REGISTERS_H_
#define INC_ADXL345REGISTERS_H_

#define ADXL_DEVICE_ID		0xE5

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

#define ADXL_INT_DISABLED	0x00
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
#define ADXL_10BIT_RESOL	0x00
#define ADXL_FULL_RESOL		0x08
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

#define ADXL_ST_MAXX_33_16G	 118
#define ADXL_ST_MINX_33_16G	 10
#define ADXL_ST_MAXY_33_16G	-10
#define ADXL_ST_MINY_33_16G	-118
#define ADXL_ST_MAXZ_33_16G	 161
#define ADXL_ST_MINZ_33_16G	 14


#define ADXL_HIGH_RESERVED_REG	0x1C	///< Number of the last reserved register
#define ADXL_NB_DATA_REGISTERS	6U		///< Number of the last reserved register

typedef enum{
	DEVICE_ID 		= 0x00,
	TAP_THRESHOLD	= 0x1D,		//Registers 0x01 to 0x1C are reserved
	OFFSET_X,
	OFFSET_Y,
	OFFSET_Z,
	TAP_DURATION,
	TAP_LATENCY,
	TAP_WINDOW,
	ACTIVITY_THRESHOLD,
	INACTIVITY_THRESHOLD,
	INACTIVITY_TIME,
	ACTIVITY_CONTROL,
	FREEFALL_THRESHOLD,
	FREEFALL_TIME,
	TAP_AXES,
	TAP_ACTIVITY_SOURCE,
	BANDWIDTH_POWERMODE,
	POWER_CONTROL,
	INTERRUPT_ENABLE,
	INTERRUPT_MAPPING,
	INTERRUPT_SOURCE,
	DATA_FORMAT,
	DATA_X0,
	DATA_X1,
	DATA_Y0,
	DATA_Y1,
	DATA_Z0,
	DATA_Z1,
	FIFO_CONTROL,
	FIFO_STATUS,
	ADXL_NB_REGISTERS
}adxl345Registers_e;


#endif /* INC_ADXL345REGISTERS_H_ */
