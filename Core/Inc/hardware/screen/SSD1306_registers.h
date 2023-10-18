#ifndef INC_HARDWARE_SCREEN_SSD1306_REGISTERS_H_
#define INC_HARDWARE_SCREEN_SSD1306_REGISTERS_H_

#define SSD_MUX_RATIO_64		0x3F	///< Value to set the multiplexer ratio to 64 (reset value)
#define SSD_OFFSET_0			0x00	///< Value to set the screen offset to 0 (reset value)
#define SSD_START_LINE_0		0x00	///< Value to set the start line to 0 (reset value)

#define SSD_CONTRAST_LOWEST		0x00	///< Value to set the contrast to the lowest value
#define SSD_CONTRAST_MID		0x7F	///< Value to set the contrast to the middle value (reset value)
#define SSD_CONTRAST_HIGHEST	0xFF	///< Value to set the contrast to the highest value

#define SSD_CLOCK_DIVIDER_1		0x00	///< Value to set the clock divide ratio to 1
#define SSD_CLOCK_DIVIDER_16	0x0F	///< Value to set the clock divide ratio to 16
#define SSD_CLOCK_FREQ_MID		0x80	///< Value to set the middle clock oscillator frequency
#define SSD_CLOCK_FREQ_MAX		0xF0	///< Value to set the maximum clock oscillator frequency

#define SSD_DISABLE_CHG_PUMP	0x10	///< Value to disable the charge pump
#define SSD_ENABLE_CHG_PUMP		0x14	///< Value to enable the charge pump

#define SSD_HORIZONTAL_ADDR		0x00	///< Value to set the horizontal addressing mode
#define SSD_VERTICAL_ADDR		0x01	///< Value to set the vertical addressing mode
#define SSD_PAGE_ADDR			0x02	///< Value to set the page addressing mode (reset value)

/**
 * @brief Enumeration of all the SSD1306 registers listed in the datasheet
 */
typedef enum{
	LOW_COL_START_ADDR	= 0x00,	///< Set Lower Column Start Address for Page Addressing Mode
	HIGH_COL_START_ADDR	= 0x10,	///< Set Higher Column Start Address for Page Addressing Mode
	MEMORY_ADDR_MODE	= 0x20,	///< Set Memory Addressing Mode
	COLUMN_ADDRESS		= 0x21,	///< Set Column start and end address in horizontal or vertical addressing mode
	PAGE_ADDRESS		= 0x22,	///< Set Page start and end Address in horizontal or vertical addressing mode
	SCROLL_HOR_RIGHT	= 0x26,	///< Continuous Horizontal Scroll Setup - Right Horizontal Scroll
	SCROLL_HOR_LEFT		= 0x27,	///< Continuous Horizontal Scroll Setup - Left Horizontal Scroll
	SCROLL_BOTH_RIGHT	= 0x29,	///< Continuous Vertical and Horizontal Scroll Setup - Vertical and Right Horizontal Scroll
	SCROLL_BOTH_LEFT	= 0x2A,	///< Continuous Vertical and Horizontal Scroll Setup - Vertical and Left Horizontal Scroll
	SCROLL_DISABLE		= 0x2E,	///< Deactivate scroll
	SCROLL_ENABLE		= 0x2F,	///< Activate scroll
	DISPLAY_START_LINE	= 0x40,	///< Set Display Start Line
	CONTRAST_CONTROL	= 0x81,	///< Set Contrast Control
	CHG_PUMP_REGULATOR	= 0x8D,	///< Charge Pump Setting
	SEGMENT_REMAP_0		= 0xA0,	///< Set Segment Re-map - column address 0 is mapped to SEG0
	SEGMENT_REMAP_127	= 0xA1,	///< Set Segment Re-map - column address 127 is mapped to SEG0
	SCROLL_VER_AREA		= 0xA3,	///< Set Vertical Scroll Area
	DISPLAY_FOLLOW_RAM	= 0xA4,	///< Entire Display ON - Output follows RAM content
	DISPLAY_ALL_ON		= 0xA5,	///< Entire Display ON - Output ignores RAM content
	DISPLAY_NORMAL		= 0xA6,	///< Set Normal/Inverse Display - Normal display
	DISPLAY_INVERSE		= 0xA7,	///< Set Normal/Inverse Display - Inverse display
	MUX_RATIO			= 0xA8,	///< Set Multiplex Ratio
	DISPLAY_OFF			= 0xAE,	///< Set Display ON/OFF - Display OFF
	DISPLAY_ON			= 0xAF,	///< Set Display ON/OFF - Display ON
	ADDR_MODE_PAGESTART = 0xB0,	///< Set Page Start Address for Page Addressing Mode
	SCAN_DIRECTION_0_N1	= 0xC0,	///< Set COM Output Scan Direction - Scan from COM0 to COM[N–1]
	SCAN_DIRECTION_N1_0	= 0xC8,	///< Set COM Output Scan Direction - Scan from COM[N-1] to COM0
	DISPLAY_OFFSET		= 0xD3,	///< Set Display Offset
	CLOCK_DIVIDE_RATIO	= 0xD5,	///< Set Display Clock Divide Ratio/Oscillator Frequency
	PRECHARGE_PERIOD	= 0xD9,	///< Set Pre-charge Period
	HARDWARE_CONFIG		= 0xDA,	///< Set COM Pins Hardware Configuration
	VCOMH_DESELECT_LVL	= 0xDB,	///< Set VCOMH Deselect Level
	NOP					= 0xE3,	///< Command for no operation
}SSD1306register_e;

#endif /* INC_HARDWARE_SCREEN_SSD1306_REGISTERS_H_ */
