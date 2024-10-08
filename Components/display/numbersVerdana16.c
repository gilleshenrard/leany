/**
 * @file numbersVerdana16.c
 * @brief Declares the Verdana 16pixels bitmaps needed to be displayed on the SSD1306 screen in this application
 * @author Gilles Henrard
 * @date 26/07/24
 *
 * @details Generated with the application The Dot Factory v0.1.4
 * 
 * Software Github page : https://github.com/pavius/the-dot-factory
 *
 * 1. Settings to reproduce :
 * - Font : Verdana 16 pts
 * - Padding removal : height and width fixed
 * - Line wrap : At column
 * - Byte : ColumnMajor, MSB first
 * - Descriptors : No descriptor lookup array
 * 
 * 2. Generate the bitmaps and copy in code
 * 3. Get rid of the font info structure at end of document
 * 4. Change the main array type to a three-dimensional array
 * 5. Encase each character bitmap in its own arrays (preferably marked with an index)
 * 6. Reorganise elements so that numbers are first (to match index values)
 */
// clang-format off
#include "numbersVerdana16.h"
#include <stdint.h>

/**
 * @brief Verdana 16 bitmaps
 */
const uint8_t verdana_16ptNumbers[NB_NUMBERS][VERDANA_NB_PAGES][VERDANA_CHAR_WIDTH] =
{
    // @44 '0' (14 pixels wide)
    //     #####     
    //    #######    
    //   ###   ###   
    //   ##     ##   
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //   ##     ##   
    //   ###   ###   
    //    #######    
    //     #####     
    [INDEX_0] = {
        {0x00, 0xF0, 0xFC, 0x0E, 0x07, 0x03, 0x03, 0x03, 0x07, 0x0E, 0xFC, 0xF0, 0x00, 0x00},
        {0x00, 0x0F, 0x3F, 0x70, 0xE0, 0xC0, 0xC0, 0xC0, 0xE0, 0x70, 0x3F, 0x0F, 0x00, 0x00}
    },

    // @66 '1' (14 pixels wide)
    //       ##      
    //       ##      
    //    #####      
    //    #####      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //    ########   
    //    ########   
    [INDEX_1] = {
        {0x00, 0x00, 0x00, 0x0C, 0x0C, 0x0C, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0x00, 0xC0, 0xC0, 0xC0, 0xFF, 0xFF, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00}
    },

    // @88 '2' (14 pixels wide)
    //    ######     
    //   ########    
    //   #     ###   
    //          ##   
    //          ##   
    //          ##   
    //          ##   
    //         ##    
    //        ##     
    //       ###     
    //      ###      
    //     ###       
    //    ###        
    //   ###         
    //   ##########  
    //   ##########  
    [INDEX_2] = {
        {0x00, 0x00, 0x06, 0x03, 0x03, 0x03, 0x03, 0x03, 0x87, 0xFE, 0x7C, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0xE0, 0xF0, 0xF8, 0xDC, 0xCE, 0xC7, 0xC3, 0xC0, 0xC0, 0xC0, 0x00, 0x00}
    },

    // @110 '3' (14 pixels wide)
    //    ######     
    //   ########    
    //   #     ###   
    //          ##   
    //          ##   
    //         ##    
    //      ####     
    //      ####     
    //         ##    
    //          ##   
    //          ##   
    //          ##   
    //          ##   
    //   #     ##    
    //   ########    
    //    #####      
    [INDEX_3] = {
        {0x00, 0x00, 0x06, 0x03, 0x03, 0xC3, 0xC3, 0xC3, 0xE7, 0x3E, 0x1C, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0x60, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x61, 0x7F, 0x1E, 0x00, 0x00, 0x00}
    },

    // @132 '4' (14 pixels wide)
    //         ##    
    //        ###    
    //       ####    
    //      #####    
    //     ### ##    
    //     ##  ##    
    //    ##   ##    
    //   ###   ##    
    //  ###    ##    
    //  ###########  
    //  ###########  
    //         ##    
    //         ##    
    //         ##    
    //         ##    
    //         ##    
    [INDEX_4] = {
        {0x00, 0x00, 0x80, 0xC0, 0xF0, 0x38, 0x1C, 0x0E, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00}, 
        {0x00, 0x07, 0x07, 0x07, 0x06, 0x06, 0x06, 0x06, 0xFF, 0xFF, 0x06, 0x06, 0x00, 0x00}
    },

    // @154 '5' (14 pixels wide)
    //    #########  
    //    #########  
    //    ##         
    //    ##         
    //    ##         
    //    ##         
    //    #######    
    //    ########   
    //          ###  
    //           ##  
    //           ##  
    //           ##  
    //           ##  
    //   #      ##   
    //   #########   
    //    ######     
    [INDEX_5] = {
        {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xC3, 0xC3, 0xC3, 0xC3, 0xC3, 0x83, 0x03, 0x00, 0x00}, 
        {0x00, 0x00, 0x60, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x61, 0x7F, 0x1F, 0x00, 0x00}
    },

    // @176 '6' (14 pixels wide)
    //      #####    
    //     ######    
    //    ###        
    //   ##          
    //   ##          
    //  ##           
    //  ## #####     
    //  ##########   
    //  ###     ###  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //   ##      ##  
    //   ###    ##   
    //    #######    
    //     #####     
    [INDEX_6] = {
        {0x00, 0xE0, 0xF8, 0x9C, 0xC6, 0xC7, 0xC3, 0xC3, 0xC3, 0x83, 0x80, 0x00, 0x00, 0x00}, 
        {0x00, 0x0F, 0x3F, 0x71, 0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0x61, 0x3F, 0x1F, 0x00, 0x00}
    },

    // @198 '7' (14 pixels wide)
    //   ##########  
    //   ##########  
    //           ##  
    //           ##  
    //          ##   
    //          ##   
    //         ##    
    //         ##    
    //        ##     
    //       ###     
    //       ##      
    //      ###      
    //      ##       
    //     ###       
    //     ##        
    //    ###      
    [INDEX_7] = {
        {0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0xC3, 0xF3, 0x3F, 0x0F, 0x00, 0x00}, 
        {0x00, 0x00, 0x00, 0x80, 0xE0, 0xF8, 0x3E, 0x0F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    // @220 '8' (14 pixels wide)
    //     #####     
    //   #########   
    //  ###     ###  
    //  ##       ##  
    //  ##       ##  
    //  ###      ##  
    //   ####  ###   
    //     #####     
    //   ##   ####   
    //  ##      ###  
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ###     ##   
    //   #########   
    //     #####     
    [INDEX_8] = {
        {0x00, 0x3C, 0x7E, 0x66, 0xC3, 0xC3, 0x83, 0x83, 0xC3, 0x46, 0x7E, 0x3C, 0x00, 0x00}, 
        {0x00, 0x3E, 0x7F, 0x61, 0xC0, 0xC0, 0xC0, 0xC1, 0xC1, 0x63, 0x7F, 0x1E, 0x00, 0x00}
    },

    // @242 '9' (14 pixels wide)
    //     #####     
    //    #######    
    //   ##    ###   
    //  ##      ##   
    //  ##       ##  
    //  ##       ##  
    //  ##       ##  
    //  ###     ###  
    //   ##########  
    //     ##### ##  
    //           ##  
    //          ##   
    //          ##   
    //        ###    
    //    ######     
    //    #####      
    [INDEX_9] = {
        {0x00, 0xF8, 0xFC, 0x86, 0x03, 0x03, 0x03, 0x03, 0x07, 0x8E, 0xFC, 0xF0, 0x00, 0x00}, 
        {0x00, 0x00, 0x01, 0xC1, 0xC3, 0xC3, 0xC3, 0xE3, 0x63, 0x39, 0x1F, 0x07, 0x00, 0x00}
    },

    /* @0 '+' (14 pixels wide) */
    //               
    //               
    //               
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //  ############ 
    //  ############ 
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //       ##      
    //               
    [INDEX_PLUS] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
        {0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x7F, 0x7F, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00}
    },

    // @0 '-' (14 pixels wide)
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //    ########   
    //    ########   
    //               
    //               
    //               
    //               
    //               
    //               
    [INDEX_MINUS] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0x00, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x00, 0x00, 0x00}
    },

    // @22 '.' (14 pixels wide)
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //       ##      
    //       ##      
    //       ##      
    [INDEX_DOT] = {
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },

    // @264 '°' (14 pixels wide)
    //      ####     
    //     ######    
    //    ###  ###   
    //    ##    ##   
    //    ##    ##   
    //    ###  ###   
    //     ######    
    //      ####     
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    //               
    [INDEX_DEG] = {
        {0x00, 0x00, 0x00, 0x3C, 0x7E, 0xE7, 0xC3, 0xC3, 0xE7, 0x7E, 0x3C, 0x00, 0x00, 0x00}, 
        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    },
};
