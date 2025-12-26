/*
 * SPDX-FileCopyrightText: 2025 Gilles Henrard <contact@gilleshenrard.com>
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file fonts.c
 * @author Gilles Henrard
 * @brief Declare icons bitmaps where each bit represent either a background or a foreground pixel
 * 
 * @details Generated with the application The Dot Factory v0.1.4
 * 
 * Software Github page : https://github.com/currentleak/the-dot-factory
 *
 * 1. generate bitmaps
 *      - characters : 0123456789+-.°
 *      - Verdana Bold 13pts
 *      - Height fixed and width tightest
 *      - Line wrap at column
 *      - Descriptors : Char width and height in bits, Font height in bits
 *      - Comments : Bitmap visualiser and char descriptor, style C
 *      - Byte : RowMajor, MSB first, Format Hex, leading 0x
 *      - Variable format : Bitmaps "const BitmapRow {0}Bitmaps"
 * 2. copy bitmaps and characters descriptors
 * 3. remove all ", 0x" from bitmaps to merge row bytes together
 * 4. enclose each bitmap with carrets and a comma
 * 5. transform the array in a two-dimensions array
 * 7. assign each bitmap array to an enum entry
 * 8. Use the values in characters descriptors to create a BitmapDescriptor array (width, height, memory address)
 */
#include "fonts.h"

#include <errorstack.h>
#include <stddef.h>
#include <stdint.h>

#include "bitmap.h"

enum {
    kNbSmallCharacters = 47U,  ///< Number of characters in the small font
    kSmallHeightPx = 8U,       ///< Height of the small font in [px]
    kNbLargeCharacters = 20U,  ///< Number of characters in the large font
    kLargeHeightPx = 14U,      ///< Height of the large font in [px]
};

//variables
static const CharacterMetadata kInterV_7pt_alpha_letters[kNbSmallCharacters] =  ///< Metadata of the small characters
    {
        {' ', 6},  /*   */
        {'%', 7},  // %
        {'.', 2},  // .
        {'0', 5},  // 0
        {'1', 3},  // 1
        {'2', 5},  // 2
        {'3', 5},  // 3
        {'4', 5},  // 4
        {'5', 5},  // 5
        {'6', 5},  // 6
        {'7', 6},  // 7
        {'8', 5},  // 8
        {'9', 5},  // 9
        {'A', 7},  /* A */
        {'B', 5},  /* B */
        {'C', 6},  /* C */
        {'D', 6},  /* D */
        {'E', 5},  /* E */
        {'F', 5},  /* F */
        {'G', 6},  /* G */
        {'H', 6},  /* H */
        {'I', 2},  /* I */
        {'J', 5},  /* J */
        {'K', 6},  /* K */
        {'L', 5},  /* L */
        {'M', 7},  /* M */
        {'N', 6},  /* N */
        {'O', 6},  /* O */
        {'P', 5},  /* P */
        {'Q', 6},  /* Q */
        {'R', 5},  /* R */
        {'S', 5},  /* S */
        {'T', 6},  /* T */
        {'U', 6},  /* U */
        {'V', 7},  /* V */
        {'W', 9},  /* W */
        {'X', 7},  /* X */
        {'Y', 7},  /* Y */
        {'Z', 5},  /* Z */
        {'a', 5},  /* a */
        {'b', 5},  /* b */
        {'c', 6},  /* c */
        {'d', 6},  /* d */
        {'e', 6},  /* e */
        {'f', 4},  /* f */
        {'|', 1},  /* | */
        {'*', 4},  /* * */
};

static const CharacterMetadata
    kInterVSemiBold_14pt_alpha_letters[kNbLargeCharacters] =  ///< Metadata of the large characters
    {
        {'+', 9},   // +
        {'-', 7},   // -
        {'.', 3},   // .
        {'0', 11},  // 0
        {'1', 6},   // 1
        {'2', 10},  // 2
        {'3', 10},  // 3
        {'4', 11},  // 4
        {'5', 10},  // 5
        {'6', 10},  // 6
        {'7', 9},   // 7
        {'8', 10},  // 8
        {'9', 10},  // 9
        {'A', 10},  // A
        {'B', 10},  // B
        {'C', 10},  // C
        {'D', 10},  // D
        {'E', 10},  // E
        {'F', 10},  // F
        {'*', 7},   // *, represents '°'
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Uncompress a character in a pixel buffer
 *
 * @param font Font to apply to the character
 * @param character_index Index of the character in the bitmaps pool
 * @param foreground_colour Foreground colour to apply
 * @param bitmap_metadata Metadata of the character bitmap
 * @return Width of the character bitmap in [px] if success, 0 otherwise
 */
uint8_t uncompressCharacter(const FontMetadata* font, uint8_t character_index, ColourBigEndian foreground_colour,
                            Bitmap* bitmap_metadata) {
    if (!font || !bitmap_metadata || !bitmap_metadata->container_width_px) {
        return 0;
    }

    if (character_index >= font->nb_characters) {
        character_index = 0;
    }

    //populate the bitmap metadata
    const CharacterMetadata* char_metadata = &font->descriptors[character_index];
    bitmap_metadata->width_px = char_metadata->width_px;
    bitmap_metadata->height_px = font->height_px;

    //retrieve the character bitmap
    const uint16_t pool_offset = (character_index * font->height_px);
    const BitmapRow* character_bitmap = &font->bitmaps[pool_offset];

    //uncompress the final bitmap
    const ErrorCode result = uncompressBitmap(bitmap_metadata, character_bitmap, foreground_colour);
    return (isError(result) ? 0U : char_metadata->width_px);
}

/**
 * Get the index of a character in the bitmap pool
 *
 * @param font Font of which search the pool
 * @param ascii_character ASCII value of the character
 * @return Index if found (0 will also be returned if not found)
 */
uint8_t getCharIndex(const FontMetadata* font, char ascii_character) {
    if (!font) {
        return 0;
    }

    uint8_t index = 0;
    for (index = 0; index < font->nb_characters; index++) {
        const char character = font->descriptors[index].ascii_character;
        if (character == ascii_character) {
            break;
        }
    }

    if (index >= font->nb_characters) {
        return 0;
    }

    return index;
}

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Small font bitmap pool
 */
static const BitmapRow kInterV_7pt_alpha_pool[kNbSmallCharacters * kSmallHeightPx] = {
    // clang-format off

	// @0 ' ' (4 pixels wide)
	0x0000, //     
	0x0000, //     
	0x0000, //     
	0x0000, //     
	0x0000, //     
	0x0000, //     
	0x0000, //     
	0x0000, //     

	// @8 '%' (6 pixels wide)
	0xC800, // ##  # 
	0xD000, // ## #  
	0xD000, // ## #  
	0x2000, //   #   
	0x5800, //  # ## 
	0x5400, //  # # #
	0x9800, // #  ## 
	0x0000, //       

	// @16 '.' (1 pixels wide)
	0x0000, //  
	0x0000, //  
	0x0000, //  
	0x0000, //  
	0x0000, //  
	0x0000, //  
	0x8000, // #
	0x0000, //  

	// @16 '0' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @24 '1' (2 pixels wide)
	0x4000, //  #
	0xC000, // ##
	0x4000, //  #
	0x4000, //  #
	0x4000, //  #
	0x4000, //  #
	0x4000, //  #
	0x0000, //   

	// @32 '2' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x1000, //    #
	0x2000, //   # 
	0x4000, //  #  
	0x8000, // #   
	0xF000, // ####
	0x0000, //     

	// @40 '3' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x1000, //    #
	0x6000, //  ## 
	0x1000, //    #
	0x9000, // #  #
	0xF000, // ####
	0x0000, //     

	// @48 '4' (4 pixels wide)
	0x2000, //   # 
	0x2000, //   # 
	0x6000, //  ## 
	0xA000, // # # 
	0xA000, // # # 
	0xF000, // ####
	0x2000, //   # 
	0x0000, //     

	// @56 '5' (4 pixels wide)
	0xF000, // ####
	0x8000, // #   
	0x8000, // #   
	0xE000, // ### 
	0x1000, //    #
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @64 '6' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x8000, // #   
	0xF000, // ####
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @72 '7' (5 pixels wide)
	0xF800, // #####
	0x1000, //    # 
	0x1000, //    # 
	0x2000, //   #  
	0x2000, //   #  
	0x4000, //  #   
	0x4000, //  #   
	0x0000, //      

	// @80 '8' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x9000, // #  #
	0x6000, //  ## 
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @88 '9' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0x7000, //  ###
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @96 'A' (6 pixels wide)
	0x2000, //   #   
	0x3000, //   ##  
	0x3000, //   ##  
	0x4800, //  #  # 
	0x7800, //  #### 
	0x4800, //  #  # 
	0x8400, // #    #
	0x0000, //       

	// @104 'B' (4 pixels wide)
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0xF000, // ####
	0x0000, //     

	// @112 'C' (5 pixels wide)
	0x2000, //   #  
	0xD800, // ## ##
	0x8800, // #   #
	0x8000, // #    
	0x8000, // #    
	0x8800, // #   #
	0x7000, //  ### 
	0x0000, //      

	// @120 'D' (5 pixels wide)
	0xE000, // ###  
	0x9000, // #  # 
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0xF000, // #### 
	0x0000, //      

	// @128 'E' (4 pixels wide)
	0xF000, // ####
	0x8000, // #   
	0x8000, // #   
	0xF000, // ####
	0x8000, // #   
	0x8000, // #   
	0xF000, // ####
	0x0000, //     

	// @136 'F' (4 pixels wide)
	0xF000, // ####
	0x8000, // #   
	0x8000, // #   
	0xE000, // ### 
	0x8000, // #   
	0x8000, // #   
	0x8000, // #   
	0x0000, //     

	// @144 'G' (5 pixels wide)
	0x6000, //  ##  
	0x9800, // #  ##
	0x8800, // #   #
	0x8000, // #    
	0x9800, // #  ##
	0x8800, // #   #
	0x7000, //  ### 
	0x0000, //      

	// @152 'H' (5 pixels wide)
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0xF800, // #####
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x0000, //      

	// @160 'I' (1 pixels wide)
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x0000, //  

	// @168 'J' (4 pixels wide)
	0x1000, //    #
	0x1000, //    #
	0x1000, //    #
	0x1000, //    #
	0x1000, //    #
	0x9000, // #  #
	0x7000, //  ###
	0x0000, //     

	// @176 'K' (5 pixels wide)
	0x8800, // #   #
	0x9000, // #  # 
	0xA000, // # #  
	0xC000, // ##   
	0xA000, // # #  
	0xA000, // # #  
	0x9800, // #  ##
	0x0000, //      

	// @184 'L' (4 pixels wide)
	0x8000, // #   
	0x8000, // #   
	0x8000, // #   
	0x8000, // #   
	0x8000, // #   
	0x8000, // #   
	0xF000, // ####
	0x0000, //     

	// @192 'M' (6 pixels wide)
	0x8400, // #    #
	0x8400, // #    #
	0xCC00, // ##  ##
	0xCC00, // ##  ##
	0xD400, // ## # #
	0xB400, // # ## #
	0xB400, // # ## #
	0x0000, //       

	// @200 'N' (5 pixels wide)
	0x8800, // #   #
	0x8800, // #   #
	0xC800, // ##  #
	0xA800, // # # #
	0xA800, // # # #
	0x9800, // #  ##
	0x8800, // #   #
	0x0000, //      

	// @208 'O' (5 pixels wide)
	0x2000, //   #  
	0xD800, // ## ##
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x7000, //  ### 
	0x0000, //      

	// @216 'P' (4 pixels wide)
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x8000, // #   
	0x8000, // #   
	0x0000, //     

	// @224 'Q' (5 pixels wide)
	0x2000, //   #  
	0xD800, // ## ##
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0xA800, // # # #
	0x7000, //  ### 
	0x0800, //     #

	// @232 'R' (4 pixels wide)
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0x0000, //     

	// @240 'S' (4 pixels wide)
	0x6000, //  ## 
	0x9000, // #  #
	0x8000, // #   
	0xE000, // ### 
	0x1000, //    #
	0x9000, // #  #
	0xF000, // ####
	0x0000, //     

	// @248 'T' (5 pixels wide)
	0xF800, // #####
	0x2000, //   #  
	0x2000, //   #  
	0x2000, //   #  
	0x2000, //   #  
	0x2000, //   #  
	0x2000, //   #  
	0x0000, //      

	// @256 'U' (5 pixels wide)
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x7000, //  ### 
	0x0000, //      

	// @264 'V' (6 pixels wide)
	0x8400, // #    #
	0x8800, // #   # 
	0x4800, //  #  # 
	0x4800, //  #  # 
	0x5000, //  # #  
	0x3000, //   ##  
	0x3000, //   ##  
	0x0000, //       

	// @272 'W' (8 pixels wide)
	0x8900, // #   #  #
	0x8900, // #   #  #
	0x5900, //  # ##  #
	0x5500, //  # # # #
	0x6600, //  ##  ## 
	0x2600, //   #  ## 
	0x2200, //   #   # 
	0x0000, //         

	// @280 'X' (6 pixels wide)
	0x8400, // #    #
	0x4800, //  #  # 
	0x5000, //  # #  
	0x3000, //   ##  
	0x3000, //   ##  
	0x4800, //  #  # 
	0x8C00, // #   ##
	0x0000, //       

	// @288 'Y' (6 pixels wide)
	0x8400, // #    #
	0x4800, //  #  # 
	0x5000, //  # #  
	0x3000, //   ##  
	0x2000, //   #   
	0x2000, //   #   
	0x2000, //   #   
	0x0000, //       

	// @296 'Z' (4 pixels wide)
	0xF000, // ####
	0x1000, //    #
	0x2000, //   # 
	0x4000, //  #  
	0x4000, //  #  
	0x8000, // #   
	0xF000, // ####
	0x0000, //

		// @312 'a' (4 pixels wide)
	0x0000, //     
	0x0000, //     
	0x7000, //  ###
	0x1000, //    #
	0x7000, //  ###
	0x9000, // #  #
	0x7000, //  ###
	0x0000, //     

	// @320 'b' (4 pixels wide)
	0x8000, // #   
	0x8000, // #   
	0xE000, // ### 
	0x9000, // #  #
	0x9000, // #  #
	0x9000, // #  #
	0xE000, // ### 
	0x0000, //     

	// @328 'c' (5 pixels wide)
	0x0000, //      
	0x0000, //      
	0x7000, //  ### 
	0x8800, // #   #
	0x8000, // #    
	0x8000, // #    
	0x7800, //  ####
	0x0000, //      

	// @336 'd' (5 pixels wide)
	0x0800, //     #
	0x0800, //     #
	0x7800, //  ####
	0x8800, // #   #
	0x8800, // #   #
	0x8800, // #   #
	0x7800, //  ####
	0x0000, //      

	// @344 'e' (5 pixels wide)
	0x0000, //      
	0x0000, //      
	0x7000, //  ### 
	0x8800, // #   #
	0xF800, // #####
	0x8000, // #    
	0x7800, //  ####
	0x0000, //      

	// @352 'f' (3 pixels wide)
	0x6000, //  ##
	0x4000, //  # 
	0xE000, // ###
	0x4000, //  # 
	0x4000, //  # 
	0x4000, //  # 
	0x4000, //  # 
	0x0000, //

	/* @484 '|' (1 pixels wide) */
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #
	0x8000, // #

	// @352 '°' (3 pixels wide)
	0x4000, //  # 
	0xC000, // ## 
	0xE000, // ###
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //

    // clang-format on
};

/**
 * Large font bitmap pool
 */
static const BitmapRow kInterVSemiBold_14pt_num_pool[kNbLargeCharacters * kLargeHeightPx] = {
    // clang-format off

	// @0 '+' (9 pixels wide)
	0x0000, //          
	0x0000, //          
	0x0000, //          
	0x0000, //          
	0x1800, //    ##    
	0x1800, //    ##    
	0x1800, //    ##    
	0xFF80, // #########
	0xFF80, // #########
	0xFF80, // #########
	0x1800, //    ##    
	0x1800, //    ##    
	0x1800, //    ##    
	0x0000, //          

	// @28 '-' (7 pixels wide)
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0xFE00, // #######
	0xFE00, // #######
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        

	// @42 '.' (3 pixels wide)
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x0000, //    
	0x6000, //  ##
	0xE000, // ###
	0x6000, //  ##

	// @56 '0' (11 pixels wide)
	0x1F00, //    #####   
	0x3F80, //   #######  
	0x71C0, //  ###   ### 
	0x61C0, //  ##    ### 
	0xE0C0, // ###     ## 
	0xE0E0, // ###     ###
	0xE0E0, // ###     ###
	0xE0E0, // ###     ###
	0xE0E0, // ###     ###
	0xE0C0, // ###     ## 
	0xE0C0, // ###     ## 
	0x71C0, //  ###   ### 
	0x3F80, //   #######  
	0x1F00, //    #####   

	// @84 '1' (6 pixels wide)
	0x1C00, //    ###
	0x7C00, //  #####
	0xFC00, // ######
	0xDC00, // ## ###
	0x9C00, // #  ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###
	0x1C00, //    ###

	// @98 '2' (10 pixels wide)
	0x3E00, //   #####   
	0x7F80, //  ######## 
	0xE380, // ###   ### 
	0xE180, // ###    ## 
	0x0180, //        ## 
	0x0180, //        ## 
	0x0380, //       ### 
	0x0700, //      ###  
	0x0E00, //     ###   
	0x1C00, //    ###    
	0x3800, //   ###     
	0x7000, //  ###      
	0xFFC0, // ##########
	0xFFC0, // ##########

	// @126 '3' (10 pixels wide)
	0x1F00, //    #####  
	0x7F80, //  ######## 
	0x71C0, //  ###   ###
	0xE1C0, // ###    ###
	0x01C0, //        ###
	0x0180, //        ## 
	0x0F00, //     ####  
	0x0F80, //     ##### 
	0x01C0, //        ###
	0x00C0, //         ##
	0xE0C0, // ###     ##
	0xE1C0, // ###    ###
	0x7F80, //  ######## 
	0x3F00, //   ######  

	// @154 '4' (11 pixels wide)
	0x0380, //       ###  
	0x0780, //      ####  
	0x0F80, //     #####  
	0x0D80, //     ## ##  
	0x1D80, //    ### ##  
	0x3980, //   ###  ##  
	0x3180, //   ##   ##  
	0x7180, //  ###   ##  
	0x6180, //  ##    ##  
	0xFFE0, // ###########
	0xFFE0, // ###########
	0x0180, //        ##  
	0x0180, //        ##  
	0x0180, //        ##  

	// @182 '5' (10 pixels wide)
	0x7F80, //  ######## 
	0x7F80, //  ######## 
	0x6000, //  ##       
	0x6000, //  ##       
	0x6000, //  ##       
	0x6F00, //  ## ####  
	0x7F80, //  ######## 
	0x21C0, //   #    ###
	0x01C0, //        ###
	0x00C0, //         ##
	0xE1C0, // ###    ###
	0x61C0, //  ##    ###
	0x7F80, //  ######## 
	0x3F00, //   ######  

	// @210 '6' (10 pixels wide)
	0x1F00, //    #####  
	0x3F80, //   ####### 
	0x71C0, //  ###   ###
	0x60C0, //  ##     ##
	0xE000, // ###       
	0xEF00, // ### ####  
	0xDF80, // ## ###### 
	0xF1C0, // ####   ###
	0xE0C0, // ###     ##
	0xE0C0, // ###     ##
	0xE0C0, // ###     ##
	0x71C0, //  ###   ###
	0x7F80, //  ######## 
	0x1F00, //    #####  

	// @238 '7' (9 pixels wide)
	0xFF80, // #########
	0xFF80, // #########
	0x0180, //        ##
	0x0380, //       ###
	0x0300, //       ## 
	0x0700, //      ### 
	0x0600, //      ##  
	0x0E00, //     ###  
	0x0C00, //     ##   
	0x1C00, //    ###   
	0x1800, //    ##    
	0x3800, //   ###    
	0x7000, //  ###     
	0x7000, //  ###     

	// @266 '8' (10 pixels wide)
	0x1F00, //    #####  
	0x7F80, //  ######## 
	0x71C0, //  ###   ###
	0x61C0, //  ##    ###
	0x61C0, //  ##    ###
	0x7180, //  ###   ## 
	0x3F00, //   ######  
	0x7F80, //  ######## 
	0xE1C0, // ###    ###
	0xE0C0, // ###     ##
	0xE0C0, // ###     ##
	0xE1C0, // ###    ###
	0x7F80, //  ######## 
	0x3F00, //   ######  

	// @294 '9' (10 pixels wide)
	0x1E00, //    ####   
	0x7F80, //  ######## 
	0x7180, //  ###   ## 
	0xE1C0, // ###    ###
	0xC0C0, // ##      ##
	0xE1C0, // ###    ###
	0xE1C0, // ###    ###
	0x7FC0, //  #########
	0x3EC0, //   ##### ##
	0x00C0, //         ##
	0xE1C0, // ###    ###
	0x6380, //  ##   ### 
	0x7F80, //  ######## 
	0x3E00, //   #####   

		// @266 'A' (13 pixels wide)
	0x0780, //      ####    
	0x0780, //      ####    
	0x0780, //      ####    
	0x0FC0, //     ######   
	0x0DC0, //     ## ###   
	0x1CC0, //    ###  ##   
	0x1CE0, //    ###  ###  
	0x18E0, //    ##   ###  
	0x3860, //   ###    ##  
	0x3FF0, //   ########## 
	0x3FF0, //   ########## 
	0x7030, //  ###      ## 
	0x7038, //  ###      ###
	0xE038, // ###       ###

	// @294 'B' (11 pixels wide)
	0xFF00, // ########   
	0xFF80, // #########  
	0xE1C0, // ###    ### 
	0xE0C0, // ###     ## 
	0xE0C0, // ###     ## 
	0xE1C0, // ###    ### 
	0xFF00, // ########   
	0xFF80, // #########  
	0xE1C0, // ###    ### 
	0xE0E0, // ###     ###
	0xE0E0, // ###     ###
	0xE1C0, // ###    ### 
	0xFFC0, // ########## 
	0xFF80, // #########  

	// @322 'C' (12 pixels wide)
	0x0F80, //     #####   
	0x3FE0, //   ######### 
	0x78E0, //  ####   ### 
	0x7070, //  ###     ###
	0xE030, // ###       ##
	0xE000, // ###         
	0xE000, // ###         
	0xE000, // ###         
	0xE000, // ###         
	0xE030, // ###       ##
	0x6070, //  ##      ###
	0x78E0, //  ####   ### 
	0x3FE0, //   ######### 
	0x1F80, //    ######   

	// @350 'D' (12 pixels wide)
	0xFE00, // #######     
	0xFF80, // #########   
	0xE3C0, // ###   ####  
	0xE0E0, // ###     ### 
	0xE060, // ###      ## 
	0xE070, // ###      ###
	0xE070, // ###      ###
	0xE070, // ###      ###
	0xE070, // ###      ###
	0xE060, // ###      ## 
	0xE0E0, // ###     ### 
	0xE1E0, // ###    #### 
	0xFFC0, // ##########  
	0xFF00, // ########    

	// @378 'E' (9 pixels wide)
	0xFF80, // #########
	0xFF80, // #########
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xFF80, // #########
	0xFF80, // #########
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xFF80, // #########
	0xFF80, // #########

	// @406 'F' (9 pixels wide)
	0xFF80, // #########
	0xFF80, // #########
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xFF80, // #########
	0xFF80, // #########
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ###      
	0xE000, // ### 

	// @322 '°' (7 pixels wide)
	0x3800, //   ###  
	0x7C00, //  ##### 
	0xC400, // ##   # 
	0xC600, // ##   ##
	0xCC00, // ##  ## 
	0x7C00, //  ##### 
	0x1000, //    #   
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //        
	0x0000, //

    // clang-format on
};

/*********************************************************************************************************************************/
/*********************************************************************************************************************************/

/**
 * Small font characters' metadata
 */
const FontMetadata kInterV_7pt_alpha_descriptor = {
    .height_px = kSmallHeightPx,
    .nb_characters = kNbSmallCharacters,
    .descriptors = kInterV_7pt_alpha_letters,
    .bitmaps = kInterV_7pt_alpha_pool,
};

/**
 * Large font characters' metadata
 */
const FontMetadata kInterVSemiBold_14_num_descriptor = {
    .height_px = kLargeHeightPx,
    .nb_characters = kNbLargeCharacters,
    .descriptors = kInterVSemiBold_14pt_alpha_letters,
    .bitmaps = kInterVSemiBold_14pt_num_pool,
};
