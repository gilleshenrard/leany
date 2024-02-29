#include "icons16.h"

const uint8_t icons_16pt[NB_ICONS][ICONS_NB_CHARS] =
{
    // Vertical arrows (16 pixels wide)
    //        #        
    //       ###       
    //      #####      
    //     #######     
    //       ###       
    //       ###       
    //       ###       
    //       ###       
    //       ###       
    //       ###       
    //       ###       
    //       ###       
    //     #######     
    //      #####      
    //       ###       
    //        #        
    [ARROWS_VERTICAL] = { 
        0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0xFE, 0xFF, 0xFE, 0x0C, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x10, 0x30, 0x7F, 0xFF, 0x7F, 0x30, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
    },

    // Horizontal arrows (16 pixels wide)
    //                 
    //                 
    //                 
    //                 
    //    #        #   
    //   ##        ##  
    //  ############## 
    // ################
    //  ############## 
    //   ##        ##  
    //    #        #   
    //                 
    //                 
    //                 
    //                 
    //                 
    [ARROWS_HORIZONTAL] = { 
        0x80, 0xC0, 0xE0, 0xF0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xF0, 0xE0, 0xC0, 0x80,
        0x00, 0x01, 0x03, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x07, 0x03, 0x01, 0x00,
    },
};
