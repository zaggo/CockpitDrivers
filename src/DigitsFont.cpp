#include "DigitsFont.h"

const uint8_t pixelMap [] PROGMEM = 
{
0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000011, 0b11110000, 0b00000000, //       ######      ,
  0b00000111, 0b11111100, 0b00000000, //      #########    ,
  0b00001111, 0b11111100, 0b00000000, //     ##########    ,
  0b00011111, 0b11111110, 0b00000000, //    ############   ,
  0b00111110, 0b00011111, 0b00000000, //   #####    #####  ,
  0b00111100, 0b00001111, 0b00000000, //   ####      ####  ,
  0b00111100, 0b00001111, 0b10000000, //   ####      ##### ,
  0b01111000, 0b00001111, 0b10000000, //  ####       ##### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111000, 0b00000111, 0b10000000, //  ####        #### ,
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### ,
  0b00111100, 0b00001111, 0b10000000, //   ####      ##### ,
  0b00111110, 0b00011111, 0b00000000, //   #####    #####  ,
  0b00011111, 0b11111111, 0b00000000, //    #############  ,
  0b00011111, 0b11111110, 0b00000000, //    ############   ,
  0b00001111, 0b11111100, 0b00000000, //     ##########    ,
  0b00000111, 0b11111000, 0b00000000, //      ########     ,
  0b00000000, 0b11000000, 0b00000000, //         ##        
  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000000, 0b11110000, 0b00000000, //         ####      ,
  0b00000011, 0b11110000, 0b00000000, //       ######      ,
  0b00001111, 0b11110000, 0b00000000, //     ########      ,
  0b00011111, 0b11110000, 0b00000000, //    #########      ,
  0b00111111, 0b11110000, 0b00000000, //   ##########      ,
  0b00011111, 0b11110000, 0b00000000, //    #########      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000000, 0b00000000, 0b00000000, //                   

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000111, 0b11111000, 0b00000000, //      ########     ,
  0b00001111, 0b11111100, 0b00000000, //     ##########    ,
  0b00011111, 0b11111110, 0b00000000, //    ############   ,
  0b00111111, 0b11111111, 0b00000000, //   ##############  ,
  0b01111100, 0b00011111, 0b00000000, //  #####     #####  ,
  0b00111000, 0b00001111, 0b00000000, //   ###       ####  ,
  0b00001000, 0b00001111, 0b10000000, //     #       ##### ,
  0b00000000, 0b00001111, 0b10000000, //             ##### ,
  0b00000000, 0b00001111, 0b00000000, //             ####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b01111100, 0b00000000, //          #####    ,
  0b00000000, 0b11111100, 0b00000000, //         ######    ,
  0b00000001, 0b11111000, 0b00000000, //        ######     ,
  0b00000011, 0b11110000, 0b00000000, //       ######      ,
  0b00000111, 0b11100000, 0b00000000, //      ######       ,
  0b00001111, 0b11000000, 0b00000000, //     ######        ,
  0b00001111, 0b10000000, 0b00000000, //     #####         ,
  0b00011111, 0b00000000, 0b00000000, //    #####          ,
  0b00111110, 0b00000000, 0b00000000, //   #####           ,
  0b00111111, 0b11111111, 0b10000000, //   ############### ,
  0b01111111, 0b11111111, 0b10000000, //  ################ ,
  0b01111111, 0b11111111, 0b10000000, //  ################ ,
  0b01111111, 0b11111111, 0b10000000, //  ################ ,
  0b00000000, 0b00000000, 0b00000000, //                   

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000111, 0b11110000, 0b00000000, //      #######      ,
  0b00011111, 0b11111100, 0b00000000, //    ###########    ,
  0b00111111, 0b11111110, 0b00000000, //   #############   ,
  0b00111111, 0b11111110, 0b00000000, //   #############   ,
  0b00011100, 0b00011111, 0b00000000, //    ###     #####  ,
  0b00001000, 0b00011111, 0b00000000, //     #      #####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000111, 0b11111100, 0b00000000, //      #########    ,
  0b00000111, 0b11111000, 0b00000000, //      ########     ,
  0b00000111, 0b11111100, 0b00000000, //      #########    ,
  0b00000111, 0b11111110, 0b00000000, //      ##########   ,
  0b00000000, 0b00111111, 0b00000000, //           ######  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00001111, 0b00000000, //             ####  ,
  0b00000000, 0b00001111, 0b10000000, //             ##### ,
  0b00000000, 0b00001111, 0b10000000, //             ##### ,
  0b00011000, 0b00001111, 0b00000000, //    ##       ####  ,
  0b00111000, 0b00011111, 0b00000000, //   ###      #####  ,
  0b00111110, 0b00111111, 0b00000000, //   #####   ######  ,
  0b01111111, 0b11111110, 0b00000000, //  ##############   ,
  0b00111111, 0b11111100, 0b00000000, //   ############    ,
  0b00011111, 0b11111000, 0b00000000, //    ##########     ,
  0b00000001, 0b10000000, 0b00000000, //        ##         

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000000, 0b00011110, 0b00000000, //            ####   ,
  0b00000000, 0b00011110, 0b00000000, //            ####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b01111110, 0b00000000, //          ######   ,
  0b00000000, 0b01111110, 0b00000000, //          ######   ,
  0b00000000, 0b11111110, 0b00000000, //         #######   ,
  0b00000001, 0b11111110, 0b00000000, //        ########   ,
  0b00000001, 0b11111110, 0b00000000, //        ########   ,
  0b00000011, 0b11111110, 0b00000000, //       #########   ,
  0b00000111, 0b10111110, 0b00000000, //      #### #####   ,
  0b00001111, 0b10111110, 0b00000000, //     ##### #####   ,
  0b00001111, 0b00111110, 0b00000000, //     ####  #####   ,
  0b00011110, 0b00111110, 0b00000000, //    ####   #####   ,
  0b00111110, 0b00111110, 0b00000000, //   #####   #####   ,
  0b00111100, 0b00111110, 0b00000000, //   ####    #####   ,
  0b01111111, 0b11111111, 0b11000000, //  #################,
  0b01111111, 0b11111111, 0b11000000, //  #################,
  0b01111111, 0b11111111, 0b11000000, //  #################,
  0b01111111, 0b11111111, 0b11000000, //  #################,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00000000, 0b00000000, //                   

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00011111, 0b11111111, 0b10000000, //    ############## ,
  0b00011111, 0b11111111, 0b10000000, //    ############## ,
  0b00011111, 0b11111111, 0b10000000, //    ############## ,
  0b00011111, 0b11111111, 0b10000000, //    ############## ,
  0b00011110, 0b00000000, 0b00000000, //    ####           ,
  0b00011110, 0b00000000, 0b00000000, //    ####           ,
  0b00011110, 0b00000000, 0b00000000, //    ####           ,
  0b00011110, 0b00000000, 0b00000000, //    ####           ,
  0b00011110, 0b11100000, 0b00000000, //    #### ###       ,
  0b00011111, 0b11111100, 0b00000000, //    ###########    ,
  0b00011111, 0b11111110, 0b00000000, //    ############   ,
  0b00111111, 0b11111111, 0b00000000, //   ##############  ,
  0b00111111, 0b00011111, 0b00000000, //   ######   #####  ,
  0b00011100, 0b00001111, 0b10000000, //    ###      ##### ,
  0b00000000, 0b00001111, 0b10000000, //             ##### ,
  0b00000000, 0b00000111, 0b10000000, //              #### ,
  0b00000000, 0b00000111, 0b10000000, //              #### ,
  0b00000000, 0b00000111, 0b10000000, //              #### ,
  0b00000000, 0b00000111, 0b10000000, //              #### ,
  0b00011000, 0b00001111, 0b10000000, //    ##       ##### ,
  0b00111100, 0b00001111, 0b10000000, //   ####      ##### ,
  0b00111111, 0b00111111, 0b00000000, //   ######  ######  ,
  0b00111111, 0b11111111, 0b00000000, //   ##############  ,
  0b00011111, 0b11111110, 0b00000000, //    ############   ,
  0b00001111, 0b11111000, 0b00000000, //     #########     ,
  0b00000001, 0b11000000, 0b00000000, //        ###        

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000001, 0b11111100, 0b00000000, //        #######    ,
  0b00000011, 0b11111110, 0b00000000, //       #########   ,
  0b00000111, 0b11111111, 0b00000000, //      ###########  ,
  0b00001111, 0b11111111, 0b00000000, //     ############  ,
  0b00011111, 0b00000110, 0b00000000, //    #####     ##   ,
  0b00011110, 0b00000100, 0b00000000, //    ####      #    ,
  0b00111110, 0b00000000, 0b00000000, //   #####           ,
  0b00111100, 0b00000000, 0b00000000, //   ####            ,
  0b00111100, 0b01000000, 0b00000000, //   ####   #        ,
  0b00111101, 0b11111000, 0b00000000, //   #### ######     ,
  0b01111111, 0b11111110, 0b00000000, //  ##############   ,
  0b01111111, 0b11111110, 0b00000000, //  ##############   ,
  0b01111111, 0b10111111, 0b00000000, //  ######## ######  ,
  0b01111110, 0b00011111, 0b10000000, //  ######    ###### ,
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### ,
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### ,
  0b01111100, 0b00000111, 0b10000000, //  #####       #### ,
  0b00111100, 0b00000111, 0b10000000, //   ####       #### ,
  0b00111100, 0b00001111, 0b10000000, //   ####      ##### ,
  0b00111110, 0b00001111, 0b10000000, //   #####     ##### ,
  0b00111110, 0b00011111, 0b00000000, //   #####    #####  ,
  0b00011111, 0b11111111, 0b00000000, //    #############  ,
  0b00001111, 0b11111110, 0b00000000, //     ###########   ,
  0b00000111, 0b11111100, 0b00000000, //      #########    ,
  0b00000011, 0b11111000, 0b00000000, //       #######     ,
  0b00000000, 0b01000000, 0b00000000, //          #        

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00111111, 0b11111111, 0b00000000, //   ##############  ,
  0b00111111, 0b11111111, 0b10000000, //   ############### ,
  0b00111111, 0b11111111, 0b10000000, //   ############### ,
  0b00111111, 0b11111111, 0b00000000, //   ##############  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00011110, 0b00000000, //            ####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b00111110, 0b00000000, //           #####   ,
  0b00000000, 0b01111100, 0b00000000, //          #####    ,
  0b00000000, 0b01111100, 0b00000000, //          #####    ,
  0b00000000, 0b01111000, 0b00000000, //          ####     ,
  0b00000000, 0b11111000, 0b00000000, //         #####     ,
  0b00000000, 0b11111000, 0b00000000, //         #####     ,
  0b00000000, 0b11110000, 0b00000000, //         ####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000001, 0b11110000, 0b00000000, //        #####      ,
  0b00000011, 0b11100000, 0b00000000, //       #####       ,
  0b00000011, 0b11100000, 0b00000000, //       #####       ,
  0b00000011, 0b11100000, 0b00000000, //       #####       ,
  0b00000111, 0b11000000, 0b00000000, //      #####        ,
  0b00000111, 0b11000000, 0b00000000, //      #####        ,
  0b00000111, 0b11000000, 0b00000000, //      #####        ,
  0b00001111, 0b10000000, 0b00000000, //     #####         ,
  0b00001111, 0b10000000, 0b00000000, //     #####         ,
  0b00001111, 0b10000000, 0b00000000, //     #####         ,
  0b00000000, 0b00000000, 0b00000000, //                   
  0b00000000, 0b00000000, 0b00000000, //                   
  0b00000011, 0b11111000, 0b00000000, //       #######     
  0b00001111, 0b11111100, 0b00000000, //     ##########    
  0b00011111, 0b11111110, 0b00000000, //    ############   
  0b00011111, 0b11111111, 0b00000000, //    #############  
  0b00111110, 0b00011111, 0b00000000, //   #####    #####  
  0b00111110, 0b00001111, 0b00000000, //   #####     ####  
  0b00111110, 0b00001111, 0b00000000, //   #####     ####  
  0b00111110, 0b00011111, 0b00000000, //   #####    #####  
  0b00111111, 0b00011111, 0b00000000, //   ######   #####  
  0b00011111, 0b10111110, 0b00000000, //    ###### #####   
  0b00001111, 0b11111100, 0b00000000, //     ##########    
  0b00000111, 0b11111000, 0b00000000, //      ########     
  0b00001111, 0b11111100, 0b00000000, //     ##########    
  0b00011111, 0b11111110, 0b00000000, //    ############   
  0b00111111, 0b00111111, 0b00000000, //   ######  ######  
  0b00111110, 0b00011111, 0b10000000, //   #####    ###### 
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### 
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### 
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### 
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### 
  0b01111110, 0b00001111, 0b10000000, //  ######     ##### 
  0b00111111, 0b10111111, 0b00000000, //   ####### ######  
  0b00111111, 0b11111111, 0b00000000, //   ##############  
  0b00011111, 0b11111110, 0b00000000, //    ############   
  0b00000111, 0b11111000, 0b00000000, //      ########     
  0b00000000, 0b11000000, 0b00000000, //         ##        

  0b00000000, 0b00000000, 0b00000000, //                   ,
  0b00000111, 0b11110000, 0b00000000, //      #######      ,
  0b00001111, 0b11111000, 0b00000000, //     #########     ,
  0b00011111, 0b11111100, 0b00000000, //    ###########    ,
  0b00111111, 0b11111110, 0b00000000, //   #############   ,
  0b00111110, 0b00111110, 0b00000000, //   #####   #####   ,
  0b01111100, 0b00011111, 0b00000000, //  #####     #####  ,
  0b01111000, 0b00011111, 0b00000000, //  ####      #####  ,
  0b01111000, 0b00001111, 0b00000000, //  ####       ####  ,
  0b01111000, 0b00001111, 0b00000000, //  ####       ####  ,
  0b01111000, 0b00001111, 0b10000000, //  ####       ##### ,
  0b01111100, 0b00001111, 0b10000000, //  #####      ##### ,
  0b01111100, 0b00011111, 0b10000000, //  #####     ###### ,
  0b00111110, 0b00111111, 0b10000000, //   #####   ####### ,
  0b00111111, 0b11111111, 0b10000000, //   ############### ,
  0b00011111, 0b11111111, 0b00000000, //    #############  ,
  0b00001111, 0b11111111, 0b00000000, //     ############  ,
  0b00000011, 0b11001111, 0b00000000, //       ####  ####  ,
  0b00000000, 0b00001111, 0b00000000, //             ####  ,
  0b00000000, 0b00011111, 0b00000000, //            #####  ,
  0b00000000, 0b00011110, 0b00000000, //            ####   ,
  0b00011000, 0b00111110, 0b00000000, //    ##     #####   ,
  0b00111100, 0b01111100, 0b00000000, //   ####   #####    ,
  0b00111111, 0b11111100, 0b00000000, //   ############    ,
  0b00111111, 0b11111000, 0b00000000, //   ###########     ,
  0b00001111, 0b11110000, 0b00000000, //     ########      ,
  0b00000001, 0b10000000, 0b00000000, //        ##         
};

Font digitsFont = {
  pixelMap,
  18, // Width
  27, // Height
};