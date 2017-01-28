#include "display.h"

#include "spi.h"

#define NUMBER_OF_SYMBOLS (8)

// The codes below indicate which segments must be illuminated to display
// each number.
static char digitCodeMap[128] = {
  //     GFEDCBA  Segments      7-segment map:
  ['0'] = ~0x3F, // 0   "0"          AAA
  ['1'] = ~0x06, // 1   "1"         F   B
  ['2'] = ~0x5B, // 2   "2"         F   B
  ['3'] = ~0x4F, // 3   "3"          GGG
  ['4'] = ~0x66, // 4   "4"         E   C
  ['5'] = ~0x6D, // 5   "5"         E   C
  ['6'] = ~0x7D, // 6   "6"          DDD
  ['7'] = ~0x07, // 7   "7"
  ['8'] = ~0x7F, // 8   "8"
  ['9'] = ~0x6F, // 9   "9"
  ['A'] = ~0x77, // 65  'A'
  ['b'] = ~0x7C, // 66  'b'
  ['C'] = ~0x39, // 67  'C'
  ['d'] = ~0x5E, // 68  'd'
  ['E'] = ~0x79, // 69  'E'
  ['F'] = ~0x71, // 70  'F'
  ['G'] = ~0x3D, // 71  'G'
  ['H'] = ~0x76, // 72  'H'
  ['I'] = ~0x06, // 73  'I'
  ['J'] = ~0x0E, // 74  'J'
  ['K'] = ~0x76, // 75  'K'  Same as 'H'
  ['L'] = ~0x38, // 76  'L'
  ['M'] = ~0x00, // 77  'M'  NO DISPLAY
  ['n'] = ~0x54, // 78  'n'
  ['O'] = ~0x3F, // 79  'O'
  ['P'] = ~0x73, // 80  'P'
  ['q'] = ~0x67, // 81  'q'
  ['r'] = ~0x50, // 82  'r'
  ['S'] = ~0x6D, // 83  'S'
  ['t'] = ~0x78, // 84  't'
  ['U'] = ~0x3E, // 85  'U'
  ['V'] = ~0x3E, // 86  'V'  Same as 'U'
  ['W'] = ~0x00, // 87  'W'  NO DISPLAY
  ['X'] = ~0x76, // 88  'X'  Same as 'H'
  ['y'] = ~0x6E, // 89  'y'
  ['Z'] = ~0x5B, // 90  'Z'  Same as '2'
  [' '] = ~0x00, // 32  ' '  BLANK
  ['-'] = ~0x40  // 45  '-'  DASH
};


void ConvertDigitToSymbolsArray(int32_t num, uint8_t *dst) {
  // Last symbol in array is lowest in the digit
  int i = NUMBER_OF_SYMBOLS - 1;
  
  int8_t isNegative = (num < 0) ? 1 : 0;
  
  // Convert digit to symbol array
  while( (num) && (i >= 0) ) {
    dst[i] = digitCodeMap[ '0' + num % 10 ];
    num = num / 10;
    --i;
  }
  
  // Write symbol 'minus' if digit is negative
  if ( (isNegative) && (i >=0) ) {
    dst[i] = digitCodeMap['-'];
    --i;
  }
  
  // Blank all empty symbols places
  while(i >= 0) {
    dst[i] = digitCodeMap[' '];
    --i;
  }
  
  // If number more than we can show
  if ( (num) && (!isNegative) ) {
    // Very biggest digit, show all symbols as '9'
    for(i = 0; i < NUMBER_OF_SYMBOLS - 1; i++) {
      dst[i] = digitCodeMap['9'];
    }
  } else if ( (num) && (isNegative) ) {
      // ERROR we cant show this negative digit because it's very biggest
      for(i = 0; i < NUMBER_OF_SYMBOLS - 1; i++) {
        dst[i] = digitCodeMap['-'];
      }
  }
}


/*
 *@brief Show time on the display
 */
void Display_ShowTime(int32_t seconds) {
  uint8_t dstArray[NUMBER_OF_SYMBOLS] = {0};
  ConvertDigitToSymbolsArray(seconds, &dstArray[0]);
  SendDataTo7SegDisplay(&dstArray[0], NUMBER_OF_SYMBOLS);
}