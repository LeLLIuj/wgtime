#ifndef __hal_Types_H_
#define __hal_Types_H_


/* ------------------------------------------------------------------------------------------------
 *                                               Types
 * ------------------------------------------------------------------------------------------------
 */
typedef signed char     int8;
typedef unsigned char   uint8;

typedef int             int16;
typedef unsigned int    uint16;

typedef long            int32;
typedef unsigned long   uint32;

typedef float           float32;

  
/* ------------------------------------------------------------------------------------------------
 *                                        Standard Defines
 * ------------------------------------------------------------------------------------------------
 */

#ifndef false
  #define false 0
#endif

#ifndef true
  #define true 1
#endif

#ifndef FALSE
  #define  FALSE  0U                   /* Boolean value FALSE. FALSE is defined always as a zero value. */
#endif
#ifndef TRUE
  #define  TRUE   1U                   /* Boolean value TRUE. TRUE is defined always as a non zero value. */
#endif

#ifndef NULL
  #define  NULL   0U
#endif

  
#endif  //__hal_Types_H_
