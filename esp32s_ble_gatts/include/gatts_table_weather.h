#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*   Attribute State Machine   */
enum{
    IDX_SVC,
    
    IDX_CHAR_A,
    CHAR_A_VAL,
    IDX_CHAR_CFG_A,

    IDX_CHAR_B,
    CHAR_B_VAL,
    IDX_CHAR_CFG_B,

    WEATHER_IDX_NB
} ;