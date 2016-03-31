/**
 * @file     conv.c
 * @brief    itoa, utoa and itoh conversion routines
 * @version  V1.0
 * @date     23/01/2016
 *
 **/


#include <stdint.h>
#include "conv.h"


/***************************************************************************
 *                                                                         *
 *              Conversion routines                                        *
 *                                                                         *
 ***************************************************************************/

 /**
 * @brief itoa
 *
 * @note  Converts an signed integer to an decimal ASCII string with signal
 * @note  Assumes 32 bit integer
 *
 */

void
itoa(int v, char *s) {
int sign=0;
int p10 = 10;
int p10ant = 0;
char *p;
unsigned x;
int n;

    n = 1;
    x = v;
    if( v < 0 ) {
        x = -v;
        sign = 1;
        n++;
    }

    p = s;
    while( x >= p10 ) {
        p10ant = p10;
        p10 *= 10;
        if( p10 < p10ant ) // if overflow
            break;
        n++;
        *p++ = 'X';
    }
    p = s+n;
    *p--='\0';
    do {
        *p-- = ((x%10)+'0');
        x /= 10;
    } while( x > 0 );
    if( sign )
        *p = '-';
    return;
}

/**
 * @brief utoa
 *
 * @note  Converts an unsigned integer to an decimal ASCII string
 * @note  Assumes 32 bit integer
 *
 */

void
utoa(unsigned x, char *s) {
int sign=0;
int p10 = 10;
int p10ant = 0;
char *p;
int n;

    n = 1;
    p = s;
    while( x >= p10 ) {
        p10ant = p10;
        p10 *= 10;
        if( p10 < p10ant ) // if overflow
            break;
        n++;
        *p++ = 'X';
    }
    p = s+n;
    *p--='\0';
    do {
        *p-- = ((x%10)+'0');
        x /= 10;
    } while( x > 0 );
    return;
}

/**
 * @brief itohex
 *
 * @note  Converts an integer to an hexadecimal ASCII string
 * @note  Assumes 32 bit integer
 *
 */
//@{
static const char tohex[] = "0123456789ABCDEF";

int itohex(unsigned x, char *s) {
int n = sizeof(unsigned)*8;

    do {
        n -= 4;
        *s++ = tohex[(x>>n)&0xF];
    } while ( n > 0 );
    *s = '\0';

    return 0;
}

//@}
