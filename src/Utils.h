/* 
 * File:   Utils.h
 * Author: lubos
 *
 * Created on September 21, 2012, 1:15 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#include <cstdlib>
#include <stdexcept>

#define SAFE_DELETE( p ) { if( p ) { delete ( p ); ( p ) = NULL; } }
#define SAFE_DELETE_ARRAY( p ) { if( p ) { delete[] ( p ); ( p ) = NULL; } }

#define BYTES_TO_FLOAT_L(var, buffer, index)\
        {\
        *((unsigned char *)&var) = buffer[index];\
        *((unsigned char *)&var + 1) = buffer[index + 1];\
        *((unsigned char *)&var + 2) = buffer[index + 2];\
        *((unsigned char *)&var + 3) = buffer[index + 3];\
        }

#define BYTES_TO_FLOAT_B(var, buffer, index)\
        {\
        *((unsigned char *)&var + 3) = buffer[index];\
        *((unsigned char *)&var + 2) = buffer[index + 1];\
        *((unsigned char *)&var + 1) = buffer[index + 2];\
        *((unsigned char *)&var) = buffer[index + 3];\
        }

class Utils
{
public:
        
    static int convertStringToHexInt(const char* str, int len)
    {
        int value = 0;
        int base;        
        char c;

        for(int i = len - 1, j = 0; i >=0; i--, j++)
        {            
            base = pow(16, j);
            c = str[i];

            if(c >= '0' && c <= '9')
            {
                int temp = atoi(&c);                
                value += base * temp;
            }
            else if((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))
            {
                switch(c)
                {
                    case 'A':
                    case 'a':
                        value += base * 10;
                        break;
                    case 'B':
                    case 'b':
                        value += base * 11;
                        break;
                    case 'C':
                    case 'c':
                        value += base * 12;
                        break;
                    case 'D':
                    case 'd':
                        value += base * 13;
                        break;
                    case 'E':
                    case 'e':
                        value += base * 14;
                        break;
                    case 'F':
                    case 'f':
                        value += base * 15;
                        break;                   
                }
            }
        }
        return value;
    }
};

#endif	/* UTILS_H */

