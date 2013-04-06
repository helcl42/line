/* 
 * File:   Utils.h
 * Author: lubos
 *
 * Created on September 21, 2012, 1:15 PM
 */

#ifndef UTILS_H
#define	UTILS_H

#include <cmath>
#include <cstdlib>
#include <string.h>
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
    static int convertStringToHexInt(std::string str)
    {
        int value = 0;
        int base;
        char c;

        for (int i = str.length() - 1, j = 0; i >= 0; i--, j++)
        {
            base = pow(16, j);
            c = str[i];

            if (c >= '0' && c <= '9')
            {
                int temp = atoi(&c);
                value += base * temp;
            }
            else if ((c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'))
            {
                switch (c)
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

    static double logBy(double n, unsigned int base)
    {
        // log(n)/log(2) is log2.  
        return std::log(n) / std::log(base);
    }

    template <class T>
    static T normalize(T val)
    {
        if (val < 0.0f)
            val = 0.0f;
        if (val > 255)
            val = 255;
        return val;
    }

    template <class T>
    static T colorThreshold(T val, float threshold)
    {
        if (val < threshold)
            val = 0.0f;
        if (val > threshold)
            val = 255;
        return val;
    }
};

#endif	/* UTILS_H */

