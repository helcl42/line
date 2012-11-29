/* 
 * File:   Utils.h
 * Author: lubos
 *
 * Created on September 21, 2012, 1:15 PM
 */

#ifndef UTILS_H
#define	UTILS_H


#define SAFE_DELETE( p ) { if( p ) { delete ( p ); ( p ) = NULL; } }
#define SAFE_DELETE_ARRAY( p ) { if( p ) { delete[] ( p ); ( p ) = NULL; } }


#endif	/* UTILS_H */

