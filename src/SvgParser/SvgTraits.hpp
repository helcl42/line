/* 
 * File:   SvgTraits.h
 * Author: lubos
 *
 * Created on March 21, 2013, 3:58 AM
 */

#ifndef SVGTRAITS_H
#define	SVGTRAITS_H

#include    <antlr3.hpp>

class SvgLexer;
class SvgParser;

typedef antlr3::Traits<SvgLexer, SvgParser> SvgTraits;

typedef SvgTraits SvgLexerTraits;
typedef SvgTraits SvgParserTraits;

#endif	/* SVGTRAITS_H */


