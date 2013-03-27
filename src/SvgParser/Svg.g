grammar Svg;

options { language=Cpp; backtrack=true; memoize=true;}

@lexer::includes
{
	#include "SvgTraits.hpp"
}

@parser::includes
{
	#include "SvgLexer.hpp"
}

COMMA_WSP: 
    (WS+ | WS* ',' WS*)
    ;

NUMBER: 
    '-'? (INT | FLOAT);

fragment
INT: 
	DIGITS 	
	;

fragment 
DIGITS: 
    '0'..'9'+;

fragment 
OCTAL_DIGITS: 
    '0' '0'..'7'+;

fragment 
HEX_DIGITS: 
    '0x' ('0'..'9' | 'a'..'f' | 'A'..'F')+;

fragment
FLOAT: 
    ('0'..'9')+ '.' ('0'..'9')* EXPONENT?
    | '.' ('0'..'9')+ EXPONENT?
    | ('0'..'9')+ EXPONENT
    ;

fragment
FLAG: 
    ('0' | '1')
    ;

fragment
WS : 
    (' ' 
    | '\t' 
    | '\r' 
    | '\n')
    ;

wsp:
    (' ' | '\n' | '\r' | '\t')?;    

flag: 
    ('0' | '1')
    ;

fragment
EXPONENT: 
    ('e'|'E') ('+'|'-')? ('0'..'9')+ 
    ;

coordinate: 
    wsp NUMBER wsp
    ;

coordinatePair: 
    coordinate COMMA_WSP? coordinate
    ;

lineto
    : ( 'L' | 'l' ) wsp coordinatePair+
    ;

moveto: 
    ( 'M' | 'm' ) wsp coordinatePair+
    ;

closepath:
    ('Z' | 'z')
    ;

curvetoArgument: 
    coordinatePair COMMA_WSP? coordinatePair COMMA_WSP? coordinatePair  
    ;

curvetoArgumentSequence: 
    curvetoArgument (COMMA_WSP? curvetoArgument)*
    ;

curveto: 
    ( 'C' | 'c' ) wsp curvetoArgumentSequence
    ;

quadraticBezierCurvetoArgument:
    coordinatePair COMMA_WSP? coordinatePair    
    ;

quadraticBezierCurvetoArgumentSequence: 
    quadraticBezierCurvetoArgument (COMMA_WSP? quadraticBezierCurvetoArgument)*       
    ;

quadraticBezierCurveto: 
    ( 'Q' | 'q' ) wsp quadraticBezierCurvetoArgumentSequence
    ;

horizontalLinetoArgumentSequence: 
    coordinate (COMMA_WSP? coordinate)*    
    ;

horizontalLineto: 
    ( 'H' | 'h' ) wsp horizontalLinetoArgumentSequence
    ;

verticalLinetoArgumentSequence: 
    coordinate (COMMA_WSP? coordinate)*    
    ;

verticalLineto: 
    ( 'V' | 'v' ) wsp verticalLinetoArgumentSequence
    ;

smoothCurvetoArgument: 
    coordinatePair COMMA_WSP? coordinatePair
    ;

smoothCurvetoArgumentSequence: 
    smoothCurvetoArgument (COMMA_WSP? smoothCurvetoArgument)*    
    ;

smoothCurveto: 
    ( 'S' | 's' ) wsp smoothCurvetoArgumentSequence
    ;

smoothQuadraticBezierCurvetoArgumentSequence: 
    coordinatePair (COMMA_WSP? coordinatePair)*
    ;

smoothQuadraticBezierCurveto: 
    ( 'T' | 't' ) wsp smoothQuadraticBezierCurvetoArgumentSequence
    ;

ellipticalArcArgument: 
    coordinatePair wsp flag wsp flag wsp flag wsp coordinatePair
    ;

ellipticalArcArgumentSequence: 
    ellipticalArcArgument (COMMA_WSP? ellipticalArcArgument)*    
    ;

ellipticalArc: 
    ( 'A' | 'a' ) wsp ellipticalArcArgumentSequence
    ;

drawtoCommand: 
    closepath
    | lineto
    | curveto
    | quadraticBezierCurveto
    | horizontalLineto
    | verticalLineto
    | smoothCurveto            
    | smoothQuadraticBezierCurveto    
    | ellipticalArc
    ;

drawtoCommands: 
    drawtoCommand (drawtoCommand)*
    ;

movetoDrawtoCommandGroup: 
    moveto wsp drawtoCommands?
    ;

movetoDrawtoCommandGroups: 
    movetoDrawtoCommandGroup (wsp movetoDrawtoCommandGroup)*    
    ;

svgPath: 
    wsp movetoDrawtoCommandGroups? wsp
    ;