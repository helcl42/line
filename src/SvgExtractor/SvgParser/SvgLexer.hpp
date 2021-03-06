/** \file
 *  This C++ header file was generated by $ANTLR version 3.4
 *
 *     -  From the grammar source file : Svg.g
 *     -                            On : 2013-03-23 01:23:54
 *     -                 for the lexer : SvgLexerLexer
 *
 * Editing it, at least manually, is not wise.
 *
 * C++ language generator and runtime by Gokulakannan Somasundaram ( heavy lifting from C Run-time by Jim Idle )
 *
 *
 * The lexer 
SvgLexer

has the callable functions (rules) shown below,
 * which will invoke the code for the associated rule in the source grammar
 * assuming that the input stream is pointing to a token/text stream that could begin
 * this rule.
 *
 * For instance if you call the first (topmost) rule in a parser grammar, you will
 * get the results of a full parse, but calling a rule half way through the grammar will
 * allow you to pass part of a full token stream to the parser, such as for syntax checking
 * in editors and so on.
 *
 */
// [The "BSD license"]
// Copyright (c) 2005-2009 Gokulakannan Somasundaram. 
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. The name of the author may not be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef	_SvgLexer_H
#define _SvgLexer_H
/* =============================================================================
 * Standard antlr3 C++ runtime definitions
 */
#include    <antlr3.hpp>
#include    <stdexcept>
#include    <iostream>
#include    <queue>

/* End of standard antlr 3 runtime definitions
 * =============================================================================
 */


#include "SvgTraits.hpp"




#ifdef	WIN32
// Disable: Unreferenced parameter,							- Rules with parameters that are not used
//          constant conditional,							- ANTLR realizes that a prediction is always true (synpred usually)
//          initialized but unused variable					- tree rewrite variables declared but not needed
//          Unreferenced local variable						- lexer rule declares but does not always use _type
//          potentially unitialized variable used			- retval always returned from a rule
//			unreferenced local function has been removed	- susually getTokenNames or freeScope, they can go without warnigns
//
// These are only really displayed at warning level /W4 but that is the code ideal I am aiming at
// and the codegen must generate some of these warnings by necessity, apart from 4100, which is
// usually generated when a parser rule is given a parameter that it does not use. Mostly though
// this is a matter of orthogonality hence I disable that one.
//
#pragma warning( disable : 4100 )
#pragma warning( disable : 4101 )
#pragma warning( disable : 4127 )
#pragma warning( disable : 4189 )
#pragma warning( disable : 4505 )
#pragma warning( disable : 4701 )
#endif

typedef SvgLexerTraits SvgLexerImplTraits;

class SvgLexerTokens
{
public:

    /** Symbolic definitions of all the tokens that the 
lexer
 will work with.
     * \{
     *
     * Antlr will define EOF, but we can't use that as it it is too common in
     * in C header files and that would be confusing. There is no way to filter this out at the moment
     * so we just undef it here for now. That isn't the value we get back from C recognizers
     * anyway. We are looking for ANTLR_TOKEN_EOF.
     */
    enum Tokens
    {
        EOF_TOKEN = SvgLexerImplTraits::CommonTokenType::TOKEN_EOF
        , T__14 = 14
        , T__15 = 15
        , T__16 = 16
        , T__17 = 17
        , T__18 = 18
        , T__19 = 19
        , T__20 = 20
        , T__21 = 21
        , T__22 = 22
        , T__23 = 23
        , T__24 = 24
        , T__25 = 25
        , T__26 = 26
        , T__27 = 27
        , T__28 = 28
        , T__29 = 29
        , T__30 = 30
        , T__31 = 31
        , T__32 = 32
        , T__33 = 33
        , T__34 = 34
        , T__35 = 35
        , T__36 = 36
        , T__37 = 37
        , T__38 = 38
        , T__39 = 39
        , COMMA_WSP = 4
        , DIGITS = 5
        , EXPONENT = 6
        , FLAG = 7
        , FLOAT = 8
        , HEX_DIGITS = 9
        , INT = 10
        , NUMBER = 11
        , OCTAL_DIGITS = 12
        , WS = 13
    };

};

/** Context tracking structure for 
SvgLexer

 */
class SvgLexer : public
SvgLexerImplTraits::BaseLexerType
, public SvgLexerTokens
{
public:
    typedef SvgLexerImplTraits ImplTraits;
    typedef SvgLexer ComponentType;
    typedef ComponentType::StreamType StreamType;
    typedef
    SvgLexerImplTraits::BaseLexerType
    BaseType;
    typedef ImplTraits::RecognizerSharedStateType<StreamType> RecognizerSharedStateType;
    typedef StreamType InputType;
    static const bool IsFiltered = false;

private:
    std::string m_identifierString;
    std::queue<std::string> m_identifiersQueue;

public:
    SvgLexer(InputType* instream);
    SvgLexer(InputType* instream, RecognizerSharedStateType* state);

    void init(InputType* instream);

    void insertIntoLiteralsQueue(std::string literal);

    std::string getIdentifierString();

    std::string getIdentifierStringWithoutPop();

    double convertStringToDouble(std::string& number);

    int convertStringToInteger(std::string& number);


    void
    mT__14();

    void
    mT__15();

    void
    mT__16();

    void
    mT__17();

    void
    mT__18();

    void
    mT__19();

    void
    mT__20();

    void
    mT__21();

    void
    mT__22();

    void
    mT__23();

    void
    mT__24();

    void
    mT__25();

    void
    mT__26();

    void
    mT__27();

    void
    mT__28();

    void
    mT__29();

    void
    mT__30();

    void
    mT__31();

    void
    mT__32();

    void
    mT__33();

    void
    mT__34();

    void
    mT__35();

    void
    mT__36();

    void
    mT__37();

    void
    mT__38();

    void
    mT__39();

    void
    mCOMMA_WSP();

    void
    mNUMBER();

    void
    mINT(std::string& number);

    void
    mDIGITS(std::string& number);

    void
    mOCTAL_DIGITS(std::string& number);

    void
    mHEX_DIGITS(std::string& number);

    void
    mFLOAT(std::string& number);

    void
    mFLAG();

    void
    mWS();

    void
    mEXPONENT(std::string& number);

    void
    mTokens();
    const char * getGrammarFileName();
    void reset();
    ~SvgLexer();

};

// Function protoypes for the constructor functions that external translation units
// such as delegators and delegates may wish to call.
//

/* End of token definitions for SvgLexer
 * =============================================================================
 */
/** } */


#endif

/* END - Note:Keep extra line feed to satisfy UNIX systems */