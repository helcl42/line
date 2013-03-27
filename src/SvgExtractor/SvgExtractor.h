/* 
 * File:   SvgExtractor.h
 * Author: lubos
 *
 * Created on March 26, 2013, 2:31 AM
 */

#ifndef SVGEXTRACTOR_H
#define	SVGEXTRACTOR_H

#include "../Utils/Utils.h"
#include "../XMLParser/XmlParser.h"

#include "../Shapes/DetectedObject.h"

#include "../SvgParser/SvgLexer.hpp"
#include "../SvgParser/SvgParser.hpp"

class SvgExtractor
{
private:
    XMLNode m_root;

    std::vector<DetectedObject*> m_objectsToDetect;

public:
    SvgExtractor(std::string filename);

    virtual ~SvgExtractor();

public:    
    std::vector<DetectedObject*> extractSvgObjects();

private:
    XMLNode getRootFromFile(std::string& filename);
    
    void extractEllipse(XMLNode node);

    void extractCircle(XMLNode node);
    
    void extractRectangle(XMLNode node);

    void extractPath(XMLNode node);
};

#endif	/* SVGEXTRACTOR_H */

