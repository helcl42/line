#include "SvgExtractor.h"
#include "../Shapes/GeneralObject.h"
#include "../Shapes/Rectangle.h"
#include "../Shapes/Circle.h"
#include "../Shapes/Ellipse.h"

SvgExtractor::SvgExtractor(std::string filename)
{
    m_root = getRootFromFile(filename);
}

SvgExtractor::~SvgExtractor()
{
    std::vector<DetectedObject*>::iterator ii;
    for (ii = m_objectsToDetect.begin(); ii != m_objectsToDetect.end(); ++ii)
    {
        SAFE_DELETE(*ii);
    }
}

XMLNode SvgExtractor::getRootFromFile(std::string& filename)
{
    XMLNode root = XMLNode::openFileHelper(filename.c_str(), "svg");
    XMLNode gNode = root.getChildNode("g");
    if (!gNode.isEmpty())
    {
        root = gNode;
    }
    return root;
}

std::vector<DetectedObject*> SvgExtractor::extractSvgObjects()
{
    std::vector<XMLNode> children;
    for (unsigned int index = 0; index < m_root.nChildNode(); index++)
    {
        XMLNode child = m_root.getChildNode(index);
        children.push_back(child);
    }

    for (unsigned int i = 0; i < children.size(); i++)
    {
        std::string nodeName(children[i].getName());

        if (nodeName == "path")
        {
            extractPath(children[i]);
        }
        else if (nodeName == "circle")
        {
            extractCircle(children[i]);
        }
        else if (nodeName == "rect")
        {
            extractRectangle(children[i]);
        }
        else if (nodeName == "ellipse")
        {
            extractEllipse(children[i]);
        }
    }
    return m_objectsToDetect;
}

void SvgExtractor::extractEllipse(XMLNode node)
{
    float width = std::atof(node.getAttribute("rx"));
    float height = std::atof(node.getAttribute("ry"));
    Ellipse* ellipse = new Ellipse(width, height);
    ellipse->generate();
    ellipse->translateToOrigin();
    m_objectsToDetect.push_back(ellipse);
}

void SvgExtractor::extractCircle(XMLNode node)
{
    float radius = std::atof(node.getAttribute("r"));

    Circle* circ = new Circle(radius);
    circ->generate();    
    circ->translateToOrigin();
    m_objectsToDetect.push_back(circ);
}

//<rect x="50" y="20" width="150" height="150">
void SvgExtractor::extractRectangle(XMLNode node)
{
    float width = std::atof(node.getAttribute("width"));
    float height = std::atof(node.getAttribute("height"));

    Rectangle* rect = new Rectangle(width, height);
    rect->generate();    
    rect->translateToOrigin();
    m_objectsToDetect.push_back(rect);
}

void SvgExtractor::extractPath(XMLNode node)
{
    std::string path(node.getAttribute("d"));

    SvgLexer::InputType pathInputStream((ANTLR_UINT8*) path.c_str(), ANTLR_ENC_8BIT, path.size(), (ANTLR_UINT8*) "path");
    SvgLexer pathLexer(&pathInputStream);

    SvgParser::StreamType tokenStream(ANTLR_SIZE_HINT, pathLexer.get_tokSource());
    SvgParser pathParser(&tokenStream, pathLexer);

    Line* line = new Line();
    Node* root = pathParser.svgPath();
    if (root != NULL)
    {
        root->draw(line);
    }
    else
    {
        throw std::runtime_error("No data parsed");
    }

    GeneralObject* object = new GeneralObject(line);
    object->translateToOrigin();
    //object->setColor(200, 399, 399);    
    m_objectsToDetect.push_back(object);
    SAFE_DELETE(root);
}
