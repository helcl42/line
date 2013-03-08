#include "DetectionSettings.h"

//format 334432 657687 657654 ...
DetectionSettings::DetectionSettings(int argc, char** argv)
: colorIndex(0)
{                
    for (int i = 1; i < argc; i++)
    {
        DetectionColorItem* item = new DetectionColorItem();

        std::string color(argv[i]);
        if (color.length() == 6)
        {
            for (int j = 0, index = 0; j < 3; j++, index += 2)
            {                     
                item->color[j] = Utils::convertStringToHexInt(color.substr(index, 2));                
            }
        }
        else
        {
            throw std::runtime_error("Invalid color format");
        }
        colors.push_back(item);
    }
}

DetectionSettings::~DetectionSettings()
{
    for (unsigned int i = 0; i < colors.size(); i++)
    {
        SAFE_DELETE(colors[i]);
    }
}

unsigned int DetectionSettings::getCountOfColors() const
{
    return colors.size();
}

DetectionColorItem* DetectionSettings::getNext()
{
    colorIndex++;
    
    if(colorIndex >= colors.size()) 
    {
        colorIndex = 0;        
    }
    return colors[colorIndex];
}


DetectionColorItem* DetectionSettings::getItem(int index) const
{
    if (index >= 0 || index < (int)colors.size())
    {
        return colors[index];
    }
    else
    {
        throw std::runtime_error("DetectionSettings:getItem:Invalid index");
    }
}

DetectionColorItem* DetectionSettings::operator[](int index)
{
    if (index >= 0 || index < (int)colors.size())
    {
        return colors[index];
    }
    else
    {
        throw std::runtime_error("DetectionSettings:operator[]:Invalid index");
    }
}

std::ostream& operator<<(std::ostream& out, const DetectionSettings& settings)
{
    out << "Detection settings:" << std::endl;
    for (unsigned int i = 0; i < settings.colors.size(); i++)
    {
        out << "-------------------------------" << std::endl;
        out << i + 1 << ") color" << std::endl;
        out << settings.colors[i]->color << std::endl;
    }
    return out;
}

