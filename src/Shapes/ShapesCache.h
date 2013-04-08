/* 
 * File:   ShapesCache.h
 * Author: lubos
 *
 * Created on April 9, 2013, 1:12 AM
 */

#ifndef SHAPESCACHE_H
#define	SHAPESCACHE_H

#include "DetectedObject.h"


class ShapesCache
{
private:
    std::map<unsigned int, std::vector<DetectedObject*> > m_objectsMap;
    
public:
    ShapesCache() {}
    
    ~ShapesCache() 
    {
        std::map<unsigned int, std::vector<DetectedObject*> >::iterator mi;
        for(mi = m_objectsMap.begin(); mi != m_objectsMap.end(); ++mi)        
        {
            std::vector<DetectedObject*>::iterator ii;
            for(ii = (*mi).second.begin(); ii != (*mi).second.end(); ++ii)
            {
                SAFE_DELETE(*ii);
            }
        }
        m_objectsMap.clear();
    }
    
    bool itemExists(unsigned int key)
    {
        return m_objectsMap.find(key) != m_objectsMap.end();
    }
    
    void addItem(unsigned int key, std::vector<DetectedObject*> objects)
    {
        m_objectsMap.insert(std::make_pair<unsigned int, std::vector<DetectedObject*> >(key, objects));
    }
    
    std::vector<DetectedObject*>& getItem(unsigned int key)
    {
        return m_objectsMap.find(key)->second;
    }
};

#endif	/* SHAPESCACHE_H */

