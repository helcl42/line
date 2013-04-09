/* 
 * File:   CurveToCommand.h
 * Author: lubos
 *
 * Created on March 21, 2013, 10:01 PM
 */

#ifndef CURVETOCOMMAND_H
#define	CURVETOCOMMAND_H

#include "Command.h"

class CurveToCommand : public Command
{
public:
    CurveToCommand() {}

    virtual ~CurveToCommand() {}

public:
    void draw(Line<int>* polygon);

    void print();

private:    
    std::vector<Vector2<float> > getBezierApproximation(std::vector<Vector2<float> >& line, int outputSegmentCount);

    Vector2<float> getBezierPoint(double t, std::vector<Vector2<float> >& line, int index, int count);    
};

#endif	/* CURVETOCOMMAND_H */

