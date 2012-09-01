/* 
 * File:   ChaikinCurveApproximation.h
 * Author: alexey
 *
 * Created on August 31, 2012, 3:23 PM
 */

#ifndef CHAIKINCURVEAPPROXIMATION_H
#define	CHAIKINCURVEAPPROXIMATION_H

#include "navigation_trajectory_adapter/PathApproximation.h"

class ChaikinCurveApproximation : public PathApproximation {
public:
    ChaikinCurveApproximation();
    ChaikinCurveApproximation(const ChaikinCurveApproximation& orig);
    virtual ~ChaikinCurveApproximation();
    
    void approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out);
    void approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, unsigned int lod);
    
private:

    unsigned int chaikinCurve (const std::vector <FrameWithId>& pointList, 
        std::vector <FrameWithId>& resultList);
    
   
    
};

#endif	/* CHAIKINCURVEAPPROXIMATION_H */

