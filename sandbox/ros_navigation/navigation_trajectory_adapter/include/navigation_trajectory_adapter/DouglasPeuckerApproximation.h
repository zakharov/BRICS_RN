/* 
 * File:   DouglasPeuckerApproximation.h
 * Author: alexey
 *
 * Created on August 31, 2012, 2:55 PM
 */

#ifndef DOUGLASPEUCKERAPPROXIMATION_H
#define	DOUGLASPEUCKERAPPROXIMATION_H

#include "navigation_trajectory_adapter/PathApproximation.h"
#include <vector>

class FrameWithId;

class DouglasPeuckerApproximation : public PathApproximation {
public:
    DouglasPeuckerApproximation();
    DouglasPeuckerApproximation(const DouglasPeuckerApproximation& orig);
    virtual ~DouglasPeuckerApproximation();
    
    void approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out);
    void approximate (const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double epsilon);
    
    
private:

    void douglasPeucker(const std::vector <FrameWithId>& pointList, 
        std::vector <FrameWithId>& resultList, int& counter, double epsilon);
    
    
};

#endif	/* DOUGLASPEUCKERAPPROXIMATION_H */

