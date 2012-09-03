/* 
 * File:   LinearInterpolation.cpp
 * Author: alexey
 * 
 * Created on September 3, 2012, 11:51 AM
 */

#include "navigation_trajectory_adapter/LinearInterpolation.h"

LinearInterpolation::LinearInterpolation() {

}

LinearInterpolation::LinearInterpolation(const LinearInterpolation& orig) {

}


LinearInterpolation::~LinearInterpolation() {

}

void LinearInterpolation::interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step) {
    
    if (in.size() < 1)
        return;
    
    double x0,y0,xn,yn,x,y;
    
    for (size_t i = 1; i < in.size(); i++) {
        
    }
    
}
