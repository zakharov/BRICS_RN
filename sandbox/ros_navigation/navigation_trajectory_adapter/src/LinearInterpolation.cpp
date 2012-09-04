/* 
 * File:   LinearInterpolation.cpp
 * Author: alexey
 * 
 * Created on September 3, 2012, 11:51 AM
 */

#include "navigation_trajectory_adapter/LinearInterpolation.h"
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/Logger.h"

LinearInterpolation::LinearInterpolation() {

}

LinearInterpolation::LinearInterpolation(const LinearInterpolation& orig) {

}


LinearInterpolation::~LinearInterpolation() {

}

void LinearInterpolation::interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step) {
    
    if (in.size() < 1)
        return;
    
    std::vector <FrameWithId> localCopy = in;
    
    double x0,y0,z0,xn,yn,zn,x,y,z,r,p;
    double s = step;
    
    x0 = localCopy[0].p.x();
    y0 = localCopy[0].p.y();
    localCopy[0].M.GetRPY(r , p, z0);
    
    xn = localCopy[1].p.x();
    yn = localCopy[1].p.y();
    localCopy[1].M.GetRPY(r, p, zn);
    
    LOG("path has %lu segments", in.size());
    
    LOG("%f, %f", in[in.size()-1].p.x(), in[in.size()-1].p.y());
    
    for (size_t i = 1; i < localCopy.size(); ++i) {
        //LOG("x0=%f y0=%f z0=%f xn=%f yn=%f zn=%f", x0, y0, z0, xn, yn, zn);
        double distance = sqrt((xn - x0)*(xn - x0) + (yn - y0)*(yn - y0));
        size_t numberOfSteps = static_cast <size_t> (floor(distance / step));
        
        LOG("numberOfSteps=%lu", numberOfSteps);
        for (size_t v = 0; v <= numberOfSteps; ++v) {
            double x = x0 + (xn - x0)/numberOfSteps*v;
            double y = y0 + (yn - y0)/numberOfSteps*v;
            double z = z0 + (zn - z0)/numberOfSteps*v;
            //LOG("step=%lu x=%f y=%f z=%f", v, x, y, z);
            FrameWithId frame;
            frame.id = localCopy[i-1].id;
            frame.p.x(x);
            frame.p.y(y);
            frame.p.z(z);
            frame.M.DoRotZ(z);
            out.push_back(frame);
        }
        
        x0 = localCopy[i].p.x();
        y0 = localCopy[i].p.y();
        localCopy[i].M.GetRPY(r , p, z0);
    
        xn = localCopy[i+1].p.x();
        yn = localCopy[i+1].p.y();
        localCopy[i+1].M.GetRPY(r, p, zn);
    }
        
     /*   
        if (distance > step) {
            s = s - distance;
            continue;
        } else
            s = step;

        x = s * (xn - x0) / sqrt((xn-x0)(xn-x0) + (yn-y0)(yn-y0)) - x0;
        y = y0 + x * (yn - y0) / (xn - x0);
    }
      * */
}
