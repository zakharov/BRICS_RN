/* 
 * File:   LinearInterpolation.cpp
 * Author: alexey
 * 
 * Created on September 3, 2012, 11:51 AM
 */

#include "navigation_trajectory_adapter/LinearInterpolation.h"
#include "navigation_trajectory_adapter/FrameWithId.h"
#include "navigation_trajectory_adapter/Logger.h"
#include "navigation_trajectory_adapter/Stopwatch.h

LinearInterpolation::LinearInterpolation() {

}

LinearInterpolation::LinearInterpolation(const LinearInterpolation& orig) {

}

LinearInterpolation::~LinearInterpolation() {

}

size_t LinearInterpolation::interpolate(const std::vector <FrameWithId>& in, std::vector <FrameWithId>& out, double step, size_t numberOfsteps = 0) {
#ifdef DEBUG
    Stopwatch stopwatch;
    stopwatch.start();
#endif
    size_t result = linearInterpolation(in, out, step, numberOfsteps);
#ifdef DEBUG  
    stopwatch.stop();
    LOG("Linear interpolation:");
    LOG("  - input end point: %f, %f", in.back().p.x(), in.back().p.y());
    LOG("  - output end point: %f, %f", out.back().p.x(), out.back().p.y());
    LOG("  - step size: %f", step);
    LOG("  - iterations : %lu", result);
    LOG("  - duration : %f ms", stopwatch.getElapsedTime());
#endif  
    return result;
}

size_t LinearInterpolation::linearInterpolation(const std::vector <FrameWithId>& in,
        std::vector <FrameWithId>& out,
        double step,
        size_t numberOfSteps = 0) {

    if (in.size() < 1)
        return 0;

   
    
    size_t counter = 0;
    std::vector <FrameWithId> localCopy = in;
    double x0, y0, z0, xn, yn, zn, x, y, z, r, p;

    
    for (size_t i = 0; i < localCopy.size()-1; ++i) {
        x0 = localCopy[i].p.x();
        y0 = localCopy[i].p.y();
        localCopy[i].M.GetRPY(r, p, z0);

        xn = localCopy[i + 1].p.x();
        yn = localCopy[i + 1].p.y();
        localCopy[i + 1].M.GetRPY(r, p, zn);

        double distance = sqrt((xn - x0)*(xn - x0) + (yn - y0)*(yn - y0));
        size_t steps = static_cast<size_t> (floor(distance / step));
      //  LOG("x0=%f, y0=%f, xn=%f, yn=%f", x0, y0, xn, yn);
      //  LOG("distance=%f, steps=%lu", distance, steps);
        
        for (size_t v = 0; v <= steps; ++v) {
            x = x0 + (xn - x0) / steps*v;
            y = y0 + (yn - y0) / steps*v;
            z = z0 + (zn - z0) / steps*v;
            
      //      LOG("x=%f, y=%f", x, y);
            
            FrameWithId frame;
            frame.id = localCopy[i].id;
            frame.p.x(x);
            frame.p.y(y);
            frame.p.z(z);
            frame.M.DoRotZ(z);
            out.push_back(frame);
            ++counter;
            if (counter >= (numberOfSteps - 1) && numberOfSteps != 0) {
                return counter;
            }
        }

        
    }

    
    
    return counter;

}