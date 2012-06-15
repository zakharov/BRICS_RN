/* 
 * File:   VelocityProfile_Trap.cpp
 * Author: alexey
 * 
 * Created on June 6, 2012, 10:16 PM
 */

#include "VelocityProfile_Line.h"

#include <iostream>

namespace KDL {

    VelocityProfile_Line::VelocityProfile_Line(double _maxvel, double _maxacc) :
    a1(0), a2(0), a3(0),
    b1(0), b2(0), b3(0),
    c1(0), c2(0), c3(0),
    duration(0), t1(0), t2(0),
    maxvel(_maxvel), maxacc(_maxacc),
    startpos(0), endpos(0)
 {
    }
    // constructs motion profile class with <maxvel> as parameter of the
    // trajectory.

    void VelocityProfile_Line::SetProfile(double pos1, double vel1, double pos2, double vel2) {
        startpos = pos1;
        endpos = pos2;
        double s = sign(endpos - startpos);
        
        maxacc = (vel2*vel2 - vel1*vel1) /  (2*(pos2-pos1));
        t1 = (vel2 - vel1) / maxacc;
        t1 =t1;
        duration = t1;
        t2 = t1;
        
        a3 = s * maxacc / 2.0;
        a2 = vel1;
        a1 = startpos;

        b3 = 0;
        b2 = a2 + 2 * a3 * t1 - 2.0 * b3*t1;
        b1 = a1 + t1 * (a2 + a3 * t1) - t1 * (b2 + t1 * b3);

        c3 = -s * maxacc / 2.0;
        c2 = b2 + 2 * b3 * t2 - 2.0 * (c3 * t2);
        c1 = b1 + t2 * (b2 + b3 * t2) - t2 * (c2 + t2 * c3);
    }

    void VelocityProfile_Line::SetProfile(double pos1, double pos2) {
        SetProfile(pos1, 0, pos2, 0);
    }

    void VelocityProfile_Line::SetProfileDuration(
            double pos1, double pos2, double newduration) {
        // duration should be longer than originally planned duration
        // Fastest :
        SetProfile(pos1, pos2);
        // Must be Slower  :
        double factor = duration / newduration;
        if (factor > 1)
            return; // do not exceed max
        a2 *= factor;
        a3 *= factor*factor;
        b2 *= factor;
        b3 *= factor*factor;
        c2 *= factor;
        c3 *= factor*factor;
        duration = newduration;
        t1 /= factor;
        t2 /= factor;
    }

    void VelocityProfile_Line::SetMax(double _maxvel, double _maxacc) {
        maxvel = _maxvel;
        maxacc = _maxacc;
    }

    double VelocityProfile_Line::Duration() const {
        return duration;
    }

    double VelocityProfile_Line::Pos(double time) const {
        if (time < 0) {
            return startpos;
        } else if (time < t1) {
           
            return a1 + time * (a2 + a3 * time);
        } else if (time < t2) {
          
            return b1 + time * (b2 + b3 * time);
        } else if (time <= duration) {
          
            return c1 + time * (c2 + c3 * time);
        } else {
            return endpos;
        }
    }

    double VelocityProfile_Line::Vel(double time) const {
        if (time < 0) {
            return 0;
        } else if (time < t1) {
            return a2 + 2 * a3*time;
        } else if (time < t2) {
            return b2 + 2 * b3*time;
        } else if (time <= duration) {
            return c2 + 2 * c3*time;
        } else {
            return 0;
        }
    }

    double VelocityProfile_Line::Acc(double time) const {
        if (time < 0) {
            return 0;
        } else if (time < t1) {
            return 2 * a3;
        } else if (time < t2) {
            return 2 * b3;
        } else if (time <= duration) {
            return 2 * c3;
        } else {
            return 0;
        }
    }

    VelocityProfile* VelocityProfile_Line::Clone() const {
        VelocityProfile_Line* res = new VelocityProfile_Line(maxvel, maxacc);
        res->SetProfileDuration(this->startpos, this->endpos, this->duration);
        return res;
    }

    VelocityProfile_Line::~VelocityProfile_Line() {
    }

    void VelocityProfile_Line::Write(std::ostream& os) const {
        os << "LINEAR[" << maxvel << "," << maxacc << "]";
    }





}


