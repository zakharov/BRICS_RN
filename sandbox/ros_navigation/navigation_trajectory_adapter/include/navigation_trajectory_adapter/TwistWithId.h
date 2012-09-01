/* 
 * File:   TwistWithId.h
 * Author: alexey
 *
 * Created on August 30, 2012, 1:24 PM
 */

#ifndef TWISTWITHID_H
#define	TWISTWITHID_H

#include <kdl/frames.hpp>
#include <string>

class TwistWithId : public KDL::Twist {
public:
    std::string id;
    
public:
    TwistWithId();
    TwistWithId(const std::string& id);
    TwistWithId(const KDL::Vector& _vel,const KDL::Vector& _rot, const std::string& id);
    TwistWithId(const TwistWithId& orig);
    virtual ~TwistWithId();

};

#endif	/* TWISTWITHID_H */

