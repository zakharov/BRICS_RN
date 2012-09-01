/* 
 * File:   FrameWithId.h
 * Author: alexey
 *
 * Created on August 29, 2012, 4:48 PM
 */

#ifndef FRAMEWITHID_H
#define	FRAMEWITHID_H

#include <kdl/frames.hpp>
#include <string>

class FrameWithId : public KDL::Frame {
public:
    std::string id;

public:
    
    FrameWithId(const KDL::Rotation& R,const KDL::Vector& V, const std::string& id);
    
    FrameWithId(const KDL::Vector& V, const std::string& id);
    
    FrameWithId(const KDL::Rotation& R, const std::string& id);

    FrameWithId(const std::string& id);
    
    FrameWithId();
    
    FrameWithId(const FrameWithId& arg);

    virtual ~FrameWithId();
};


#endif	/* FRAMEWITHID_H */

