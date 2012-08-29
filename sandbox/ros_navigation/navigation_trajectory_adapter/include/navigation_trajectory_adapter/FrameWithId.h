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

namespace KDL {

class FrameWithId : public Frame {
public:
    std::string id;

public:
    
    inline FrameWithId(const Rotation& R,const Vector& V, const std::string& id);

    //! The rotation matrix defaults to identity
    explicit inline FrameWithId(const Vector& V, const std::string& id);

    //! The position matrix defaults to zero
    explicit inline FrameWithId(const Rotation& R, const std::string& id);

    inline FrameWithId(const std::string& id);
    
    inline FrameWithId();
    
    //! The copy constructor. Normal copy by value semantics.
    inline FrameWithId(const FrameWithId& arg);

    virtual ~FrameWithId();
};

}

#endif	/* FRAMEWITHID_H */

