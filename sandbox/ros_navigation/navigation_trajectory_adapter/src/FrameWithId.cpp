/* 
 * File:   FrameWithId.cpp
 * Author: alexey
 * 
 * Created on August 29, 2012, 4:48 PM
 */
#include <kdl/frames.hpp>
#include "navigation_trajectory_adapter/FrameWithId.h"

namespace KDL {

inline FrameWithId::FrameWithId(const Rotation& R,const Vector& V, const std::string& id) : Frame(R,V), id(id) {
    
}

inline FrameWithId::FrameWithId(const Vector& V, const std::string& id) : Frame(V), id(id) {
    
}

//! The position matrix defaults to zero
inline FrameWithId::FrameWithId(const Rotation& R, const std::string& id) : Frame(R), id(id) {
    
}

inline FrameWithId::FrameWithId(const std::string& id) : Frame(), id(id) {

}

inline FrameWithId::FrameWithId() : Frame(), id("") {

}
    //! The copy constructor. Normal copy by value semantics.
inline FrameWithId::FrameWithId(const FrameWithId& arg) : Frame (arg) , id(arg.id) {
       
}

FrameWithId::~FrameWithId() {
    
}

}