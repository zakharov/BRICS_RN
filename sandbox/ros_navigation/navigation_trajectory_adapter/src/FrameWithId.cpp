/* 
 * File:   FrameWithId.cpp
 * Author: alexey
 * 
 * Created on August 29, 2012, 4:48 PM
 */

#include "navigation_trajectory_adapter/FrameWithId.h"

using namespace KDL;

FrameWithId::FrameWithId(const Rotation& R,const Vector& V, const std::string& id) : Frame(R,V), id(id) {
    
}

FrameWithId::FrameWithId(const Vector& V, const std::string& id) : Frame(V), id(id) {
    
}

//! The position matrix defaults to zero
FrameWithId::FrameWithId(const Rotation& R, const std::string& id) : Frame(R), id(id) {
    
}

FrameWithId::FrameWithId(const std::string& id) : Frame(), id(id) {

}

FrameWithId::FrameWithId() : Frame(), id("") {

}
    //! The copy constructor. Normal copy by value semantics.
FrameWithId::FrameWithId(const FrameWithId& arg) : Frame (arg) , id(arg.id) {
       
}

FrameWithId::~FrameWithId() {
    
}

