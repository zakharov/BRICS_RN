/* 
 * File:   PathIterator.cpp
 * Author: alexey
 * 
 * Created on September 3, 2012, 11:13 AM
 */

#include "navigation_trajectory_adapter/PathIterator.h"
#include "navigation_trajectory_adapter/FrameWithId.h"

const FrameWithId& PathIterator::dummy = FrameWithId();

PathIterator::PathIterator(const std::vector <FrameWithId>&) : path(path) {
    

}

PathIterator::PathIterator(const PathIterator& orig) : path(orig.path) {
    
}

PathIterator::~PathIterator() {
    
}

bool PathIterator::hasNext() const {
    
    return false;
}
    
const FrameWithId& PathIterator::next() const {
    
    return dummy;
}
