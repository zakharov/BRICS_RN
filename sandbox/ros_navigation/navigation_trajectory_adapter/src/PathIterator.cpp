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
    cursor = 0;
}

PathIterator::PathIterator(const PathIterator& orig) : path(orig.path), cursor(orig.cursor) {

}

PathIterator::~PathIterator() {

}

bool PathIterator::hasNext() const {
    if (cursor < path.size() - 1)
        return true;
    return false;
}

const FrameWithId& PathIterator::next() const {
    if (hasNext())
        return path[cursor++];
    else
        return path[cursor];
}
