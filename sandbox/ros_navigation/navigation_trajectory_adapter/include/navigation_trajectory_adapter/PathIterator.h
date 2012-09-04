/* 
 * File:   PathIterator.h
 * Author: alexey
 *
 * Created on September 3, 2012, 11:13 AM
 */

#ifndef PATHITERATOR_H
#define	PATHITERATOR_H

#include <vector>

class FrameWithId;
class PathInterpolation;

class PathIterator {
public:
    PathIterator(const std::vector <FrameWithId>& path);
    PathIterator(const PathIterator& orig);
    virtual ~PathIterator();

    bool hasNext() const;
    const FrameWithId& next() const;

private:
    const std::vector <FrameWithId>& path;
    mutable size_t cursor;
    static const FrameWithId& dummy; 
};

#endif	/* PATHITERATOR_H */

