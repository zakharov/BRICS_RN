/* 
 * File:   Stopwatch.h
 * Author: alexey
 *
 * Created on August 31, 2012, 4:57 PM
 */

#ifndef STOPWATCH_H
#define	STOPWATCH_H

#include <ctime>

class Stopwatch {
public:
    Stopwatch();
    Stopwatch(const Stopwatch& orig);
    virtual ~Stopwatch();
    
    void start();
    void stop();
    void pause();
    
    double getStartTime() const;
    double getStopTime() const;
    double getElapsedTime() const;
    
private:
    
    double convertToMs(const timespec& time) const;
    
    timespec startTime;
    timespec stopTime;
    double elapsedTime;
    bool isPause;
    
};

#endif	/* STOPWATCH_H */

