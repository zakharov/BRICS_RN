/* 
 * File:   Pose2D.h
 * Author: alexey
 *
 * Created on May 26, 2012, 9:53 AM
 */

#ifndef POSE2D_H
#define	POSE2D_H

class Pose2D {
public:
    Pose2D();
    Pose2D(float x, float y, float theta);
    Pose2D(const Pose2D& orig);
    virtual ~Pose2D();

    friend Pose2D operator+(const Pose2D& op1, const Pose2D& op2);
    const Pose2D& operator=(const Pose2D& orig);
    float getX() const;
    float getY() const;
    float getTheta() const;

private:
    float x;
    float y;
    float theta;

};

#endif	/* POSE2D_H */

