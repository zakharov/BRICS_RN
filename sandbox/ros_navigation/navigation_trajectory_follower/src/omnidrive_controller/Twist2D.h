/* 
 * File:   Twist2D.h
 * Author: alexey
 *
 * Created on May 26, 2012, 9:53 AM
 */

#ifndef TWIST2D_H
#define	TWIST2D_H

class Twist2D {
public:
    Twist2D();
    Twist2D(float x, float y, float theta);
    Twist2D(const Twist2D& orig);
    const Twist2D& operator=(const Twist2D& orig);
    virtual ~Twist2D();

    friend Twist2D operator+(const Twist2D& op1, const Twist2D& op2);
   
    float getX() const;
    void setX(float x);
    float getY() const;
    void setY(float y);
    float getTheta() const;
    void setTheta(float theta);

private:
    float x;
    float y;
    float theta;
};


#endif	/* TWIST2D_H */

