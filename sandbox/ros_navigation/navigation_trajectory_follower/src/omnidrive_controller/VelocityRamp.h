/* 
 * File:   VelocityRamp.h
 * Author: alexey
 *
 * Created on May 26, 2012, 9:50 AM
 */

#ifndef VELOCITYRAMP_H
#define	VELOCITYRAMP_H

class VelocityRamp {
public:
    VelocityRamp();
    VelocityRamp(const VelocityRamp& orig);
    virtual ~VelocityRamp();

    void setTotalDistance(float totalDistance);
    float getTotalDistance();
    
    void setActualDistance(float actualDistance);
    float getActualDistance();

    void setActualVelocity(float actualVelocity);
    float getActualVelocity();

    void setAcceleration(float acceleration);
    float getAcceleration();
    
    void setDeceleration(float deceleration);
    float getDeceleration();

    void setMaxVelocityLimits (float negative, float positive);
    void getMaxVelocityLimits(float& negative, float& positive);

    void setMinVelocityLimits(float negative, float positive);
    void getMinVelocityLimits(float& negative, float& positive);
    
    void setInitialVelocity(float initialVelocity);
    float getInitialVelocity();
    
    void setFinalVelocity(float finalVelocity);
    float getFinalVelocity();
  
    float computeVelocity(float actualDistance);
           
private:
    float initialVelocity;      // m/s
    float actualVelocity;
    float finalVelocity;
        
    float minNegativeVelocityLimit;          // m/s
    float minPositiveVelocityLimit;          
    float maxNegativeVelocityLimit;
    float maxPositiveVelocityLimit;
    
    float acceleration;         // m/s^2
    float deceleration;
      
    float totalDistance;        // meters
    float actualDistance;

};

#endif	/* VELOCITYRAMP_H */

