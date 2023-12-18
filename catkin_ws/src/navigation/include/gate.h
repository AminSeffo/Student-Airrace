//Autor: Amin Seffo
//Date: 04.03.2023

#ifndef GATE_H
#define GATE_H

#include <ros/ros.h>
#include <vector>
#include <tf/transform_listener.h>

class Gate {
public:
    Gate(tf::Vector3 leftPylon, tf::Vector3 rightPylon) : leftPylon(leftPylon), rightPylon(rightPylon) {}
    
    tf::Quaternion getOrientation()
    {
       double angle = atan2(rightPylon.getY() - leftPylon.getY(), rightPylon.getX() - leftPylon.getX());
       angle += 1.5708;
       tf::Quaternion q;
       q.setRPY(0, 0, angle); 
       return q;
    }
    double getAngle()
    {   
        // in radian
        double angle = atan2(rightPylon.getY() - leftPylon.getY(), rightPylon.getX() - leftPylon.getX());
        angle += 1.5708;
        // convert to degree
        angle = angle * 180 / M_PI;
        return angle;
    }
    tf::Vector3 getCenter()
    {
        return (leftPylon + rightPylon) / 2;
    }
    // getter and setter from leftPylon and rightPylon
    tf::Vector3 getLeftPylon() const
    {
        return leftPylon;
    }
    void setLeftPylon(const tf::Vector3 &value)
    {
        leftPylon = value;
    }
    tf::Vector3 getRightPylon() const
    {
        return rightPylon;
    }
    void setRightPylon(const tf::Vector3 &value)
    {
        rightPylon = value;
    }
    
private:
    tf::Vector3 leftPylon;
    tf::Vector3 rightPylon;

};
#endif // GATE_H
