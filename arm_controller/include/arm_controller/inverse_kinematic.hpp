#ifndef IK_H
#define IK_H

#include <ros/ros.h>
#include <iostream>
#include <cmath>

#define rad M_PI / 180
#define link1 12.5
#define link2 16.5
#define link3 5
#define deg 180/ M_PI

#define x0 0
#define y0 16

float inverse_teta1(float data, float y){
    float dis = data -10;

    // x standby + distance + link to sensor - sensor to grip
    float x = 18.8 + dis + 0.2; //kalau 7cm

    float dy = abs(y0 - y);
    float dx = abs(x - x0);
    float b = sqrt(dx * dx + dy * dy);
    
    float a = (link1*link1) + (b*b) - (link2*link2);
    a = a/(2*link1*b);
    a = acos(a);
    
    float z = acos(dx/b);
    
    float teta1 = a - z;    

    return teta1;
}

float inverse_teta2(float data, float teta1, float y){

    float dis = data - 10;

    // x standby + distance + link to sensor - sensor to grip
    float x = 18.8 + dis + 0.2; 

    float dy = abs(y0 - y);
    float dx = abs(x - x0);
    float b = sqrt(dx * dx + dy * dy);

    float Beta = link1 * link1 + link2 * link2 - b * b;
    Beta = Beta / (2 * link1 * link2);
    Beta = acos(Beta);
    float teta2 = 180*rad - Beta;

    return teta2;

}

float inverse_teta3(float teta1, float teta2){
    float teta3 = teta2 - teta1;
    return teta3; 
}

#endif