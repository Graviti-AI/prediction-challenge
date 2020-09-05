#ifndef _COLLISION_OR_NOT_H
#define _COLLISION_OR_NOT_H

#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>


namespace collision_or_not {

typedef struct line
    {
        double x1,y1;
        double x2,y2;
        double k;             
        double b;            
        double k_;
    } Line;



    int lineIntersectSide(Line *line1,Line *line2);
    int sideIntersectSide(Line *line1,Line *line2);
    double MaxOf_x(Line *line);
    double MinOf_x(Line *line);
    double MaxOf_y(Line *line);
    double MinOf_y(Line *line);
    void initialize(Line *line,double x1,double y1,double x2,double y2);
    void K_B_of_line(Line *line);
    void equation_result(Line *line1,Line *line2,int *p); 
    int collisionBetweenCar(double x[], double y[]);
}


      
#endif
