#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include "CollisionOrNot.h"

namespace collision_or_not {
// check collision


int lineIntersectSide(Line *line1,Line *line2)
{

 
	double fC = (line2->y1 - line1->y1) * (line1->x1 - line1->x2) - (line2->x1 - line1->x1) * (line1->y1 - line1->y2);
	double fD = (line2->y2 - line1->y1) * (line1->x1 - line1->x2) - (line2->x2 - line1->x1) * (line1->y1 - line1->y2);
	
	if(fC * fD > 0)
		return 0;
 
	return 1;
}
 
int sideIntersectSide(Line *line1,Line *line2)
{
	if(!lineIntersectSide(line1,line2))
		return 0;
	if(!lineIntersectSide(line2,line1))
		return 0;
 
	return 1;
}
 


double MaxOf_x(Line *line)
{
    return (line->x1>line->x2)?line->x1:line->x2;
}

double MinOf_x(Line *line)
{
    return (line->x1<line->x2)?line->x1:line->x2;
}

double MaxOf_y(Line *line)
{
    return (line->y1>line->y2)?line->y1:line->y2;
}
double MinOf_y(Line *line)
{
    return (line->y1<line->y2)?line->y1:line->y2;
}


void initialize(Line *line,double x1,double y1,double x2,double y2) 
{
    //line=(Line *)malloc(sizeof(Line));
    line->x1=x1;
    line->y1=y1;
    line->x2=x2;
    line->y2=y2;
    //return line;
}



void K_B_of_line(Line *line)      
{
    if(line->x1==line->x2)
    {
        line->k_=-100;
    }
    else
    {
        line->k=(double)(line->y2-line->y1)/(line->x2-line->x1);
        line->b=line->y1-line->k*line->x1;
    }

}

void equation_result(Line *line1,Line *line2,int *p)
{

    int num=0,num1=0,num2=0;


    if (line1->k_ == -100 || line2->k_ == -100){
    	if(line1->k_ == -100 && line2->k_ != -100)
	    {
	        if(MaxOf_x(line2)<line1->x1||MinOf_x(line2)>line1->x1)
	        {
	            ;
	        }
	        else if((line2->k*line1->x1+line2->b) < MinOf_y(line1) || (line2->k*line1->x1+line2->b)>MaxOf_y(line1))
	        {
	            ;
	        }
	        else
	            num1++;
	    }
	    else if(line2->k_ == -100 && line1->k_ != -100)
	    {
	        if(MaxOf_x(line1) < line2->x1||MinOf_x(line1) > line2->x1)
	        {
	            ;
	        }
	        else if((line1->k*line2->x1+line1->b) < MinOf_y(line2) || (line1->k*line2->x1+line1->b) > MaxOf_y(line2))
	        {
	            ;
	        }
	        else
	            num1++;
	    }
	    else if(line1->k_ == -100 && line1->k_ == line2->k_ && line1->x1 == line2->x1)
	    {
	        if( MaxOf_y(line1)< MinOf_y(line2) || MaxOf_y(line2) < MinOf_y(line1) )
	            ;
	        else
	            num1++;
	    }
	}
	else{
		if (line1->k == line2->k){
			if(line1->b == line2->b  && line1->k != 0 )
		    {
		        if(MaxOf_x(line1) < MinOf_x(line2) || MaxOf_x(line2) < MinOf_x(line1))
		        {
		           ;
		        }
		        else
		            num2++;
		
		    }
		
		    else if ( line1->k == 0 && line1->b == line2->b )
		    {
		        if(MaxOf_x(line1) < MinOf_x(line2) || MaxOf_x(line2) < MinOf_x(line1))
		            ;
		        else
		            num2++;
		    }
		    
		    else if ( line1->b != line2->b && line1->k != 0)
		    {
		        ;
		
		    }
		
		    else if (line1->b != line2->b && line1->k == 0 )
		    {
		    	;
		    }
		}
		else{
			if(sideIntersectSide(line1,line2) == 1){
				num++;
			}

		}
	}


    if(num > 0 || num1 > 0  || num2 > 0)
        (*p)++;

}

int collisionBetweenCar(double x[], double y[])
{

    Line** line1 = new Line*[8];
    for(int i=0; i<8; i++){
        line1[i] = new Line;
    }
    //Line* line1[8];
    //line1 = (Line *)malloc(sizeof(Line*)*8);

    int *p = new int(0);
    int j=0;
    int i=0;

    for(i=0; i<4; i++)     
    {
    	if(i!=3){
            //line1[i] = (Line *)malloc(sizeof(Line));
	    	initialize(line1[i],x[i],y[i],x[i+1],y[i+1]);
	        K_B_of_line(line1[i]);	
		}
        
        if(i==3)
        {
            //line1[3]= (Line *)malloc(sizeof(Line));
            initialize(line1[3],x[3],y[3],x[0],y[0]);
            K_B_of_line(line1[3]);
        }
    }
    for(i=4; i<8; i++)     
    {
    	if(i!=7){
            //line1[i] = (Line *)malloc(sizeof(Line));
    		initialize(line1[i],x[i],y[i],x[i+1],y[i+1]);
        	K_B_of_line(line1[i]);
		}
        if(i==7)
        {
            //line1[7]= (Line *)malloc(sizeof(Line));
            initialize(line1[7],x[7],y[7],x[4],y[4]);
            K_B_of_line(line1[7]);
        }
    }
    for(i=0; i<4; i++)     
    {
        for(j=4; j<8; j++)
        {
            equation_result(line1[i],line1[j],p);
        }
    }

    for(i = 0; i < 8; i++){
        delete line1[i];
        line1[i] = NULL;
    }

    delete[] line1;
    line1 = NULL;
    
    
    if(*p == 0){
        delete p;
        p = NULL;
    	return 0;
	} 
	if(*p > 0){
        delete p;
        p = NULL;
		return 1;
	}


} 

}


