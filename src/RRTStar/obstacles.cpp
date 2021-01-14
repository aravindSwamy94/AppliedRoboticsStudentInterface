/** 
 *  @file   obstacles.cpp
 *  @brief  Contains the implementation of Obstacle processing related function in RRT* implementaion 
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#include "obstacles.h"
#include<iostream>
using namespace std;
RRTObstacles::RRTObstacles()
{
}

/**
 * @brief Obstacles are stored as circles. Circles is denoted by center point and radius.
 * @param radius
 * @param centerPoint
 */
void RRTObstacles::addObstacle(double radius, Vector2f centerPoint)
{
    obstacles.push_back(make_pair(radius, centerPoint));
}


/**
 * @brief Check if point is inside the obstacle circle 
 * @param radius
 * @param secondPoint
 */
bool isInside(int circle_x, int circle_y, 
                   int rad, int x, int y) 
{ 
    if ((x - circle_x) * (x - circle_x) + 
        (y - circle_y) * (y - circle_y) <= rad * rad) 
        return true; 
    else
        return false; 
} 

bool checkCollision(int a, int b, int c,  
                  int x, int y, int radius) 
{ 
    // Finding the distance of line from center. 
    int dist = (abs(a * x + b * y + c)) /  
                     sqrt(a * a + b * b); 

    if(radius >= dist+10 )
        return 1;

    else return 0;
} 

/**
 * @brief Check if a line segment intersects a rectangle.
 * @param p1
 * @param p2
 * @return
 */
bool RRTObstacles::isSegmentInObstacle(Vector2f &p1, Vector2f &p2)
{
    double a = p1.y() - p2.y();
    double b = p2.x() - p1.x();
    double c = (p1.x()*p2.y()) - (p2.x()*p1.y());

    for(int i = 0; i < (int)obstacles.size(); i++) {
        double r = obstacles[i].first;
        double x = obstacles[i].second.x();
        double y = obstacles[i].second.y();        
	if(isInside(p1.x(),p1.y(),r,x,y) or isInside(p2.x(),p2.y(),r,x,y))
            return true;
        if(checkCollision(a,b,c,x,y,r)){
            return true;
	}
    }

    //cout<<"Node didnt collide"<<endl;
    return false;
}
