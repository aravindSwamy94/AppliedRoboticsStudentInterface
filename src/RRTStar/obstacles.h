/**
 *  @file   obstacles.h
 *  @brief  Contains the declaration of Obstacles class
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <vector>
#include <eigen3/Eigen/Dense>
#include <assert.h>

using namespace Eigen;
using namespace std;

/**
@brief Obstacles class for RRT* Star Implementation
*/
class RRTObstacles
{
public:
    RRTObstacles();
    void addObstacle(double radius, Vector2f secondPoint);
    bool isSegmentInObstacle(Vector2f &p1, Vector2f &p2);
    vector<pair<double, Vector2f> > obstacles;
};

#endif // OBSTACLES_H
