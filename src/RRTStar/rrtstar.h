/**
 *  @file   rrtstar.h
 *  @brief  Contains the declation of RRT* class and RRT* Node
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#ifndef RRTSTAR_H
#define RRTSTAR_H
#include "obstacles.h"
#include <stdlib.h>
#include <vector>
#include <math.h>
#include<iostream>
using namespace std;
using namespace std;
using namespace Eigen;

/**
@brief RRT* Node strut
*/
struct Node {
    vector<Node *> children;
    Node *parent;
    Vector2f position;
    float orientation;
    double cost;
};

/**
@brief RRTStar Class 
*/
class RRTSTAR
{
public:
    RRTSTAR();
    void initialize();
    Node* getRandomNode();
    Node* nearest(Vector2f point);
    void near(Vector2f point, float radius, vector<Node *>& out_nodes);
    double distance(Vector2f &p, Vector2f &q);
    double Cost(Node *q);
    double PathCost(Node *qFrom, Node *qTo);
    Vector3f newConfig(Node *q, Node *qNearest);

    void add(Node *qNearest, Node *qNew);
    bool reached();
    void setStepSize(double step);
    void setMaxIterations(int iter);
    void deleteNodes(Node *root);

    /*Functions to set the parameters*/
    void setStartPose(double x,double y,double theta);
    void setGoalPose(double x, double y);
    void setWorldInfo(double width,double height);
    void set_max_iter(int x);
    void set_step_size(double x);
    void set_bot_radius(double x);
    void set_goalBias(double x);
    void set_turn_radius(double x);
    void set_rrt_star_neighbour_factor(double x);
    void set_bot_follow_dubin(bool x);

    RRTObstacles *obstacles; /** obstacle information  */ 
    vector<Node *> nodes; /** List of nodes in explored database*/ 
    vector<Node *> path; /**path with tracing the nodes */ 
    Node *root; /** root node (initial pose) */ 
    Node *lastNode; /** last Node of the rrt* exploration*/ 

    /* Parameters to be set before using RRT*/
    Vector2f startPos; /** Start pose */ 
    Vector2f endPos; /** end pose */
    double start_orient; /** Start orientation */
    int max_iter; /** maximum iterations */
    double step_size; /** Step size(distance between points) */
    double world_width; /** World width  */ 
    double world_height;/** World height  */  
    double bot_radius; /** bot radius */
    double goalBias; /** goal Bias */
    double turn_radius; /** Robot turn radius  */ 
    bool bot_follow_dubin; 
    double rrt_star_neighbour_factor; /** Neighbour distance to search for rewiring*/ 
    
};

#endif // RRTSTAR_H
