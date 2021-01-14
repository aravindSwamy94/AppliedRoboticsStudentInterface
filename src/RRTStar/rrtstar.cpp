/**
 *  @file   rrtstar.cpp
 *  @brief  Contains the implementation of RRT* planning functions
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/

#include "rrtstar.h"

/**
 * @brief Constructor
 */

RRTSTAR::RRTSTAR()
{
    obstacles = new RRTObstacles;
    root = new Node;
}

/**
 * @brief Set the World information
 * @param x
 * @param y
 * @return
 */

void RRTSTAR::setWorldInfo(double width,double height){
    world_width = width;
    world_height= height;
}


/**
 * @brief Set the Destination position
 * @param x
 * @param y
 * @return
 */

void RRTSTAR::setGoalPose(double x, double y){
    endPos.x() = x;
    endPos.y() = y;
}


/**
 * @brief Set the Start position of robot.
 * @param x
 * @param y
 * @param theta
 * @return
 */

void RRTSTAR::setStartPose(double x, double y,double theta){
    startPos.x() = x;
    startPos.y() = y;
    root->parent = NULL;
    root->position = startPos;
    root->orientation = theta;
    start_orient = theta;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}


void RRTSTAR::set_bot_radius(double x){
    bot_radius = x;
}

void RRTSTAR::set_goalBias(double x){
    goalBias = x;
}

void RRTSTAR::set_turn_radius(double x){
    turn_radius = x;
}

void RRTSTAR::set_rrt_star_neighbour_factor(double x){
    rrt_star_neighbour_factor = x;
}

void RRTSTAR::set_bot_follow_dubin(bool x){
    bot_follow_dubin = x;
}


void RRTSTAR::set_step_size(double x){
    step_size = x;
}

void RRTSTAR::set_max_iter(int x){
    max_iter = x;
}


/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->orientation = start_orient;
    root->cost = 0.0;
    lastNode = root;
    nodes.push_back(root);
}

/**
 * @brief Generate a random node in the field.
 * @return Random Node
 */
Node* RRTSTAR::getRandomNode()
{
    Node* ret;
    Vector2f point(drand48() * world_width, drand48() * world_height);
    float orient = drand48() * 2 * 3.142;
    if (point.x() >= 0 && point.x() <= world_width && point.y() >= 0 && point.y() <= world_height && orient > 0 && orient < 2*3.142) {
        ret = new Node;
        ret->position = point;
        ret->orientation = orient;
        return ret;
    }
    return NULL;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p Point p
 * @param q Point q 
 * @return Euclidean distance between Point P and Q  
 */
double RRTSTAR::distance(Vector2f &p, Vector2f &q)
{
    Vector2f v = p - q;
    return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point given point in world
 * @return Nearest Node in the node list
 */
Node* RRTSTAR::nearest(Vector2f point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point point in the world
 * @param radius radius to check for neighbors
 * @param out_nodes list of nodes in that radius

 */
void RRTSTAR::near(Vector2f point, float radius, vector<Node *>& out_nodes)
{
    for(int i = 0; i < (int)nodes.size(); i++) {
        double dist = distance(point, nodes[i]->position);
        if (dist < radius) {
            out_nodes.push_back(nodes[i]);
        }
    }
}



/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q given node
 * @param qNearest nearest node in list
 * @return New config with (x,y,theta)
 */
Vector3f RRTSTAR::newConfig(Node *q, Node *qNearest)
{
    Vector2f to = q->position;
    Vector2f from = qNearest->position;
    Vector2f intermediate = to - from;
    intermediate = intermediate / intermediate.norm();
    Vector2f pos = from + step_size * intermediate;
    Vector3f ret(pos.x(), pos.y(), 0.0);
    return ret;
}


/**
 * @brief Return trajectory cost.
 * @param q cost of node be computed
 * @return Cost of the node
 */
double RRTSTAR::Cost(Node *q)
{
    return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom from node
 * @param qTo to node
 * @return The path cost from Node a to Node b
 */
double RRTSTAR::PathCost(Node *qFrom, Node *qTo)
{
    return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node *qNearest, Node *qNew)
{
    qNew->parent = qNearest;
    qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
    qNearest->children.push_back(qNew);
    nodes.push_back(qNew);
    lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return true/false - reached destination/not reached
 */
bool RRTSTAR::reached()
{
    if (distance(lastNode->position, endPos) < goalBias)
        return true;
    return false;
}

void RRTSTAR::setStepSize(double step)
{
    step_size = step;
}

void RRTSTAR::setMaxIterations(int iter)
{
    max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root root node or the first node
 */
void RRTSTAR::deleteNodes(Node *root)
{
    for(int i = 0; i < (int)root->children.size(); i++) {
        deleteNodes(root->children[i]);
    }
    delete root;
}
