/** 
 *  @file   dubins_local.h
 *  @brief  Contains function declaration related to dubins planner 
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#ifndef DUBINS_HPP_
#define DUBINS_HPP_

#include <vector>
#include "utils.hpp"	// Path struct

double sinc(double t);

double mod2pi(double ang);

double rangeSymm(double const& ang);

bool check(double const& s1, double const& k0, double const& s2, double const& k1, double const& s3, double const& k2, double const& th0, double const& thf);

void scaleToStandard(double x0, double y0, double th0, double xf, double yf, double thf, double kmax, double& sc_th0, double& sc_thf, double& sc_kmax, double& lambda);

void scaleFromStandard(double lambda, double sc_s1, double sc_s2, double sc_s3, double& s1, double& s2, double& s3);


/**
*  @brief Compute the arcs for LSL configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void LSL(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);

/**
*  @brief Compute the arcs for RSR configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void RSR(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);

/**
*  @brief Compute the arcs for LSR configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void LSR(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);

/**
*  @brief Compute the arcs for RSL configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void RSL(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);

/**
*  @brief Compute the arcs for RLR configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void RLR(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);
/**
*  @brief Compute the arcs for LRL configuration  
*  @param sc_th0 Start theta
*  @param sc_thf Final theta
*  @param sc_kmax Curvature max
*  @param ok Check if this configuration fits the given input
*  @param sc_s1 Arc 1 coeff
*  @param sc_s2 Arc 2 coeff 
*  @param sc_s3 Arc 3 coeff   
*/
void LRL(double sc_th0, double sc_thf, double sc_kmax, bool& ok, double& sc_s1, double& sc_s2, double& sc_s3);

/// @brief Structure for a Dubins arc	
struct dubinsArc {
	double x0, y0, th0, k, s, xf, yf, thf;
};

/// @brief Structure for a Dubins curve
struct dubinsCurve {
	dubinsArc arc_1, arc_2, arc_3;
	double L;
};

void set_dubinsArc(dubinsArc& ptr, double x0, double y0, double th0, double k, double s);

void set_dubinsCurve(dubinsCurve& curve_ptr, double x0, double y0, double th0, double s1, double s2, double s3, double k0, double k1, double k2);

/**
*  @brief Compute shortest path using dunins confiuration  
*  @param curve Output curve 
*  @param x0 Start location x
*  @param y0 Start location y
*  @param th0 Start location theta
*  @param xf End location x
*  @param yf End location y
*  @param thf End location theta
*  @param kmax Maximum curvature of the robot
*/
void dubins_shortest_path(dubinsCurve& curve, double const& x0, double const& y0, double const& th0, double const& xf, double const& yf, double const& thf, double const& kmax);

/**
*  @brief Discretize the dubins curve to points   
*  @param curve The dubins curve obtained 
*  @param npts Number of points for the whole curve
*  @return Output path structure
*/
Path getPath(dubinsCurve curve,int npts);
#endif
