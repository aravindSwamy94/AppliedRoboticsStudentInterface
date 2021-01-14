/** 
 *  @file   findRobot.cpp
 *  @brief  Contains the implementation of findRobot Function 
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/
#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>
#include <config4cpp/Configuration.h>
using namespace config4cpp;
using namespace std;

namespace student{
    /**
    *  @brief find Robot function in student interface
    *  @details Directly utilized the function provided by the teaching assistant as I found that implementation was 
    *  already in the best shape.
    *  RGB→HSV→blue mask→Contours→Approximate polynomial→find 3 points of polynomial→Find center of 
    *  triangle→Find angle between top vertex and center(Orientation)→return state(x,y,ψ) 
    *  @param img_in Input Image
    *  @param scale scaling factor
    *  @param triangle The outpu triangle of robot
    *  @param x robot pose (center) x
    *  @param y robot pose (center) y
    *  @param theta robot pose (initial) theta
    *  @param config_folder config folder if any configuration params to be loaded
    *  @return  true/false robot found or not
    */
    bool findRobot(const cv::Mat& img_in, const double scale, Polygon& triangle, double& x, double& y, double& theta, const std::string& config_folder){

        /*Steps involved in find robot funtion one by one*/
        std::string file_path = config_folder + "/config_params.cfg";
        const char *  configFile =file_path.c_str() ;
        Configuration *  cfg = Configuration::create();
        const char *     scope = "";
        cfg->parse(configFile);
        bool find_robot_debug_plot = cfg->lookupBoolean(scope, "find_robot_debug_plot");
        //cout<<"find_robot_debug_plot "<<find_robot_debug_plot<<endl;

        cv::Mat hsv_img;
        cv::cvtColor(img_in, hsv_img,cv::COLOR_BGR2HSV); /* convert the input image to hsv image*/

        cv::Mat blue_mask;    
    //cv::inRange(hsv_img, cv::Scalar(100, 120, 150), cv::Scalar(135, 255, 255), blue_mask);

        //cv::inRange(hsv_img, cv::Scalar(80, 140, 150), cv::Scalar(140, 255, 255), blue_mask); /*apply blue mask for the robot detection, given robot will be a blue triangle*/
        cv::inRange(hsv_img, cv::Scalar(90, 80, 70), cv::Scalar(130, 255, 255), blue_mask);
        std::vector<std::vector<cv::Point>> contours;    
        cv::findContours(blue_mask, contours,cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); /* find contours to detect the contours on the image*/
        cv::Mat contours_img;
        if(find_robot_debug_plot){
            cv::imshow("findRobotHsv", hsv_img);
            cv::imshow("findRobotMask", blue_mask);            
            contours_img = img_in.clone();
            cv::drawContours(contours_img, contours, -1, cv::Scalar(0,0,0), 4, cv::LINE_AA); // debug plot
            std::cout << "N. contours: " << contours.size() << std::endl;
        }

        std::vector<cv::Point> approx_curve;
        std::vector<std::vector<cv::Point>> contours_approx;
        bool found=false;
        for(int i=0; i<contours.size(); ++i)
        { 
            cv::approxPolyDP(contours[i], approx_curve, 30, true); /* approximate the contours to obtain a triangle*/

            if (approx_curve.size() != 3) continue; /* check if the obtained curve is a triangle */

            double area = cv::contourArea(approx_curve);

            if(find_robot_debug_plot){ 
                std::cout << (i+1) << ") Contour size: " << contours[i].size() << std::endl;
                std::cout << (i+1) << ") Aprox Contour size: " << approx_curve.size() << std::endl;
                std::cout << "Area: " << area << std::endl;
                contours_approx = {approx_curve};
                cv::drawContours(contours_img, contours_approx, -1, cv::Scalar(0,0,255), 3, cv::LINE_AA); //debug plot
            }
            found=true; // robot found
            break;
        }


  
        for (const auto& pt: approx_curve) {        
            triangle.emplace_back(pt.x/scale, pt.y/scale); /* populate the triangle points*/
        }

        double cx = 0, cy = 0;

        for (auto vertex: triangle) 
        {
            cx += vertex.x; 
            cy += vertex.y;
        }
        /* to get the center of triangle, this gives the position(x,y) of the robot  */
        cx /= triangle.size(); 
        cy /= triangle.size();

        double dst = 0;
        Point top_vertex; 
        /* to determine the third state, orientation of the robot */
        for (auto& vertex: triangle)
        {
            const double dx = vertex.x-cx;      
            const double dy = vertex.y-cy;
            const double curr_d = dx*dx + dy*dy;
            if (curr_d > dst)
            { 
              dst = curr_d;
              top_vertex = vertex; // to find the top vertex of the robot
            }
        }


        x = cx;
        y = cy;

        const double dx = cx - top_vertex.x;
        const double dy = cy - top_vertex.y;
        theta = std::atan2(dy, dx); // atan2 of two points gives the orientation of the robot

        if(find_robot_debug_plot){           
            cv::Point cv_baricenter(x*scale, y*scale); // convert back m to px
            cv::Point cv_vertex(top_vertex.x*scale, top_vertex.y*scale); // convert back m to px
            cv::line(contours_img, cv_baricenter, cv_vertex, cv::Scalar(0,255,0), 3);
            cv::circle(contours_img, cv_baricenter, 5, cv::Scalar(0,0,255), -1);
            cv::circle(contours_img, cv_vertex, 5, cv::Scalar(0,255,0), -1);      
            std::cout << "(x, y, theta) = " << x << ", " << y << ", " << theta*180/M_PI << std::endl;
        }
        

        if(find_robot_debug_plot){   // do this only if FIND_DEBUG_PLOT is defined 
          cv::imshow("findRobot", contours_img);
          cv::waitKey(1);
        }
        return found;
    }
}
