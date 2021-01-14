/**
 *  @file   extrinsicCalib.cpp
 *  @brief  Contains Functions related to extrinsic Calibration 
 *  @author Aravind Swaminathan
 *  @date   10-Jan-2020 
 ***********************************************/

#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>

namespace student{

  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;
  cv::Mat dist_coeffs_for_ex;


  /**
  *  @brief Function called after every mouse click
  *  @details Function obatined from Professor interface
  *  @param event mouse event occured 
  *  @param x x-position of mouse event
  *  @param y y-position of mouse event   
  */

  void mouseCallback(int event, int x, int y, int, void* p) 
  {
    if (event != cv::EVENT_LBUTTONDOWN || done.load()) return;
    
    result.emplace_back(x*show_scale, y*show_scale);
    cv::circle(bg_img, cv::Point(x,y), 20/show_scale, cv::Scalar(0,0,255), -1);
    cv::imshow(name.c_str(), bg_img);

    if (result.size() >= n) {
      usleep(500*1000);
      done.store(true);
    }
  }
  /**
  *  @brief Function to pick points from image
  *  @details Function obatined from Professor interface
  *  @param n0 number of points to be picked 
  *  @param img the image from which the points are to be chosen
  */

  std::vector<cv::Point2f> pickNPoints(int n0, const cv::Mat& img)  // Function obatined from Professor interface
  {
    result.clear();
    cv::Size small_size(img.cols/show_scale, img.rows/show_scale);
    cv::resize(img, bg_img, small_size);
    //bg_img = img.clone();
    name = "Pick " + std::to_string(n0) + " points";
    cv::imshow(name.c_str(), bg_img);
    cv::namedWindow(name.c_str());
    n = n0;

    done.store(false);

    cv::setMouseCallback(name.c_str(), &mouseCallback, nullptr);
    while (!done.load()) {
      cv::waitKey(500);
    }

    cv::destroyWindow(name.c_str());
    return result;
  }

  /**
  *  @brief Function for extrinsic calibration
  *  @details Extrinsic calibration to determine the Rotational and translational matrix. Four points will be chosen
  *  in the image plane and then these 4 points will be solved using the solvePnP interface from opencv to solve the 
  *  extrinsic problem.
  *  @param img_in input image 
  *  @param oject_points 4 points that are chosen in image
  *  @param camera_matrix The obtained camera matrix from intrinsic calibration  
  *  @param rvec Output rotational vector
  *  @param tvec Output translational vector
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec)
  */

  bool extrinsicCalib(const cv::Mat& img_in, std::vector<cv::Point3f> object_points, const cv::Mat& camera_matrix, cv::Mat& rvec, cv::Mat& tvec, const std::string& config_folder){

  std::string file_path = config_folder + "/extrinsicCalib.csv"; // General path to store the calib output file, if file already present, then read from this location

  std::vector<cv::Point2f> image_points;

  if (!std::experimental::filesystem::exists(file_path)) { // If file doesnt exist

      std::experimental::filesystem::create_directories(config_folder); 

      image_points = pickNPoints(4, img_in); //pick four points in the image

      std::ofstream output(file_path);
      if (!output.is_open()) {
          throw std::runtime_error("Cannot write file: " + file_path);
      }
      for (const auto pt: image_points) {
          output << pt.x << " " << pt.y << std::endl;
      }
      output.close();
  } else { // if file already exists
      std::ifstream input(file_path);
      if (!input.is_open()) {
          throw std::runtime_error("Cannot read file: " + file_path);
      } 
      while (!input.eof()) {
          double x, y;
          if (!(input >> x >> y)) {
              if (input.eof()) break;
              else {
                  throw std::runtime_error("Malformed file: " + file_path);
              }
          }
          image_points.emplace_back(x, y);
      }
      input.close();
  }

  bool ok = cv::solvePnP(object_points, image_points, camera_matrix, dist_coeffs_for_ex, rvec, tvec); // call solve pnp function 

  if (!ok)
      std::cerr << "FAILED SOLVE_PNP" << std::endl;

  return ok;
  }

  /**
  *  @brief Function undistort the given image
  *  @details Using the distortion coefficients obtained in the previous steps, remove the distorted 
  *  effect on the image. This is done using the opencv undistort function
  *  @param img_in input image
  *  @param img_out output image 
  *  @param camera_matrix The obtained camera matrix from intrinsic calibration  
  *  @param dist_coeffs distortion coefficients
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */
  
  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    cv::undistort(img_in,img_out,cam_matrix,dist_coeffs); // using opencv undistort() function to undistort the images and store the undistortion coefficients for the extrinsic calibration
    dist_coeffs_for_ex = dist_coeffs;
  }

  /**
  *  @brief Perspective projection
  *  @details Now to have a bird’s eye view of the image, where we need to project the 3D objects in a image plane, 
  *  carry out Perspective Projection initially.  This is again carried out with the  opencv  interfaces,   
  *  projectPoints()  and getPerspectiveTransform().  
  *  @param cam_matrix camera_matrix
  *  @param rvec Output rotational vector
  *  @param tvec Output translational vector
  *  @param object_points_plane Object points   
  *  @param dest_image_points_plane destination image points plane
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat image_points;

    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points); // Get the object points and project on a 2D image plane

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane); // Do the persepective projection with the 4 image points
  }

  /**
  *  @brief Image unwarping
  *  @details Image unwarping using opencv API warpPerspective()  
  *  @param img_in input image
  *  @param img_out output image
  *  @param transf tranformation matrix
  *  @param config_folder Output folder (if file existing then function justs reads the file to get rvec and tvec) 
  */


  void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size()); // unwarp the image to get a kind of bird's eye view
  }

}
