//#pragma once
#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"

#include <stdexcept>
#include <atomic>
#include <unistd.h>

#include <sstream>
#include <experimental/filesystem>

namespace student{


  //-------------------------------------------------------------------------
  //          EXTRINSIC CALIB IMPLEMENTATION
  //-------------------------------------------------------------------------

  // Defintion of the function pickNPoints and the callback mouseCallback.
  // The function pickNPoints is used to display a window with a background
  // image, and to prompt the user to select n points on this image.
  static cv::Mat bg_img;
  static std::vector<cv::Point2f> result;
  static std::string name;
  static std::atomic<bool> done;
  static int n;
  static double show_scale = 1.0;
  cv::Mat dist_coeffs_for_ex;

  void mouseCallback(int event, int x, int y, int, void* p) // Function obatined from Professor interface
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

  void imageUndistort(const cv::Mat& img_in, cv::Mat& img_out, 
          const cv::Mat& cam_matrix, const cv::Mat& dist_coeffs, const std::string& config_folder){
    cv::undistort(img_in,img_out,cam_matrix,dist_coeffs); // using opencv undistort() function to undistort the images and store the undistortion coefficients for the extrinsic calibration
    dist_coeffs_for_ex = dist_coeffs;
  }

  void findPlaneTransform(const cv::Mat& cam_matrix, const cv::Mat& rvec, 
                        const cv::Mat& tvec, const std::vector<cv::Point3f>& object_points_plane, 
                        const std::vector<cv::Point2f>& dest_image_points_plane, 
                        cv::Mat& plane_transf, const std::string& config_folder){
    cv::Mat image_points;

    cv::projectPoints(object_points_plane, rvec, tvec, cam_matrix, cv::Mat(), image_points); // Get the object points and project on a 2D image plane

    plane_transf = cv::getPerspectiveTransform(image_points, dest_image_points_plane); // Do the persepective projection with the 4 image points
  }


void unwarp(const cv::Mat& img_in, cv::Mat& img_out, const cv::Mat& transf, 
            const std::string& config_folder){
    cv::warpPerspective(img_in, img_out, transf, img_in.size()); // unwarp the image to get a kind of bird's eye view
  }

}
