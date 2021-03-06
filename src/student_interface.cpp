/** 
 *  @file   student_interface.cpp
 *  @brief  Contains the implementation of genericImageListener Function.Other functions are moved out in other files 
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


namespace student {

  /**
  *  @brief Function to save images for intrinsic calibration
  *  @details Through this the images are saved from the simulator which contains checkerboard. 
  *  @param img_in Input Image
  *  @param topic topic in which the image arrives
  *  @param config_folder config folder to store the saved images
  *  @return  true/false robot found or not
  */

 void genericImageListener(const cv::Mat& img_in, std::string topic, const std::string& config_folder){

    static int image_number = 0; // image number to be saved in the destination folder
    static bool first_flag = true; // create a destination folder for the first time
    static std::string folder_path; //output folder_path

    if(first_flag){      

      std::stringstream ss;
      ss << config_folder << "/camera_image_captured/"; // Creating a folder name for storing the images Appending from the one in config file
      folder_path = ss.str();
      if(!std::experimental::filesystem::exists(folder_path)){ // If the folder doesnt exist in that location
          if(!std::experimental::filesystem::create_directories(folder_path))
              throw std::logic_error( "CANNOT CREATE DIRECTORY" );
      } // Error to check if the directory is created or nots
      first_flag = false; // open the folder only for the first time.s
    }

    cv::imshow( topic, img_in); // Show the image in the same topic name
    char c = (char)cv::waitKey(30);

    std::stringstream img_file; 
    if(c == 's'){ // if the pressed key is 's' -->save the image
        img_file << folder_path<< "raw_" <<(image_number++) << ".jpg"; // 
        cv::imwrite( img_file.str(), img_in ); // save the image in desired location
        std::cout << "Saved image " << img_file.str() << std::endl;
    }
  }
}

