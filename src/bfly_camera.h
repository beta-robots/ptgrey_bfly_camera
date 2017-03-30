#ifndef bflyCamera_H
#define bflyCamera_H

//includes
#include <iostream>
#include "FlyCapture2.h"
#include "opencv2/core/core.hpp"

//
namespace BflyCamera
{
    
//constants
const int ERROR = -1;
const int SUCCESS = 1;

//enums
enum videoMode {MODE0 = 0, MODE1, MODE2, MODE3, MODE4, MODE5};
enum pixelFormat {MONO8 = 0, RGB8};


/** \brief Class for openCV image acquisition from BlackFly Point Grey camera
 * 
 * Class for openCV image acquisition from BlackFly Point Grey camera
 * 
 **/
class Device
{
      protected:
            bool init_ok_;
            FlyCapture2::Error flycap_error_;
            FlyCapture2::PGRGuid flycap_guid_;            
            FlyCapture2::GigECamera flycap_camera_;
            FlyCapture2::Image flycap_image_;  
            FlyCapture2::GigEImageSettings flycap_image_settings_;
            cv::Mat opencv_image_;

      public:
            Device();
            ~Device();
            bool isInitOk() const;
            int open();
            int close();
            int configure(videoMode _v_mode, pixelFormat _px_format);
            int configure(unsigned int _stream_ch);
            int startAcquisition();
            int stopAcquisition();
            double getFrameRate();
            int getCurrentImage(cv::Mat & _img);
            pixelFormat getPixelFormat() const; 
            void printCameraInfo();
            void printImageInfo() const;
};//end class

}//end namespace

#endif
