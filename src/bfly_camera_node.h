#ifndef bfly_camera_node_H
#define bfly_camera_node_H

//std
#include <iostream>
#include <sstream>

//this package
#include "bfly_camera.h"

//std ros dependencies
#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <ros/package.h>
#include <sensor_msgs/SnapshotImage.h> //forked at https://github.com/beta-robots

//custom ROS dependencies
// #include "ptgrey_bfly_camera/ImageAsService.h" //custom "capture" service

//enum run mode
enum RunMode {SERVER=0,PUBLISHER};

/** \brief Point Grey's Black Fly ROS wrapper for image capture
 * 
 * Point Grey's Black Fly ROS wrapper for image capture
 * 
 * Two running modes:
 *    * SERVER: Snapshot upon request
 *    * PUBLISHER: Continuous image publication
 * 
 * In both cases the image is published thorugh the same topic
 * 
 **/
class BflyCameraNode
{
    protected:                    
        //Device object with HW API
        BflyCamera::Device *camera_;
        
        //ros node handle
        ros::NodeHandle nh_;
        
        //capture server
        ros::ServiceServer image_server_; 

        //set camera info server
        ros::ServiceServer set_camera_info_server_; 
        
        //image transport, publisher and message
        image_transport::ImageTransport image_tp_;
        image_transport::Publisher image_publisher_;      
        cv_bridge::CvImage image_;        
        
        //camera info publisher and message
        ros::Publisher camera_info_publisher_;
        sensor_msgs::CameraInfo camera_info_msg_;        
        
        //node configuration parameters
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher        
        double rate_; //loop rate
        std::string camera_frame_name_; //name of the camera frame of reference
        std::string camera_info_file_; //name of the camera info file. Where calibration data is stored
        
        //Calibration data and yaml file to store it
        cv::Mat paramsD;
        cv::Mat matrixK;
        cv::Mat matrixP;
        cv::FileStorage calibration_file_; 
        
    public:
        //constructor
        BflyCameraNode();
        
        //destructor
        ~BflyCameraNode();
        
        //returns run_mode_
        RunMode runMode() const;
        
        //returns rate_ value
        double rate() const; 
        
        //Call to device snapshot acquisition and publish the image
        void publish();
                        
    protected: 
        //sets calibration data from file. Retunrns false if file does not exist
        bool setCalibrationFromFile();
        
        //Service callback implementing the image capture
//         bool imageServiceCallback(ptgrey_bfly_camera::ImageAsService::Request  & _request, 
//                                   ptgrey_bfly_camera::ImageAsService::Response & _reply);
        bool imageServiceCallback(sensor_msgs::SnapshotImage::Request  & _request, 
                                  sensor_msgs::SnapshotImage::Response & _reply);
        
        //Service to set the camera info data. Stores data in a yaml file
        bool SetCameraInfoServiceCallback(sensor_msgs::SetCameraInfo::Request  & _request, 
                                          sensor_msgs::SetCameraInfo::Response & _reply);
        
                
};
#endif
