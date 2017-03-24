#ifndef bfly_camera_node_H
#define bfly_camera_node_H

//std
#include <iostream>

//this package
#include "bfly_camera.h"

//std ros dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//custom ROS dependencies
#include "ptgrey_bfly_camera/ImageAsService.h" //custom "capture" service
// #include <ensenso_nx/ensenso_nx_paramsConfig.h> //ROS dynamic configure

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
        BflyCamera *camera_;
        
        //ros node handle
        ros::NodeHandle nh_;
        
        //capture server
        ros::ServiceServer image_server_; 
        
        //image transport and publisher
        image_transport::ImageTransport image_tp_;
        image_transport::Publisher image_publisher_;      

        //published image
        cv_bridge::CvImage image_;        
        
        //node configuration parameters
        RunMode run_mode_;//run mode: The node acts as a server, or a continuous pcl publisher        
        double rate_; //loop rate
        
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
        //Service callback implementing the image capture
        bool imageServiceCallback(ptgrey_bfly_camera::ImageAsService::Request  & _request, 
                                  ptgrey_bfly_camera::ImageAsService::Response & _reply);
                
};
#endif
