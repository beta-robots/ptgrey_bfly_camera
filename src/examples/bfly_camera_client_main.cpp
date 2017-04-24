
//ros dependencies
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/SnapshotImage.h> //forked at https://github.com/beta-robots

//opencv
#include "opencv2/opencv.hpp"

//node main
int main(int argc, char **argv)
{
    //init ros and node handle
    ros::init(argc, argv, "bfly_camera_client_node");
    ros::NodeHandle nh_(ros::this_node::getName()); 
        
    //call service
    ros::ServiceClient image_client;
    image_client = nh_.serviceClient<sensor_msgs::SnapshotImage>("/ptgrey_bfly_camera/image_server");
    sensor_msgs::SnapshotImage image_service; 
    if ( !image_client.call(image_service) ) // blocks here up to get an image response
    {
        ROS_ERROR("Error on getting an image as a service");
        return -1; 
    }

    //make a copy and display the received image
    cv_bridge::CvImagePtr cv_bridge_ptr; 
    cv_bridge_ptr = cv_bridge::toCvCopy(image_service.response.image, 
                                        image_service.response.image.encoding); 
    cv::imshow("Raw Image", cv_bridge_ptr->image);
    cv::waitKey(0);        
    
    //rectify image using the response camera info data
    image_geometry::PinholeCameraModel camera_model;
    camera_model.fromCameraInfo(image_service.response.camera_info);
    cv::Mat image_rectified; 
    camera_model.rectifyImage(cv_bridge_ptr->image, image_rectified);
    cv::imshow("Rectified Image", image_rectified);
    cv::waitKey(0);            
    
    //viusalize the image difference
    cv::Mat image_diff; 
    cv::subtract( cv_bridge_ptr->image, image_rectified, image_diff);
    cv::imshow("Diff Image", image_diff);
    cv::waitKey(0);  
    
    //exit program
    return 0;
}

