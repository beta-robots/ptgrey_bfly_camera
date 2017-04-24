
//ros dependencies
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
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
    cv::imshow("Response Image (raw)", cv_bridge_ptr->image);
    cv::waitKey(0);        
    
    //exit program
    return 0;
}
