
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
//     sensor_msgs::ImageConstPtr image_msg_ptr(&image_service.response.image); 
//     std::cout << "image_msg_ptr.use_count(): " << image_msg_ptr.use_count() << std::endl; 
    cv_bridge::CvImagePtr cv_bridge_ptr(
                        cv_bridge::toCvCopy(image_service.response.image, 
                                            image_service.response.image.encoding));     
//     cv_bridge::CvImageConstPtr cv_bridge_ptr(
//                     cv_bridge::toCvCopy(&image_service.response.image, 
//                                         image_service.response.image.encoding)); 
    cv::imshow("Response Image (raw)", cv_bridge_ptr->image);
    cv::waitKey(0);        
    
    //display received image (sharing instead of copying... causes run time core dumped error)
//     cv_bridge::CvImageConstPtr cv_bridge_ptr;
//     sensor_msgs::ImageConstPtr image_msg_ptr(&image_service.response.image); 
//     cv_bridge_ptr = cv_bridge::toCvShare(image_msg_ptr, image_service.response.image.encoding); 
//     cv::imshow("Response Image (raw)", cv_bridge_ptr->image);
//     cv::waitKey(0);        
        
    //rectify image
    
    //display rectified image
    
    //release
//     std::cout << __LINE__ << std::endl;
//     image_msg_ptr.reset(); 
//     std::cout << __LINE__ << std::endl;
    
//     delete (image_msg_ptr.get()); 
//     std::cout << __LINE__ << std::endl;
//     delete (cv_bridge_ptr.get());
//     std::cout << __LINE__ << std::endl;
    
    //exit program
    return 0;
}
