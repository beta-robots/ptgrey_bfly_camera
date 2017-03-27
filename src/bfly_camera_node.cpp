#include "bfly_camera_node.h"

BflyCameraNode::BflyCameraNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_)
{
    //Set node parameters //TODO: get them from yaml file
    run_mode_ = PUBLISHER; 
    rate_ = 1; 
    camera_frame_name_ = "ptgrey_bfly_camera";
    
    //constructs the camera object interfacing with HW
    camera_ = new BflyCamera(); 
    
    //starts camera image acquisition
    if ( camera_->startAcquisition() == BFLY_ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Error starting image acquisition" << std::endl;
        return;
    }
    
}
        
BflyCameraNode::~BflyCameraNode()
{
    //free allocated memeory
    delete camera_; 
}
        
RunMode BflyCameraNode::runMode() const
{
    return run_mode_; 
}

double BflyCameraNode::rate() const
{
    return rate_; 
}

void BflyCameraNode::publish()
{
    //Get image data in openCV format
    camera_->getCurrentImage(image_.image);
    
    //Get timestamp and fill the header
    ros::Time ts = ros::Time::now();
    image_.header.seq ++;
    image_.header.stamp = ts;
    image_.header.frame_id = camera_frame_name_; 
    image_.encoding = sensor_msgs::image_encodings::MONO8;
    
    //publish the image    
    image_publisher_.publish(image_.toImageMsg());        
    
}

bool BflyCameraNode::imageServiceCallback(
            ptgrey_bfly_camera::ImageAsService::Request  & _request, 
            ptgrey_bfly_camera::ImageAsService::Response & _reply)
{
    //publish images
    for (unsigned int ii=0; ii< _request.num_images; ii++)
    {        
        this->publish();
    }
    
    //set the reply
    _reply.width = image_.image.cols; 
    _reply.height = image_.image.rows; 
    
    //return
    return true;     
}
