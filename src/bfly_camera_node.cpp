#include "bfly_camera_node.h"

BflyCameraNode::BflyCameraNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_)
{
    //Set node parameters //TODO: get them from yaml file
    run_mode_ = PUBLISHER; 
    rate_ = 1; 
    
    //constructs the camera object interfacing with HW
    camera_ = new BflyCamera(); 
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
    //TODO
}

bool BflyCameraNode::imageServiceCallback(
            ptgrey_bfly_camera::ImageAsService::Request  & _request, 
            ptgrey_bfly_camera::ImageAsService::Response & _reply)
{
    //TODO
}
