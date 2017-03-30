#include "bfly_camera_node.h"

BflyCameraNode::BflyCameraNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_)
{
    //local vars
    BflyCamera::videoMode video_mode; 
    BflyCamera::pixelFormat pixel_format; 
    int param_int; 
    
    //Configure node according yaml params
    nh_.getParam("run_mode", param_int); this->run_mode_ = (RunMode)param_int; 
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->camera_frame_name_);
    nh_.getParam("video_mode", param_int); video_mode = (BflyCamera::videoMode)param_int;
    nh_.getParam("pixel_format", param_int); pixel_format = (BflyCamera::pixelFormat)param_int;
    
    //init the image publisher
    image_publisher_ = image_tp_.advertise("bfly_raw_image", 1);
    
    //init server
    image_server_ = nh_.advertiseService("bfly_server", &BflyCameraNode::imageServiceCallback, this);    
    
    //constructs the camera object interfacing with HW
    camera_ = new BflyCamera::Device(); 
    
    //open/connect to the HW device
    if ( camera_->open() == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Error opening the camera" << std::endl;
        return;        
    }
    
    //configure image acquisition according yaml inputs
    if ( camera_->configure(video_mode,pixel_format) == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Error configuring the camera" << std::endl;
        return;        
    }
        
    //starts camera image acquisition
    if ( camera_->startAcquisition() == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Error starting image acquisition" << std::endl;
        return;
    }
    
    //print camera info
    camera_->printCameraInfo();
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
    BflyCamera::pixelFormat pxf = camera_->getPixelFormat();
    switch (pxf)
    {
        case BflyCamera::MONO8:
            image_.encoding = sensor_msgs::image_encodings::MONO8;
            break;

        case BflyCamera::RGB8:
            image_.encoding = sensor_msgs::image_encodings::RGB8;
            break;

        default: 
            image_.encoding = sensor_msgs::image_encodings::MONO8;
            break;            
    }
    
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
