#include "bfly_camera_node.h"

BflyCameraNode::BflyCameraNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_), 
    matD(5,1,CV_64FC1),
    matK(3,3,CV_64FC1),
    matP(3,4,CV_64FC1)
{
    //local vars
    BflyCamera::videoMode video_mode; 
    BflyCamera::pixelFormat pixel_format; 
    int param_int; 
    
    //Configure node according yaml params
    nh_.getParam("run_mode", param_int); this->run_mode_ = (RunMode)param_int; 
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->camera_frame_name_);
    nh_.getParam("camera_info_file", this->camera_info_file_);
    nh_.getParam("video_mode", param_int); video_mode = (BflyCamera::videoMode)param_int;
    nh_.getParam("pixel_format", param_int); pixel_format = (BflyCamera::pixelFormat)param_int;
    
    //init the image publisher
    image_publisher_ = image_tp_.advertise("image_raw", 1);
    
    //init the camera_info publisher
    camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    
    //init servers
    image_server_ = nh_.advertiseService("image_server", &BflyCameraNode::imageServiceCallback, this);
    set_camera_info_server_ = nh_.advertiseService("set_camera_info_server", &BflyCameraNode::SetCameraInfoServiceCallback, this);        
    
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
    
    //fill the camera info
    //TODO
    camera_info_msg_.header.seq ++;
    camera_info_msg_.header.stamp = ts;
    camera_info_msg_.header.frame_id = camera_frame_name_; 
//     camera_info_msg_.height = ...
//     camera_info_msg_.width = ...
//     ...
    
    //publish the image    
    image_publisher_.publish(image_.toImageMsg());        
    
    //publish the camera info
    camera_info_publisher_.publish(camera_info_msg_);
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

bool BflyCameraNode::SetCameraInfoServiceCallback(
            sensor_msgs::SetCameraInfo::Request  & _request, 
            sensor_msgs::SetCameraInfo::Response & _reply)
{
    //create and open an openCV file
    cv::FileStorage file(camera_info_file_, cv::FileStorage::WRITE); 
    if ( ! file.isOpened() )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Error opening file to store calibration data: " << camera_info_file_ << " not found." << std::endl;
        _reply.success = false; 
        _reply.status_message = -1; 
        return false;
    }
    
    //Fill the file with message content
    time_t raw_time; 
    time(&raw_time);
    file << "date" << asctime(localtime(&raw_time));    
    matD = (cv::Mat_<double>(5,1) << _request.camera_info.D[0],
                                     _request.camera_info.D[1],
                                     _request.camera_info.D[2],
                                     _request.camera_info.D[3],
                                     _request.camera_info.D[4]);
    file << "paramsD" << matD;
    matK = (cv::Mat_<double>(3,3) << _request.camera_info.K[0],_request.camera_info.K[1],_request.camera_info.K[2],
                                     _request.camera_info.K[3],_request.camera_info.K[4],_request.camera_info.K[5],    
                                     _request.camera_info.K[6],_request.camera_info.K[7],_request.camera_info.K[8]);
    file << "matrixK" << matK;
    matP = (cv::Mat_<double>(3,4) << _request.camera_info.R[0],_request.camera_info.R[1],_request.camera_info.R[2],_request.camera_info.R[3],
                                     _request.camera_info.R[4],_request.camera_info.R[5],_request.camera_info.R[6],_request.camera_info.R[7],
                                     _request.camera_info.R[8],_request.camera_info.R[9],_request.camera_info.R[10],_request.camera_info.R[11]);
    file << "matrixP" << matP;
    
    //save the file
    file.release();
        
    //reply the service
    _reply.success = true; 
    _reply.status_message = 1; 
    
    //return
    return true; 
}
    