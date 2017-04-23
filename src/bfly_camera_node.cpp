#include "bfly_camera_node.h"

BflyCameraNode::BflyCameraNode() :
    nh_(ros::this_node::getName()), 
    image_tp_(nh_), 
    paramsD(5,1,CV_64FC1),
    matrixK(3,3,CV_64FC1),
    matrixP(3,4,CV_64FC1), 
    calibration_file_()
{
    //local vars
    BflyCamera::videoMode video_mode; 
    BflyCamera::pixelFormat pixel_format; 
    int param_int; 
    std::ostringstream sstream; 
    std::string package_path;
    
    //Configure node according yaml params
    nh_.getParam("run_mode", param_int); this->run_mode_ = (RunMode)param_int; 
    nh_.getParam("rate", this->rate_);
    nh_.getParam("frame_name", this->camera_frame_name_);
    nh_.getParam("camera_info_file", this->camera_info_file_);
    nh_.getParam("video_mode", param_int); video_mode = (BflyCamera::videoMode)param_int;
    nh_.getParam("pixel_format", param_int); pixel_format = (BflyCamera::pixelFormat)param_int;
    
    //build calibration full file name
    package_path = ros::package::getPath("ptgrey_bfly_camera");
    sstream << package_path << "/calibration/" << camera_info_file_;
    camera_info_file_ = sstream.str();
    
    //init the image publisher
    image_publisher_ = image_tp_.advertise("image_raw", 1);
    
    //init the camera_info publisher
    camera_info_publisher_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
    
    //init servers
    image_server_ = nh_.advertiseService("image_server", &BflyCameraNode::imageServiceCallback, this);
    set_camera_info_server_ = nh_.advertiseService("set_camera_info", &BflyCameraNode::SetCameraInfoServiceCallback, this);        
    
    //constructs the camera object interfacing with HW
    camera_ = new BflyCamera::Device(); 
    
    //open/connect to the HW device
    if ( camera_->open() == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): ERROR opening the camera" << std::endl;
        return;        
    }
    
    //configure image acquisition according yaml inputs
    if ( camera_->configure(video_mode,pixel_format) == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): ERROR configuring the camera" << std::endl;
        return;        
    }
        
    //starts camera image acquisition
    if ( camera_->startAcquisition() == BflyCamera::ERROR )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): ERROR starting image acquisition" << std::endl;
        return;
    }
    
    //sets calibration data from calibration file
    if ( setCalibrationFromFile() )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): Calibration data load from file. camera_info topic will publish these data." << std::endl;
    }
    else //some error occurred
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): WARNING! Calibration file not found. camera_info topic will publish incorrect data." << std::endl;
    }
    
    //print camera info
    camera_->printDeviceInfo();
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

    //fill the camera info message
    camera_info_msg_.header.seq ++;
    camera_info_msg_.header.stamp = ts;
    camera_info_msg_.header.frame_id = camera_frame_name_; 
    camera_info_msg_.height = image_.image.rows; 
    camera_info_msg_.width = image_.image.cols;     
    camera_info_msg_.distortion_model = "plumb_bob"; 
    camera_info_msg_.D.resize(5); 
    for(unsigned int ii=0; ii<5; ii++) camera_info_msg_.D[ii] = paramsD.at<double>(ii);
    for(unsigned int ii=0; ii<3; ii++)
        for(unsigned int jj=0; jj<3; jj++) camera_info_msg_.K[ii*3+jj] = matrixK.at<double>(ii,jj);
    for(unsigned int ii=0; ii<3; ii++)
        for(unsigned int jj=0; jj<4; jj++) camera_info_msg_.P[ii*4+jj] = matrixP.at<double>(ii,jj);        
    camera_info_msg_.R[0] = 1.; //R is the identity (no rotation)
    camera_info_msg_.R[4] = 1.; //R is the identity (no rotation)
    camera_info_msg_.R[8] = 1.; //R is the identity (no rotation)
    camera_info_msg_.binning_x = 1;
    camera_info_msg_.binning_y = 1;
    camera_info_msg_.roi.width = 0;
    camera_info_msg_.roi.height = 0; 
    camera_info_msg_.roi.do_rectify = false; 

    //publish the image    
    image_publisher_.publish(image_.toImageMsg());        

    //publish the camera info
    camera_info_publisher_.publish(camera_info_msg_);
}

//seee http://docs.opencv.org/2.4/modules/core/doc/xml_yaml_persistence.html
bool BflyCameraNode::setCalibrationFromFile()
{
    //open calibration data file
    calibration_file_.open(camera_info_file_, cv::FileStorage::READ); 
    if ( calibration_file_.isOpened() )
    {
        calibration_file_["paramsD"] >> paramsD;
        calibration_file_["matrixK"] >> matrixK;
        calibration_file_["matrixP"] >> matrixP;
        calibration_file_.release(); 
        return true; 
    }
    else //file not opened
    {
        calibration_file_.release(); 
        return false; 
    }
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
    //Message
    std::cout << "BflyCameraNode::BflyCameraNode(): Setting new camera info parameters " << std::endl; 
    
    //open an openCV-yaml file
    calibration_file_.open(camera_info_file_, cv::FileStorage::WRITE); 
    if ( ! calibration_file_.isOpened() )
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): ERROR opening file to store calibration data: " << camera_info_file_ << " not found." << std::endl;
        _reply.success = false; 
        _reply.status_message = -1; 
        return false;
    }
    
    //check distortion model. Only plumb_bob implemented with 5 parameters
    if ( _request.camera_info.distortion_model != "plumb_bob")
    {
        std::cout << "BflyCameraNode::BflyCameraNode(): ERROR, only plumb_bob distortion model allowed (5 parameters)" << std::endl;
        _reply.success = false; 
        _reply.status_message = -1; 
        return false;        
    }
    
    //Set class member data
    paramsD = (cv::Mat_<double>(5,1) << _request.camera_info.D[0],
                                        _request.camera_info.D[1],
                                        _request.camera_info.D[2],
                                        _request.camera_info.D[3],
                                        _request.camera_info.D[4]);
    matrixK = (cv::Mat_<double>(3,3) << _request.camera_info.K[0],_request.camera_info.K[1],_request.camera_info.K[2],
                                        _request.camera_info.K[3],_request.camera_info.K[4],_request.camera_info.K[5],    
                                        _request.camera_info.K[6],_request.camera_info.K[7],_request.camera_info.K[8]);
    matrixP = (cv::Mat_<double>(3,4) << _request.camera_info.P[0],_request.camera_info.P[1],_request.camera_info.P[2],_request.camera_info.P[3],
                                        _request.camera_info.P[4],_request.camera_info.P[5],_request.camera_info.P[6],_request.camera_info.P[7],
                                        _request.camera_info.P[8],_request.camera_info.P[9],_request.camera_info.P[10],_request.camera_info.P[11]);
        
    //Fill the calibration file
    time_t raw_time; 
    time(&raw_time);
    calibration_file_ << "date" << asctime(localtime(&raw_time));   
    calibration_file_ << "paramsD" << paramsD;
    calibration_file_ << "matrixK" << matrixK;
    calibration_file_ << "matrixP" << matrixP;
    
    //save the file
    calibration_file_.release();
        
    //reply the service
    _reply.success = true; 
    _reply.status_message = 1; 
    
    //return
    return true; 
}
    