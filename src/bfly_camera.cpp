#include "bflyCamera.h"

BflyCamera::BflyCamera()
{
    FlyCapture2::BusManager busMgr;
    FlyCapture2::InterfaceType interfaceType;
    unsigned int numCameras;
    
    //reset inti flag
    init_ok_ = false;
    
    //get number of cameras available
    flycap_error_ = busMgr.GetNumOfCameras(&numCameras);
    if (flycap_error_ != FlyCapture2::PGRERROR_OK) 
    {
        flycap_error_.PrintErrorTrace();
        return;
    }
    std::cout << std::endl << "Number of cameras detected: " << numCameras << std::endl;
    if ( numCameras < 1 )
    {
        std::cout << "Insufficient number of cameras... exiting" << std::endl;
        return;
    }

    //Gets id of the first camera found (camera 0)
    flycap_error_ = busMgr.GetCameraFromIndex(0, &flycap_guid_);
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return;
    }
    
    //check if it is a GigE camera
    flycap_error_ = busMgr.GetInterfaceTypeFromGuid( &flycap_guid_, &interfaceType );
    if ( flycap_error_ != FlyCapture2::PGRERROR_OK )
    {
        flycap_error_.PrintErrorTrace();
        return;
    }
    if ( interfaceType != FlyCapture2::INTERFACE_GIGE )
    {
        std::cout << "Error: GigE camera not found" << std::endl;
    }

    //set init flag
    init_ok_ = true;
    std::cout << "GigE camera found!" << std::endl;
}

BflyCamera::~BflyCamera()
{
    if ( flycap_camera_.IsConnected() ) this->close();
}

bool BflyCamera::isInitOk() const
{
    return init_ok_;
}

int BflyCamera::open()
{
    //connects to the camera
    flycap_error_ = flycap_camera_.Connect(&flycap_guid_);
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    return BFLY_SUCCESS;
}

int BflyCamera::close()
{
    // Disconnects from the camera
    flycap_error_ = flycap_camera_.Disconnect();
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    return BFLY_SUCCESS;    
}

int BflyCamera::configure(const bfly_videoMode vMode, const bfly_pixelFormat pxFormat)
{
    FlyCapture2::GigEImageSettingsInfo flycap_image_settings_Info;
    FlyCapture2::Mode vModeFC2;
    
    //Set video mode
    switch (vMode)
    {
        case MODE0: vModeFC2 = FlyCapture2::MODE_0; break;
        case MODE1: vModeFC2 = FlyCapture2::MODE_1; break;
        case MODE4: vModeFC2 = FlyCapture2::MODE_4; break;
        case MODE5: vModeFC2 = FlyCapture2::MODE_5; break;
        default: 
                std::cout << "BflyCamera::configure(): Unknown video mode" << std::endl; 
                return BFLY_ERROR;
                break;
    }
    flycap_error_ = flycap_camera_.SetGigEImagingMode( vModeFC2 );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }

    //set pixel format and image size to maximum allowed
    flycap_error_ = flycap_camera_.GetGigEImageSettingsInfo( &flycap_image_settings_Info );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    flycap_image_settings_.offsetX = 0;
    flycap_image_settings_.offsetY = 0;
    flycap_image_settings_.height = flycap_image_settings_Info.maxHeight;
    flycap_image_settings_.width = flycap_image_settings_Info.maxWidth;
    switch (pxFormat)
    {
        case MONO8: flycap_image_settings_.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8; break;
        case RGB8: flycap_image_settings_.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8; break;
        default:
                std::cout << "BflyCamera::configure(): Unknown pixel format: " << pxFormat << std::endl;
                return BFLY_ERROR;
                break;
    }
    flycap_error_ = flycap_camera_.SetGigEImageSettings( &flycap_image_settings_ );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
        
    //return SUCCESS
    return BFLY_SUCCESS;
}

int BflyCamera::configure(const unsigned int streamCh)
{
      
}

int BflyCamera::startAcquisition()
{
    flycap_error_ = flycap_camera_.StartCapture(); // Start capturing images
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    std::cout << "Start Image Acquisition" << std::endl;
    return BFLY_SUCCESS;
}

int BflyCamera::stopAcquisition()
{
    flycap_error_ = flycap_camera_.StopCapture();// Stop capturing images
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    std::cout << "Stop Image Acquisition" << std::endl;
    return BFLY_SUCCESS;
}

double BflyCamera::getFrameRate()
{
    FlyCapture2::Property frameRate;
    
    frameRate.type = FlyCapture2::FRAME_RATE;
    flycap_error_ = flycap_camera_.GetProperty( &frameRate );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    return frameRate.absValue;
}

int BflyCamera::getCurrentImage(cv::Mat & img)
{
    //Gets current camera buffer image
    flycap_error_ = flycap_camera_.RetrieveBuffer( &flycap_image_ );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return BFLY_ERROR;
    }
    
    //do conversion from FlyCapture2::Image to cv::Mat      
    if ( flycap_image_settings_.pixelFormat == FlyCapture2::PIXEL_FORMAT_MONO8 )
    {
        opencv_image_.create(flycap_image_.GetRows(),flycap_image_.GetCols(),CV_8UC1);
    }
    if ( flycap_image_settings_.pixelFormat == FlyCapture2::PIXEL_FORMAT_RGB8 ) 
    {
        opencv_image_.create(flycap_image_.GetRows(),flycap_image_.GetCols(),CV_8UC3);
    }
    opencv_image_.data = flycap_image_(0,0);//set cv data pointer to flycap_image_ address
    
    //clone to external argument image
    img = opencv_image_.clone();
    
    //return success
    return BFLY_SUCCESS;
}

void BflyCamera::printCameraInfo()
{
    // Get the camera information
    FlyCapture2::CameraInfo info;
    flycap_error_ = flycap_camera_.GetCameraInfo(&info);
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return;
    }       
    else
    {
        std::cout << std::endl
            << "******** CAMERA INFORMATION ********" << std::endl
            << "Camera vendor - " << info.vendorName << std::endl
            << "Camera model - " << info.modelName << std::endl
            << "Serial number - " << info.serialNumber << std::endl
            << "Sensor - " << info.sensorInfo << std::endl
            << "Resolution - " << info.sensorResolution << std::endl
            << "Firmware version - " << info.firmwareVersion << std::endl
            << "Firmware build time - " << info.firmwareBuildTime << std::endl
            << "GigE version - " << info.gigEMajorVersion << "." << info.gigEMinorVersion << std::endl
            << "IP Address - " << (unsigned int)info.ipAddress.octets[0] << "." << (unsigned int)info.ipAddress.octets[1] << "." << (unsigned int)info.ipAddress.octets[2] << "." << (unsigned int)info.ipAddress.octets[3] << std::endl << std::endl;
    }
}

void BflyCamera::printImageInfo() const
{
    std::cout   << "  Image Rows: " << flycap_image_.GetRows() << std::endl
        << "  Image Cols: " << flycap_image_.GetCols() << std::endl
        << "  Image Stride: " << flycap_image_.GetStride() << std::endl
        << "  Image BitsPerPixel: " << flycap_image_.GetBitsPerPixel() << std::endl
        << "  Image DataSize (Bytes): " << flycap_image_.GetDataSize() << std::endl
        << std::endl;
}
