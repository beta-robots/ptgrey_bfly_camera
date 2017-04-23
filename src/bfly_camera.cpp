#include "bfly_camera.h"

namespace BflyCamera
{
    
Device::Device()
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
    std::cout << std::endl << "BflyCamera::Device::Device(): Number of cameras detected: " << numCameras << std::endl;
    if ( numCameras < 1 )
    {
        std::cout << "BflyCamera::Device::Device() ERROR: No camera detected. EXIT. " << std::endl;
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
        std::cout << "BflyCamera::Device::Device() ERROR: GigE camera not found" << std::endl;
    }

    //set init flag
    init_ok_ = true;
    std::cout << "BflyCamera::Device::Device(): GigE camera found!" << std::endl;
}

Device::~Device()
{
    if ( flycap_camera_.IsConnected() ) this->close();
}

bool Device::isInitOk() const
{
    return init_ok_;
}

int Device::open()
{
    //connects to the camera
    flycap_error_ = flycap_camera_.Connect(&flycap_guid_);
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    return SUCCESS;
}

int Device::close()
{
    // Disconnects from the camera
    flycap_error_ = flycap_camera_.Disconnect();
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    return SUCCESS;    
}

int Device::configure(videoMode _v_mode, pixelFormat _px_format)
{
    FlyCapture2::GigEImageSettingsInfo flycap_image_settings_Info;
    FlyCapture2::Mode vModeFC2;
    
    //Set video mode
    switch (_v_mode)
    {
        case MODE0: vModeFC2 = FlyCapture2::MODE_0; break;
        case MODE1: vModeFC2 = FlyCapture2::MODE_1; break;
        case MODE4: vModeFC2 = FlyCapture2::MODE_4; break;
        case MODE5: vModeFC2 = FlyCapture2::MODE_5; break;
        default: 
                std::cout << "BflyCamera::Device::configure() ERROR: Unknown video mode" << std::endl; 
                return ERROR;
                break;
    }
    flycap_error_ = flycap_camera_.SetGigEImagingMode( vModeFC2 );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }

    //set pixel format and image size to maximum allowed
    flycap_error_ = flycap_camera_.GetGigEImageSettingsInfo( &flycap_image_settings_Info );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    flycap_image_settings_.offsetX = 0;
    flycap_image_settings_.offsetY = 0;
    flycap_image_settings_.height = flycap_image_settings_Info.maxHeight;
    flycap_image_settings_.width = flycap_image_settings_Info.maxWidth;
    switch (_px_format)
    {
        case MONO8: 
            flycap_image_settings_.pixelFormat = FlyCapture2::PIXEL_FORMAT_MONO8; 
            break;
        case RGB8: 
            flycap_image_settings_.pixelFormat = FlyCapture2::PIXEL_FORMAT_RGB8; 
            break;
        default:
            std::cout << "BflyCamera::Device::configure() ERROR: Unknown pixel format: " << _px_format << std::endl;
            return ERROR;
            break;
    }
    flycap_error_ = flycap_camera_.SetGigEImageSettings( &flycap_image_settings_ );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
        
    //return SUCCESS
    return SUCCESS;
}

int Device::configure(unsigned int _stream_ch)
{
      
}

int Device::startAcquisition()
{
    flycap_error_ = flycap_camera_.StartCapture(); // Start capturing images
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    std::cout << "BflyCamera::Device: Start Image Acquisition" << std::endl;
    return SUCCESS;
}

int Device::stopAcquisition()
{
    flycap_error_ = flycap_camera_.StopCapture();// Stop capturing images
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    std::cout << "BflyCamera::Device: Stop Image Acquisition" << std::endl;
    return SUCCESS;
}

double Device::getFrameRate()
{
    FlyCapture2::Property frameRate;
    
    frameRate.type = FlyCapture2::FRAME_RATE;
    flycap_error_ = flycap_camera_.GetProperty( &frameRate );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
    }
    return frameRate.absValue;
}

int Device::getCurrentImage(cv::Mat & _img)
{
    //Gets current camera buffer image
    flycap_error_ = flycap_camera_.RetrieveBuffer( &flycap_image_ );
    if (flycap_error_ != FlyCapture2::PGRERROR_OK)
    {
        flycap_error_.PrintErrorTrace();
        return ERROR;
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
    _img = opencv_image_.clone();
    
    //return success
    return SUCCESS;
}

pixelFormat Device::getPixelFormat() const
{
    unsigned int pxFormat = flycap_image_settings_.pixelFormat; 
    switch( pxFormat)
    {
        case FlyCapture2::PIXEL_FORMAT_MONO8:
            return MONO8;
            break;
            
        case FlyCapture2::PIXEL_FORMAT_RGB8:
            return RGB8;
            break;

        default:
            return MONO8; //TODO
            break;       
    }
}

void Device::printDeviceInfo()
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
            << "BflyCamera::Device: DEVICE INFORMATION" << std::endl
            << "\tCamera vendor - " << info.vendorName << std::endl
            << "\tCamera model - " << info.modelName << std::endl
            << "\tSerial number - " << info.serialNumber << std::endl
            << "\tSensor - " << info.sensorInfo << std::endl
            << "\tResolution - " << info.sensorResolution << std::endl
            << "\tFirmware version - " << info.firmwareVersion << std::endl
            << "\tFirmware build time - " << info.firmwareBuildTime << std::endl
            << "\tGigE version - " << info.gigEMajorVersion << "." << info.gigEMinorVersion << std::endl
            << "\tIP Address - " << (unsigned int)info.ipAddress.octets[0] << "." << (unsigned int)info.ipAddress.octets[1] << "." << (unsigned int)info.ipAddress.octets[2] << "." << (unsigned int)info.ipAddress.octets[3] << std::endl << std::endl;
    }
}

void Device::printImageInfo() const
{
    std::cout << "BflyCamera::Device: IMAGE INFORMATION" << std::endl   
        << "\tImage Rows: " << flycap_image_.GetRows() << std::endl
        << "\tImage Cols: " << flycap_image_.GetCols() << std::endl
        << "\tImage Stride: " << flycap_image_.GetStride() << std::endl
        << "\tImage BitsPerPixel: " << flycap_image_.GetBitsPerPixel() << std::endl
        << "\tImage DataSize (Bytes): " << flycap_image_.GetDataSize() << std::endl
        << std::endl;
}

}//end of namespace

