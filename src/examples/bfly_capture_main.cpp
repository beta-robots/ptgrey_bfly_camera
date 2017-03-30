
#include <sstream>
#include <string>
#include <vector>
#include "../bfly_camera.h"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

//constants
const int numUserEntries = 4;

// print command line usage
void printUsage()
{
      std::cout
            << "Execute as: bfly_capture [numImages videoMode pixelFormat folderName]" << std::endl 
            << "   numImages is a positive integer" << std::endl
            << "   videoMode is one of the following video modes: 0,1,4,5" << std::endl
            << "   pixelFormat is 0->MONO8 or 1->RGB8" << std::endl
            << "   folderName where the *.jpg frames will be saved (optional)" << std::endl << std::endl;
}

//main loop 
int main(int argc, char *argv[])
{            
      //variable declaration
      unsigned int ii;
      int retV = 0; 
      unsigned int numImages;
      BflyCamera::bfly_videoMode vMode;
      BflyCamera::bfly_pixelFormat pxFormat;
      bool saveFrames;
      std::string folderName;
      std::ostringstream fullFileName;
      std::vector<int> imwriteParams;
      
      //prompt header
      std::cout << std::endl << "**************** BFLY ACQUISITION PROCESS ******************" << std::endl;
      
      //check user entries (there is one optional argument)
      if ( (argc < numUserEntries) || (argc > numUserEntries+1) ) 
      {
            std::cout  << "EXIT due to missing or too many parameters" << std::endl;
            printUsage();
            return BflyCamera::ERROR;
      }
      
      //get user entries
      numImages = atoi(argv[1]);
      vMode = (BflyCamera::bfly_videoMode)atoi(argv[2]);
      if ( (vMode < 0) || (vMode == 2) || (vMode == 3) || (vMode > 5) )
      {
            std::cout << "EXIT due to unavailable Video Mode." << std::endl;                  
            printUsage();
            return BflyCamera::ERROR;
      }
      pxFormat = (BflyCamera::bfly_pixelFormat)atoi(argv[3]);
      if ( (pxFormat < 0) || (pxFormat > 1) )
      {
            std::cout << "EXIT due to unavailable Pixel Format." << std::endl;                  
            printUsage();
            return BflyCamera::ERROR;
      }
      if (argc == 5) //if optional argument is provided (folder name where frames will be saved)
      {
            saveFrames = true;
            folderName = std::string(argv[4]);
            if ( folderName.at(folderName.length()-1) != '/') folderName.push_back('/');
            imwriteParams.push_back(CV_IMWRITE_JPEG_QUALITY);
            imwriteParams.push_back(100);
      }
      else 
      {
            saveFrames = false;
      }
      
      //print user entries
      std::cout 
            << "User entries are:" << std::endl
            << "  Num Images to acquire: " << numImages << std::endl
            << "  Video Mode: " << vMode << std::endl
            << "  Pixel Format: " << pxFormat << std::endl;
      if (saveFrames) std::cout << "  Folder Name: " << folderName << std::endl;
      else std::cout << "  Folder not provided. Frames will not be saved" << std::endl;
            
      //create camera and image objects
      BflyCamera::Device camera;
      cv::Mat img;
                 
      //check if camera object has been built correctly. It mainly checks if there is a camera available
      if( camera.isInitOk() )
      {                  
            //opens & configures camera
            retV += camera.open();
            retV += camera.configure(vMode, pxFormat);
            camera.printCameraInfo();
            if (retV != 2*BflyCamera::SUCCESS) return BflyCamera::ERROR;
            
            //set OpenCV Window
            cv::namedWindow("rawImage", CV_WINDOW_AUTOSIZE);
            cv::moveWindow("rawImage",50,50);
                        
            //starts acquisition
            if ( camera.startAcquisition() == BflyCamera::ERROR ) return BflyCamera::ERROR;
            
            //acquisition loop            
            for (ii=0; ii<numImages; ii++)
            {
                  //prints useful information, each 10 frames
                  if (ii%10 == 0) 
                  {
                        std::cout << "Frame rate: " << camera.getFrameRate() << std::endl;
                        camera.printImageInfo();
                  }
                  
                  //Gets image data in openCV format
                  camera.getCurrentImage(img);
                  
                  // show current image
                  cv::imshow("rawImage", img);
                  
                  //saves current image
                  if (saveFrames)
                  {
                        fullFileName.str("");
                        fullFileName.clear();
                        fullFileName << folderName;
                        fullFileName.fill('0');
                        fullFileName.width(6);
                        fullFileName << ii;
                        fullFileName.clear();
                        fullFileName << ".jpg";
                        cv::imwrite(fullFileName.str(), img, imwriteParams);
                  }
                  
                  // wait for a key during 1ms
                  cv::waitKey(1); 
            }
            
            camera.stopAcquisition();
            camera.close();
      }
      
      return BflyCamera::SUCCESS;
}
