
//ros dependencies
#include "bfly_camera_node.h"

//node main
int main(int argc, char **argv)
{
      //init ros
      ros::init(argc, argv, "bfly_camera_node");
      
      //create ros wrapper object
      BflyCameraNode bfly;
      
      //set node loop rate
      ros::Rate loop_rate(bfly.rate());
      
      //node loop 
      while ( ros::ok() )
      {
            //execute pending callbacks
            ros::spinOnce(); 
            
            //switch according run mode
            switch(bfly.runMode())
            {
                case SERVER:
                    //nothing to do, ROS spin will do the job
                    break;
                    
                case PUBLISHER:
                    //just publish the cloud
                    bfly.publish(); 
                    break;
                    
                default:
                    break;
            }
            
            //relax to fit output rate
            loop_rate.sleep();            
      }
            
      //exit program
      return 0;
}
