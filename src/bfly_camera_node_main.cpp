
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

    switch(bfly.runMode())
    {
        case SERVER: //just spin to give service             
            ros::spin(); 
            break;
            
        case PUBLISHER: //publish loop
            while ( ros::ok() ) 
            {
                //execute pending callbacks
                ros::spinOnce(); 
                    
                //just publish the cloud
                bfly.publish(); 
                    
                //relax to fit output rate
                loop_rate.sleep();            
            }
            break;
            
        default:
            break;
    }

    //exit program
    return 0;
}
