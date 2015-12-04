#include "imu_tool.h"


int main(int argc, char **argv)
{

       ros::init(argc, argv, "imu_tool");
       ImuTool im;

       ros::spin();
 
      return (0);
}
