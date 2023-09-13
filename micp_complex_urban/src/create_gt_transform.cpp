#include <ros/ros.h>

#include <fstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_gt_transform");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::string gt_filename;

    if(!nh_p.getParam("gt_file", gt_filename))
    {
        std::cout << "param ~gt_file must be provided!" << std::endl;
        return 0;
    }
    
    

    



    return 0;
}