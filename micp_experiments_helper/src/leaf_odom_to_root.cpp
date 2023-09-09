#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <rmagine/math/types.h>

namespace rm = rmagine;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leaf_odom_to_root");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    std::string base_frame, odom_frame_leaf, odom_frame;

    nh_p.param<std::string>("base_frame", base_frame, "base_link");
    nh_p.param<std::string>("odom_frame_leaf", odom_frame_leaf, "world");
    nh_p.param<std::string>("odom_frame",  odom_frame,  "odom");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(100.0);

    bool first_publish = true;
    ros::Time last_publish;

    std::cout << "START" << std::endl;

    while(nh.ok())
    {
        geometry_msgs::TransformStamped Tbl;

        try
        {
            Tbl = tfBuffer.lookupTransform(odom_frame_leaf, base_frame, ros::Time(0));
            
            if(first_publish || last_publish < Tbl.header.stamp)
            {
                Tbl.header.frame_id = odom_frame;

                static tf2_ros::TransformBroadcaster br;
                br.sendTransform(Tbl);

                last_publish = Tbl.header.stamp; 
                first_publish = false;
            }
        }
        catch (tf2::TransformException &ex) 
        {
            ROS_WARN("%s",ex.what());
            continue;
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}