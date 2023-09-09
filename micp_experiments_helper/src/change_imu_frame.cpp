#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Publisher pub_imu;
std::string frame;

void imu_cb(sensor_msgs::Imu msg)
{
    msg.header.frame_id = frame;
    pub_imu.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "change_imu_frame");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    if(!nh_p.getParam("frame", frame))
    {
        ROS_ERROR("NO 'frame' GIVEN");
        return 0;
    }

    pub_imu = nh.advertise<sensor_msgs::Imu>("imu_out", 1);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>("imu_in", 1, imu_cb);
    
    ros::spin();

    return 0;
}