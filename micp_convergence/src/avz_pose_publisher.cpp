#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <geometry_msgs/PoseArray.h>

namespace rm = rmagine;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avz_pose_publisher");

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ros::Publisher pose_pub = nh_p.advertise<geometry_msgs::PoseArray>("poses", 1);


    std::vector<rm::Transform> Tsms = {
         {rm::EulerAngles{0.0, 0.0, 0.0}, {5.5, -14.0, 0.5}} //  1 (top - right)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {5.5, -10.6, 0.5}} //  2 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {5.5,  -7.3, 0.5}} //  3 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {5.5,  -4.2, 0.5}} //  4 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {4.5,   0.7, 0.5}} //  5 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {4.0,   5.4, 0.5}} //  6 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {4.5,   8.4, 0.5}} //  7 (top)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {4.5,  12.0, 0.5}} //  8 (top - left)

        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, 12.6, 0.5}} //  9 (bottom - left)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4,  9.6, 0.5}} // 10 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, 6.7, 0.5}} // 11 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, 3.0, 0.5}} // 12 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, -0.6, 0.5}} // 13 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, -4.2, 0.5}} // 14 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, -7.7, 0.5}} // 15 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, -10.8, 0.5}} // 16 (bottom)
        ,{rm::EulerAngles{0.0, 0.0, 0.0}, {-4.4, -13.1, 0.5}} // 17 (bottom)
    };

    // rm::Transform T1 = {rm::EulerAngles{0.0, 0.0, 0.0}, {5.47, -10.6, 0.0}};

    std::cout << Tsms[0] << std::endl;

    geometry_msgs::PoseArray poses;
    poses.header.frame_id = "map";
    for(auto Tsm : Tsms)
    {
        geometry_msgs::Pose pose;
        pose.position.x = Tsm.t.x;
        pose.position.y = Tsm.t.y;
        pose.position.z = Tsm.t.z;
        pose.orientation.x = Tsm.R.x;
        pose.orientation.y = Tsm.R.y;
        pose.orientation.z = Tsm.R.z;
        pose.orientation.w = Tsm.R.w;
        poses.poses.push_back(pose);
    }
    

    ros::Rate r(10);

    while(ros::ok())
    {
        poses.header.stamp = ros::Time::now();
        pose_pub.publish(poses);

        r.sleep();
        ros::spinOnce();
    }

    return 0;
}
