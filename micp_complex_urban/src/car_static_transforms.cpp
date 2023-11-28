#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rmagine/math/types.h>
#include <rmagine/util/prints.h>
#include <unordered_map>


namespace rm = rmagine;

geometry_msgs::Transform to_ros(rm::Transform T)
{
    geometry_msgs::Transform Tros;
    Tros.translation.x = T.t.x;
    Tros.translation.y = T.t.y;
    Tros.translation.z = T.t.z;
    Tros.rotation.x = T.R.x;
    Tros.rotation.y = T.R.y;
    Tros.rotation.z = T.R.z;
    Tros.rotation.w = T.R.w;
    return Tros;
}


std::vector<geometry_msgs::TransformStamped> load_static_transforms(
    ros::NodeHandle nh)
{
    std::vector<geometry_msgs::TransformStamped> ret;

    XmlRpc::XmlRpcValue transform_list_xml;
    nh.getParam("calibrations", transform_list_xml);
    ROS_ASSERT(transform_list_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < transform_list_xml.size(); ++i) 
    {
        auto transform_xml = transform_list_xml[i];
        ROS_ASSERT(transform_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        
        std::cout << "Loading transform " << i+1 << "/" << transform_list_xml.size() << std::endl;

        geometry_msgs::TransformStamped Tros;
        Tros.header.frame_id = static_cast<std::string>(transform_xml["to"]);
        Tros.child_frame_id = static_cast<std::string>(transform_xml["from"]);
        
        std::cout << "- from: " << Tros.child_frame_id << std::endl;
        std::cout << "- to: "<< Tros.header.frame_id << std::endl;

        auto transform_transform_xml = transform_xml["transform"];

        rm::Transform T;
        if(transform_transform_xml.size() == 6)
        {
            // x, y, z, roll, pitch, yaw
            T.t.x = static_cast<double>(transform_transform_xml[0]);
            T.t.y = static_cast<double>(transform_transform_xml[1]);
            T.t.z = static_cast<double>(transform_transform_xml[2]);
            rm::EulerAngles e;
            e.roll = static_cast<double>(transform_transform_xml[3]);
            e.pitch = static_cast<double>(transform_transform_xml[4]);
            e.yaw = static_cast<double>(transform_transform_xml[5]);
            T.R = e;
        } else if(transform_transform_xml.size() == 7) {
            // x, y, z, qx, qy, qz, qw
            T.t.x = static_cast<double>(transform_transform_xml[0]);
            T.t.y = static_cast<double>(transform_transform_xml[1]);
            T.t.z = static_cast<double>(transform_transform_xml[2]);
            T.R.x = static_cast<double>(transform_transform_xml[3]);
            T.R.y = static_cast<double>(transform_transform_xml[4]);
            T.R.z = static_cast<double>(transform_transform_xml[5]);
            T.R.w = static_cast<double>(transform_transform_xml[6]);
        } else {
            std::cout << "ERROR: Only 6 or 7 pose elements are allowed. given: " << transform_transform_xml.size() << std::endl;
            continue;
        }

        std::cout << "- transform: " << T << std::endl;
        Tros.transform = to_ros(T);
        std::cout << "- rot: " 
                    << Tros.transform.rotation.x << ", " 
                    << Tros.transform.rotation.y << ", " 
                    << Tros.transform.rotation.z << ", " 
                    << Tros.transform.rotation.w << std::endl;

        // auto transform_to
        ret.push_back(Tros);
    }

    return ret;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_static_transforms");

    static tf2_ros::StaticTransformBroadcaster static_broadcaster;

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    auto transforms = load_static_transforms(nh_p);

    for(auto transform : transforms)
    {
        std::cout << "BR" << std::endl;
        transform.header.stamp = ros::Time::now();
        static_broadcaster.sendTransform(transform);
    }

    std::cout << "SPIN" << std::endl;
    ros::spin();

    return 0;
}
