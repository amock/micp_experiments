#include <ros/ros.h>
#include <ros/package.h>

#include <iostream>
#include <iomanip>
#include <rmagine/util/synthetic.h>
#include <rmagine/util/StopWatch.hpp>
#include <rmagine/map/embree/embree_shapes.h>
#include <rmagine/types/sensors.h>
#include <rmagine/math/types.h>
#include <rmagine/math/math.h>
#include <rmagine/util/prints.h>
#include <random>

#include <rmcl/correction/SphereCorrectorEmbree.hpp>

#include <fstream>

#include <Eigen/Dense>

#include <fstream>



namespace rm = rmagine;

struct CubeGridSettings
{
    int start_x = -1;
    int start_y = -1;
    int size_x = 3;
    int size_y = 3;
    float dist_between_cubes = 0.1;
    bool use_instances = false;
};

rm::EmbreeMapPtr make_cube_grid_map(
    CubeGridSettings settings = {})
{
    // Create a map out of cubes looking like this:
    //
    //  ------- ------- -------
    //  |     | |     | |     |
    //  |     | |     | |     |
    //  ------- ------- -------
    //  ------- ------- -------
    //  |     | |     | |     |
    //  |     | |     | |     |
    //  ------- ------- -------
    //  ------- ------- -------
    //  |     | |     | |     |
    //  |     | |     | |     |
    //  ------- ------- -------

    rm::EmbreeMeshPtr cube_mesh_for_instance;

    if(settings.use_instances)
    {
        cube_mesh_for_instance = std::make_shared<rm::EmbreeCube>();
        cube_mesh_for_instance->commit();
    }

    rm::EmbreeScenePtr cube_scene = std::make_shared<rm::EmbreeScene>();

    float shift_dist = 1.0 + settings.dist_between_cubes;
    // create 3x3 grid of cube instances
    for(int i=settings.start_x; i<settings.start_x + settings.size_x; i++)
    {
        for(int j=settings.start_y; j<settings.start_y + settings.size_y; j++)
        {
            rm::Transform T = rm::Transform::Identity();
            T.t.x = static_cast<float>(i) * shift_dist;
            T.t.y = static_cast<float>(j) * shift_dist;
            
            if(settings.use_instances)
            {
                rm::EmbreeInstancePtr cube_inst = cube_mesh_for_instance->instantiate();
                cube_inst->setTransform(T);
                cube_inst->apply();
                cube_inst->commit();
                cube_scene->add(cube_inst);
            } else {
                rm::EmbreeMeshPtr cube_mesh = std::make_shared<rm::EmbreeCube>();
                cube_mesh->setTransform(T);
                cube_mesh->apply();
                cube_mesh->commit();
                cube_scene->add(cube_mesh);
            }   
        }
    }

    cube_scene->commit();
    return std::make_shared<rm::EmbreeMap>(cube_scene);
}

// TODO
rm::EmbreeMapPtr load_avz_map()
{
    rm::EmbreeMapPtr ret;
    std::cout << "Try to find uos_tools/uos_gazebo_worlds" << std::endl;
    std::string path_to_uos_tools = ros::package::getPath("uos_gazebo_worlds");

    if(path_to_uos_tools == "")
    {
        std::cout << "-> Could not find uos_tools. Download it from 'https://github.com/uos/uos_tools' and put it into your workspace." << std::endl;
        return ret;
    }

    std::string path_to_avz_map = path_to_uos_tools + "/Media/models/avz_neu.dae";


    return rm::import_embree_map(path_to_avz_map);
}


// rm::CorrectionResults correct

rmcl::CorrectionResults<rm::RAM> correct_embree_p2l(
    rm::EmbreeMapPtr map,
    rm::MemoryView<float> ranges,
    rm::SphericalModel model,
    rm::MemoryView<rm::Transform> Tbms
)
{
    rm::Transform Tsb = rm::Transform::Identity();


    rmcl::CorrectionResults<rm::RAM> res;
    res.Ncorr.resize(Tbms.size());
    res.Tdelta.resize(Tbms.size());

    float max_distance = 0.5;


    #pragma omp parallel for default(shared) if(Tbms.size() > 4)
    for(size_t pid=0; pid<Tbms.size(); pid++)
    {
        const rmagine::Transform Tbm = Tbms[pid];
        const rmagine::Transform Tsm = Tbm * Tsb;
        const rmagine::Transform Tms = ~Tsm;

        const unsigned int glob_shift = pid * model.size();

        rm::Vector Dmean = {0.0, 0.0, 0.0};
        rm::Vector Mmean = {0.0, 0.0, 0.0};
        unsigned int Ncorr = 0;
        rm::Matrix3x3 C;
        C.setZeros();

        for(unsigned int vid = 0; vid < model.getHeight(); vid++)
        {
            // #pragma omp parallel for default(shared) reduction(+:C_inner,Ncorr_inner)
            for(unsigned int hid = 0; hid < model.getWidth(); hid++)
            {
                const unsigned int loc_id = model.getBufferId(vid, hid);
                const unsigned int glob_id = glob_shift + loc_id;

                const float range_real = ranges[loc_id];
                if(range_real < model.range.min 
                    || range_real > model.range.max)
                {
                    continue;
                }

                const rm::Vector ray_orig_s = model.getOrigin(vid, hid);
                const rm::Vector ray_dir_s = model.getDirection(vid, hid);

                const rm::Vector pscan_s = ray_orig_s + ray_dir_s * range_real;
                const rm::Vector pscan_m = Tsm * pscan_s;

                const rm::Vector pmesh_m = map->closestPoint(pscan_m);
                const rm::Vector pmesh_s = Tms * pmesh_m;

                const float distance = (pmesh_s - pscan_s).l2norm();

                if(distance < max_distance)
                {
                    const rm::Vector pscan_b = Tsb * pscan_s;
                    const rm::Vector pmesh_b = Tsb * pmesh_s;

                    {
                        const float N_1 = static_cast<float>(Ncorr);
                        const float N = static_cast<float>(Ncorr + 1);
                        const float w1 = N_1/N;
                        const float w2 = 1.0/N;

                        const rm::Vector d_mean_old = Dmean;
                        const rm::Vector m_mean_old = Mmean;

                        const rm::Vector d_mean_new = d_mean_old * w1 + pscan_b * w2; 
                        const rm::Vector m_mean_new = m_mean_old * w1 + pmesh_b * w2;

                        auto P1 = (pmesh_b - m_mean_new).multT(pscan_b - d_mean_new);
                        auto P2 = (m_mean_old - m_mean_new).multT(d_mean_old - d_mean_new);

                        // write
                        Dmean = d_mean_new;
                        Mmean = m_mean_new;
                        C = C * w1 + P1 * w2 + P2 * w1;
                        Ncorr = Ncorr + 1;
                    }
                }
            }
        }

        res.Ncorr[pid] = Ncorr;

        if(Ncorr > 0)
        {
            rm::Matrix3x3 U, V, S;

            { // the only Eigen code left
                const Eigen::Matrix3f* Ceig = reinterpret_cast<const Eigen::Matrix3f*>(&C);
                Eigen::Matrix3f* Ueig = reinterpret_cast<Eigen::Matrix3f*>(&U);
                Eigen::Matrix3f* Veig = reinterpret_cast<Eigen::Matrix3f*>(&V);

                Eigen::JacobiSVD<Eigen::Matrix3f> svd(Ceig[0], Eigen::ComputeFullU | Eigen::ComputeFullV);
                Ueig[0] = svd.matrixU();
                Veig[0] = svd.matrixV();
            }

            S.setIdentity();
            if(U.det() * V.det() < 0)
            {
                S(2, 2) = -1;
            }

            res.Tdelta[pid].R.set( U * S * V.transpose() );
            res.Tdelta[pid].t = Mmean - res.Tdelta[pid].R * Dmean;
        } else {
            res.Tdelta[pid].setIdentity();
        }
    }

    

    return res;
}

struct HoursMinutesSeconds
{
    int days;
    int hours;
    int minutes;
    int seconds;
};

std::ostream& operator<<(std::ostream& os, const HoursMinutesSeconds& dt)
{
    os << dt.hours << "h, " << dt.minutes << "m, " << dt.seconds << "s";
    return os;
}

HoursMinutesSeconds to_time(double seconds)
{
    HoursMinutesSeconds ret;
    ret.seconds = seconds;
    
    ret.minutes = ret.seconds / 60;
    ret.seconds = ret.seconds % 60;
    
    ret.hours = ret.minutes / 60;
    ret.minutes = ret.minutes % 60;
    
    ret.days = ret.hours / 24;
    ret.hours = ret.hours % 24;
    
    return ret;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "micp_convergence");
    std::cout << "MICP Convergence Test" << std::endl;

    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    // PARAMS
    // 0: 3x3 cube grid
    // 1: AVZ
    int map_type = 1;
    // rm::Transform T_dest = rm::Transform::Identity();

    std::vector<rm::Transform> sensor_poses = {
        rm::Transform::Identity()
    };
    
    
    // Distribution type
    // 0: uniform circle
    // 1: uniform square: radius is half a side length. sample_radius=0.5 for a 1x1 square
    int sample_dist_type = 0;
    int sample_seed = 42;
    double sample_radius_min = 0.0;
    double sample_radius_inc = 0.1;
    int sample_radius_steps = 11;
    int Nposes = 100;
    int skip_poses = 0;

    int Nruns = 50;
    double dist_converged = 0.01;

    std::string evalfile_name = "micp_convergence";
    

    
    
    
    // CODE START

    


    nh_p.param<int>("map", map_type, 0);
    nh_p.param<int>("sampling/distribution", sample_dist_type, 0);
    nh_p.param<int>("sampling/seed", sample_seed, 42);
    nh_p.param<double>("sampling/radius_min", sample_radius_min, 0.0);
    nh_p.param<double>("sampling/radius_inc", sample_radius_inc, 0.1);
    nh_p.param<int>("sampling/radius_steps", sample_radius_steps, 11);
    nh_p.param<int>("sampling/poses", Nposes, 100);
    nh_p.param<int>("optimizer/iterations", Nruns, 50);
    nh_p.param<double>("optimizer/dist_converged", dist_converged, 0.01);

    nh_p.param<int>("skip_poses", skip_poses, 0);

    nh_p.param<std::string>("evaluation_file", evalfile_name, "micp_convergence");
    
    XmlRpc::XmlRpcValue sensor_poses_xml;
    if(nh_p.getParam("sensor_poses", sensor_poses_xml))
    {
        sensor_poses.clear();
        ROS_ASSERT(sensor_poses_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < sensor_poses_xml.size(); ++i) 
        {
            ROS_ASSERT(sensor_poses_xml[i].getType() == XmlRpc::XmlRpcValue::TypeArray);

            if(sensor_poses_xml[i].size() == 6)
            {
                rm::Transform T;
                // x, y, z, roll, pitch, yaw
                T.t.x = static_cast<double>(sensor_poses_xml[i][0]);
                T.t.y = static_cast<double>(sensor_poses_xml[i][1]);
                T.t.z = static_cast<double>(sensor_poses_xml[i][2]);
                rm::EulerAngles e;
                e.roll = static_cast<double>(sensor_poses_xml[i][3]);
                e.pitch = static_cast<double>(sensor_poses_xml[i][4]);
                e.yaw = static_cast<double>(sensor_poses_xml[i][5]);
                T.R = e;
                sensor_poses.push_back(T);
            } else if(sensor_poses_xml[i].size() == 7) {
                // x, y, z, qx, qy, qz, qw
                rm::Transform T;
                T.t.x = static_cast<double>(sensor_poses_xml[i][0]);
                T.t.y = static_cast<double>(sensor_poses_xml[i][1]);
                T.t.z = static_cast<double>(sensor_poses_xml[i][2]);
                T.R.x = static_cast<double>(sensor_poses_xml[i][3]);
                T.R.y = static_cast<double>(sensor_poses_xml[i][4]);
                T.R.z = static_cast<double>(sensor_poses_xml[i][5]);
                T.R.w = static_cast<double>(sensor_poses_xml[i][6]);
                sensor_poses.push_back(T);
            }

        }
    }


    CubeGridSettings settings;
    settings.dist_between_cubes = 0.3;
    settings.use_instances = false;
    
    rm::EmbreeMapPtr map;
    
    if(map_type == 0)
    {
        map = make_cube_grid_map(settings);
    } else if(map_type == 1) {
        map = load_avz_map();
    }
    
    if(!map)
    {
        return 0;
    }

    rmcl::SphereCorrectorEmbree correct(map);
    correct.setTsb(rm::Transform::Identity());
    auto model = rm::vlp16_900();
    model.range.min = 0.0;

    correct.setModel(model);


    std::cout << std::fixed << std::setprecision(2);

    std::cout << std::endl;
    std::cout << "Analyzing convergence for:" << std::endl;
    if(map_type == 0)
    {
        std::cout << "- map: cube grid" << std::endl;
    } else if(map_type == 1) {
        std::cout << "- map: avz" << std::endl;
    }

    if(sample_dist_type == 0)
    {
        std::cout << "- distribution: uniform circle" << std::endl;
    } else if(sample_dist_type == 1) {
        std::cout << "- distribution: uniform square" << std::endl;
    }

    std::cout << "- sample radii: [";
    for(size_t i=0; i<sample_radius_steps; i++)
    {
        float sample_radius = sample_radius_min + sample_radius_inc * static_cast<float>(i);
        std::cout << sample_radius;
        if(i < sample_radius_steps - 1 )
        {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;

    std::cout << "- sensor poses: " << std::endl;
    for(size_t spid = 0; spid < sensor_poses.size(); spid++)
    {
        std::cout << spid + 1 << ": " << sensor_poses[spid] << std::endl;
    }
    std::cout << std::endl;


    std::default_random_engine eng{(size_t)sample_seed};



    std::cout << "ICP convergence rates (#convergences / #poses) dependend on correspondence finding algorithm:" << std::endl;
    std::cout << "1. P2L: Point 2 Plane" << std::endl;
    std::cout << "2. SPC: Simulative Projective Correspondences" << std::endl;
    std::cout << std::endl;
    


    rm::StopWatch sw;
    double runtime_per_sensor_pose = 0.0; // seconds
    double runtime_per_iter = 0.0;

    
    for(size_t spid=0; spid < sensor_poses.size(); spid++)
    {
        if(spid < skip_poses)
        {
            continue;
        }
        
        std::cout << "POSE " << spid << ". " << spid + 1 << "/" << sensor_poses.size() << " poses. ";
        if(spid > 0)
        {
            std::cout << "Approx time: " << to_time(runtime_per_sensor_pose) << " per pose. ";
            std::cout << to_time(runtime_per_sensor_pose * (sensor_poses.size() - spid)) << " left. ";
            
        }

        std::cout << std::endl;

        std::cout << "| radius |   P2L conv (%) | P2L err (cm) | SPC conv (%) | SPC err (cm) |" << std::endl;
        std::cout << "|--------|----------------|--------------|--------------|--------------|" << std::endl;
        
        // |   100.00%      |
        // |----------------|
        
        std::stringstream ss;
        ss << evalfile_name << "_" << spid << ".csv";
        std::ofstream evalfile;
        evalfile.open(ss.str());
        evalfile << std::fixed << std::setprecision(8);
        evalfile << "sample radius, P2L, SPC" << std::endl;


        sw();

        rm::Transform T_dest = sensor_poses[spid];
        
        using ResultT = rm::Bundle<
            rm::Ranges<rm::RAM>
        >;

        ResultT sim_res;
        sim_res.ranges.resize(model.size());

        rm::Mem<rm::Transform, rm::RAM> T_dest_(1);
        T_dest_[0] = T_dest;

        // simulate the data that would be recorded at destination
        correct.simulate(T_dest_, sim_res);
        correct.setInputData(sim_res.ranges);

        std::uniform_real_distribution<float> dist_radius(0.0, 1.0);
        std::uniform_real_distribution<float> dist_angle(-M_PI, M_PI);

        for(size_t rid = 0; rid < sample_radius_steps; rid++)
        {
            if(!ros::ok())
            {
                return 0;
            }

            rm::StopWatch sw_inner;

            float sample_radius = sample_radius_min + sample_radius_inc * static_cast<float>(rid);
        
            rm::AABB sample_bb;
            sample_bb.min = rm::Vector::Max();
            sample_bb.max = rm::Vector::Min();
            // genererate poses around dest
            rm::Mem<rm::Transform, rm::RAM> Tbms(Nposes);
            for(size_t pid=0; pid<Tbms.size(); pid++)
            {
                Tbms[pid] = T_dest;
                
                rm::Vector point_random = {0.0, 0.0, 0.0};

                if(sample_dist_type == 0)
                { // uniform in circle
                    float radius_sample = sqrt(dist_radius(eng)) * sample_radius;
                    float angle_sample = dist_angle(eng);
                    point_random = rm::Vector{cos(angle_sample), sin(angle_sample), 0.0} * radius_sample;
                }
                else if(sample_dist_type == 1)
                { // uniform in square
                    float rand_x = (dist_radius(eng) - 0.5) * 2.0; // [-1, 1]
                    float rand_y = (dist_radius(eng) - 0.5) * 2.0; // [-1, 1]

                    point_random = {rand_x * sample_radius, rand_y * sample_radius, 0.0};
                }

                Tbms[pid].t += point_random;
                sample_bb.expand(Tbms[pid].t);
            }

            float p2l_rate = 0.0;
            float p2l_pose_error = 0.0;
            {
                rm::Memory<rm::Transform> T_p2l = Tbms;
                for(size_t i=0; i<Nruns; i++)
                {
                    {
                        auto corr_res = correct_embree_p2l(map, sim_res.ranges, model, T_p2l);
                        T_p2l = rm::multNxN(T_p2l, corr_res.Tdelta);
                    }

                    size_t n_converged = 0;
                    for(size_t pid=0; pid<Nposes; pid++)
                    {
                        float dist = (T_p2l[pid].t - T_dest.t).l2norm();
                        p2l_pose_error += dist;
                        if(dist <= dist_converged)
                        {
                            n_converged++;
                        }
                    }
                    p2l_rate = static_cast<double>(n_converged) / static_cast<double>(Nposes);
                    if(p2l_rate >= 1.0)
                    {
                        break;
                    }
                }
            }
            p2l_pose_error /= static_cast<float>(Nposes);

            float spc_rate = 0.0;
            float spc_pose_error = 0.0;
            {
                rm::Memory<rm::Transform> T_spc = Tbms;
                for(size_t i=0; i<Nruns; i++)
                {
                    {
                        auto corr_res = correct.correct(T_spc);
                        T_spc = rm::multNxN(T_spc, corr_res.Tdelta);
                    }

                    size_t n_converged = 0;
                    for(size_t pid=0; pid<Nposes; pid++)
                    {
                        float dist = (T_spc[pid].t - T_dest.t).l2norm();
                        spc_pose_error += dist;
                        if(dist <= dist_converged)
                        {
                            n_converged++;
                        }
                    }
                    spc_rate = static_cast<double>(n_converged) / static_cast<double>(Nposes);
                    if(spc_rate >= 1.0)
                    {
                        break;
                    }
                }
            }
            spc_pose_error /= static_cast<float>(Nposes);
            
            double el_inner = sw_inner();
            runtime_per_iter = std::max(runtime_per_iter, el_inner);



            

            std::cout << "| " 
                << std::setfill(' ') << std::setw(6) << sample_radius << " | " 
                << std::setfill(' ') << std::setw(13) << p2l_rate * 100.0 << "% | " 
                << std::setfill(' ') << std::setw(13) << p2l_pose_error * 100.0 << "cm | " 
                << std::setfill(' ') << std::setw(13) << spc_rate * 100.0 << "% | "
                << std::setfill(' ') << std::setw(13) << spc_pose_error * 100.0 << "cm | ";


            {
                double time_end_of_pose = runtime_per_iter * (sample_radius_steps - rid);
                double time_end_of_experiment = runtime_per_iter * static_cast<double>(Nruns) * static_cast<double>(sensor_poses.size() - spid + 1) + time_end_of_pose;
                std::cout << " time left: " << to_time(time_end_of_pose) << " for this pose, ";
                std::cout << to_time(time_end_of_experiment) << " to end of experiment";
            }
            std::cout << std::endl;
            
        
            evalfile << sample_radius << ", " 
                     << p2l_rate << ", " 
                     << p2l_pose_error << ", "
                     << spc_rate << ", "
                     << spc_pose_error << std::endl;
            
        }
        double el = sw();
        runtime_per_sensor_pose = std::max(runtime_per_sensor_pose, el);

        evalfile.close();
    }
    


    return 0;
}