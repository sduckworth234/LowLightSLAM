/**
* 
* Common functions and variables across all modes (mono/stereo, with or w/o imu)
*
*/

#include "common.h"


ORB_SLAM3::System::eSensor sensor_type;
std::string world_frame_id, cam_frame_id, imu_frame_id;
Sophus::SE3f Tc0w = Sophus::SE3f();

// Add the ORB-SLAM3 system pointer
ORB_SLAM3::System* mpSLAM = nullptr;

ros::Publisher pose_pub, map_points_pub;

void setup_ros_publishers(ros::NodeHandle &node_handler, image_transport::ImageTransport &image_transport, Eigen::Vector3d rpy_rad)
{
    pose_pub = node_handler.advertise<geometry_msgs::PoseStamped>("orb_slam3/camera_pose", 1);

    map_points_pub = node_handler.advertise<sensor_msgs::PointCloud2>("orb_slam3/map_points", 1);
    
    if (!rpy_rad.isZero(0))
    {
        Eigen::AngleAxisf AngleR(rpy_rad(0), Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf AngleP(rpy_rad(1), Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf AngleY(rpy_rad(2), Eigen::Vector3f::UnitZ());
        Eigen::Quaternionf qRPY = AngleR * AngleP * AngleY;
        Eigen::Matrix3f RotRPY = qRPY.matrix();
        Tc0w = Sophus::SE3f(RotRPY, Eigen::Vector3f::Zero());
        ROS_INFO("World frame will be rotated by RPY (in that order) %f %f %f (rad)", rpy_rad[0], rpy_rad[1], rpy_rad[2]);
    }
}

void publish_ros_camera_pose(Sophus::SE3f Twc_SE3f, ros::Time msg_time)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id = world_frame_id;
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Twc_SE3f.translation().x();
    pose_msg.pose.position.y = Twc_SE3f.translation().y();
    pose_msg.pose.position.z = Twc_SE3f.translation().z();

    pose_msg.pose.orientation.w = Twc_SE3f.unit_quaternion().coeffs().w();
    pose_msg.pose.orientation.x = Twc_SE3f.unit_quaternion().coeffs().x();
    pose_msg.pose.orientation.y = Twc_SE3f.unit_quaternion().coeffs().y();
    pose_msg.pose.orientation.z = Twc_SE3f.unit_quaternion().coeffs().z();

    pose_pub.publish(pose_msg);
}

void publish_ros_tf_transform(Sophus::SE3f Twc_SE3f, string frame_id, string child_frame_id, ros::Time msg_time)
{
    tf::Transform tf_transform = SE3f_to_tfTransform(Twc_SE3f);

    static tf::TransformBroadcaster tf_broadcaster;

    tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, msg_time, frame_id, child_frame_id));
}

void publish_ros_tracked_mappoints(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = tracked_mappoints_to_pointcloud(map_points, msg_time);
    
    map_points_pub.publish(cloud);
}


// SERVICE FUNCTIONS - Updated to use available methods
bool save_map_srv(orb_slam3_ros_wrapper::SaveMap::Request &req, orb_slam3_ros_wrapper::SaveMap::Response &res)
{
    if (!mpSLAM) {
        ROS_ERROR("ORB-SLAM3 system not initialized!");
        res.success = false;
        return false;
    }

    try {
        // Since SaveMap method doesn't exist, we'll save the map points manually
        std::vector<ORB_SLAM3::MapPoint*> map_points = mpSLAM->GetTrackedMapPoints();
        std::string map_file = req.name + "_mappoints.txt";
        
        std::ofstream file(map_file);
        if (!file.is_open()) {
            res.success = false;
            ROS_ERROR("Could not open file %s for writing", map_file.c_str());
            return false;
        }
        
        int valid_points = 0;
        for (auto* mp : map_points) {
            if (mp && !mp->isBad()) {
                Eigen::Vector3f pos = mp->GetWorldPos();
                file << pos.x() << " " << pos.y() << " " << pos.z() << std::endl;
                valid_points++;
            }
        }
        file.close();
        
        res.success = true;
        ROS_INFO("Map points saved to %s (%d points)", map_file.c_str(), valid_points);
        
    } catch (const std::exception &e) {
        ROS_ERROR("Map save failed: %s", e.what());
        res.success = false;
    } catch (...) {
        ROS_ERROR("Map save failed: Unknown exception");
        res.success = false;
    }

    return res.success;
}

bool save_traj_srv(orb_slam3_ros_wrapper::SaveMap::Request &req, orb_slam3_ros_wrapper::SaveMap::Response &res)
{
    if (!mpSLAM) {
        ROS_ERROR("ORB-SLAM3 system not initialized!");
        res.success = false;
        return false;
    }

    const std::string cam_traj_file = req.name + "_cam_traj.txt";
    const std::string kf_traj_file = req.name + "_kf_traj.txt";

    try {
        // These methods exist in your ORB-SLAM3 version
        mpSLAM->SaveTrajectoryEuRoC(cam_traj_file);
        mpSLAM->SaveKeyFrameTrajectoryEuRoC(kf_traj_file);
        res.success = true;
        ROS_INFO("Trajectories saved as %s and %s", cam_traj_file.c_str(), kf_traj_file.c_str());
        
    } catch (const std::exception &e) {
        std::cerr << e.what() << std::endl;
        res.success = false;
        ROS_ERROR("Trajectory could not be saved: %s", e.what());
    } catch (...) {
        std::cerr << "Unknown exception" << std::endl;
        res.success = false;
        ROS_ERROR("Trajectory could not be saved: Unknown exception");
    }

    return res.success;
}

void setup_services(ros::NodeHandle &node_handler, std::string node_name)
{
    static ros::ServiceServer save_map_service = node_handler.advertiseService(node_name + "/save_map", save_map_srv);
    static ros::ServiceServer save_traj_service = node_handler.advertiseService(node_name + "/save_traj", save_traj_srv);
    ROS_INFO("Save services initialized for %s", node_name.c_str());
}

// ...rest of existing code...

//
// Miscellaneous functions
//
sensor_msgs::PointCloud2 tracked_mappoints_to_pointcloud(std::vector<ORB_SLAM3::MapPoint*> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = world_frame_id;
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            // Original data
            Eigen::Vector3f pMPw = map_points[i]->GetWorldPos();
            
            // Apply world frame orientation for non-IMU cases
            if (sensor_type == ORB_SLAM3::System::MONOCULAR || sensor_type == ORB_SLAM3::System::STEREO)
            {
                Sophus::SE3f Tc0mp(Eigen::Matrix3f::Identity(), pMPw);
                Sophus::SE3f Twmp = Tc0w.inverse() * Tc0mp;
                pMPw = Twmp.translation();
            }

            tf::Vector3 point_translation(pMPw.x(), pMPw.y(), pMPw.z());

            float data_array[num_channels] = {
                point_translation.x(),
                point_translation.y(),
                point_translation.z()
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

tf::Transform SE3f_to_tfTransform(Sophus::SE3f T_SE3f)
{
    Eigen::Matrix3f R_mat = T_SE3f.rotationMatrix();
    Eigen::Vector3f t_vec = T_SE3f.translation();

    tf::Matrix3x3 R_tf(
        R_mat(0, 0), R_mat(0, 1), R_mat(0, 2),
        R_mat(1, 0), R_mat(1, 1), R_mat(1, 2),
        R_mat(2, 0), R_mat(2, 1), R_mat(2, 2)
    );

    tf::Vector3 t_tf(
        t_vec(0),
        t_vec(1),
        t_vec(2)
    );

    return tf::Transform(R_tf, t_tf);
}