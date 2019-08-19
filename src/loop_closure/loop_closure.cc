#include "loop_closure.h"

using namespace gtsam;

LoopClosure::LoopClosure()
{
}

LoopClosure::~LoopClosure()
{
}

bool LoopClosure::Initialize(const ros::NodeHandle &n)
{
    ros::NodeHandle nl(n);

    //sub_path_ = nl.subscribe("/vins_estimator/lidar_path", 100, &LoopClosure::PathCallback, this);
    sub_path_ = nl.subscribe("/vins_estimator/path", 100, &LoopClosure::PathCallback, this);
    sub_points_ = nl.subscribe("/rslidar_points", 1, &LoopClosure::PointCloudCallback, this);

    pub_path_loop_ = nl.advertise<nav_msgs::Path>("path_loop", 10, false);
    pub_octmap_ = nl.advertise<sensor_msgs::PointCloud2>("/oct_map", 1);

    timeshift_ = 0;

    return true;
}

void LoopClosure::HandleLoopClosures(bool bforce)
{
    ROS_INFO("HandleLoopClosures");

    int pose_size = path_.poses.size();
    if (pose_size < 10)
    {
        ROS_INFO("HandleLoopClosures: too few poses.");
        return;
    }

    std::unique_ptr<gtsam::ISAM2> isam;
    gtsam::Values initial_estimate;
    gtsam::Values final_estimate;
    gtsam::Pose3 delta;
    unsigned int key = 0;
    std::vector<ros::Time> timestamp_buff;

    ROS_INFO("ISAM2 INIT");

    // 1. ISAM2 INIT
    // isam initialize
    gtsam::ISAM2Params parameters;
    parameters.relinearizeSkip = 1;
    parameters.relinearizeThreshold = 0.01;
    isam.reset(new ISAM2(parameters));

    // Set the covariance on initial position.
    double sigma_x = 0.01, sigma_y = 0.01, sigma_z = 0.01;
    double sigma_roll = 0.004, sigma_pitch = 0.004, sigma_yaw = 0.004;
    gtsam::Vector6 noise;
    noise << sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw;
    gtsam::Pose3 init_pose = RosToGtsam(path_.poses[0].pose);
    gtsam::noiseModel::Diagonal::shared_ptr init_covariance(gtsam::noiseModel::Diagonal::Sigmas(noise));

    // Initialize ISAM2.
    gtsam::NonlinearFactorGraph new_factor;
    gtsam::Values new_value;
    new_factor.add(PriorFactor<gtsam::Pose3>(key, init_pose, init_covariance));
    new_value.insert(key, init_pose);

    isam->update(new_factor, new_value);
    initial_estimate = isam->calculateEstimate();

    timestamp_buff.push_back(path_.poses[0].header.stamp); //时间戳
    key++;
    ROS_INFO("ISAM2 add edges");

    // 2.ISAM2 add edges
    gtsam::noiseModel::Diagonal::shared_ptr prior_model(gtsam::noiseModel::Diagonal::Sigmas(noise));

    // isam edges
    for (int i = 1; i < pose_size; i++)
    {
        gtsam::Pose3 curr_pose = RosToGtsam(path_.poses[i].pose);
        gtsam::Pose3 last_pose = initial_estimate.at<gtsam::Pose3>(key - 1);
        delta = curr_pose.between(last_pose).inverse(); //这里，很重要 [delta=(last-new).inv，也就是new to last]

        double norm = sqrt(pow(curr_pose.translation().x() - last_pose.translation().x(), 2) +
                           pow(curr_pose.translation().y() - last_pose.translation().y(), 2) +
                           pow(curr_pose.translation().z() - last_pose.translation().z(), 2));
        double angle = acos(delta.rotation().toQuaternion().w()) * 2.f * 57.3f;

        // 直线距离大于1m，旋转角超过5度的帧加入关键帧，最后一帧也加入
        if (!((norm > 1 || angle > 5) && i < pose_size - 1))
            continue;

        // add factor
        gtsam::NonlinearFactorGraph new_factor2;
        new_factor2.add(BetweenFactor<gtsam::Pose3>(key - 1, key, delta, prior_model));

        gtsam::Values new_value2;
        new_value2.insert(key, curr_pose);

        // Update ISAM2.
        isam->update(new_factor2, new_value2);
        initial_estimate = isam->calculateEstimate();

        timestamp_buff.push_back(path_.poses[i].header.stamp); //时间戳
        key++;
    }

    ROS_INFO("ISAM2 correct loop");
#if 0
    gu::Transform3 delta;
    if (PerformICP(scan_curr, scan_start, pose_start, pose_start, &delta)) //这里以start frame为参考帧，一定注意
    {
        ROS_INFO("-----SUCCESS------");
        ROS_INFO("found loop closure with %d and %d", curr_key, start_key);

        // We found a loop closure. Add it to the pose graph.
        NonlinearFactorGraph new_factor;
        new_factor.add(BetweenFactor<gtsam::Pose3>(curr_key, start_key, ToGtsam(delta), ToGtsam(covariance)));
        isam_->update(new_factor, Values());
        closed_loop = true;
        last_closure_key_ = curr_key;

        // Store for visualization and output.
        loop_edges_.push_back(std::make_pair(key, start_key));
        closure_keys->push_back(start_key);

        // Send an empty message notifying any subscribers that we found a loop closure.
        loop_closure_notifier_pub_.publish(std_msgs::Empty());

        //values_.print("prev_loop values:");
        values_ = isam_->calculateEstimate();
        //values_.print("after_loop values:");

        ROS_DEBUG(" CorrectLoop: %d", curr_key);
    }
    else
        ROS_INFO("-----FAILED------");
#endif

    // 3.ISAM2 loop closure
    //gtsam::noiseModel::Diagonal::shared_ptr loop_model(gtsam::noiseModel::Diagonal::Sigmas(noise));
    unsigned int start_key = 0;
    unsigned int loop_key = key - 1;
    gtsam::Pose3 start_pose = initial_estimate.at<gtsam::Pose3>(start_key); //RosToGtsam(path_.poses[0].pose);
    gtsam::Pose3 loop_pose = initial_estimate.at<gtsam::Pose3>(loop_key);   //RosToGtsam(path_.poses[pose_size - 1].pose);
    delta = loop_pose.between(loop_pose);

    // ICP求解delta

    gtsam::NonlinearFactorGraph new_factor3;
    new_factor3.add(BetweenFactor<gtsam::Pose3>(loop_key, start_key, delta, prior_model));
    isam->update(new_factor3, Values());
    final_estimate = isam->calculateEstimate();

    //initial_estimate.print("input:");
    //final_estimate.print("output:");

    ROS_INFO("loop closure with %d and %d", start_key, loop_key);

    // publish result
    nav_msgs::Path loop_path;
    loop_path.header = path_.header;
    int n = 0;
    for (const auto &keyed_pose : final_estimate)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = timestamp_buff[n++]; //时间戳
        pose_stamped.pose = GtsamToRos(final_estimate.at<gtsam::Pose3>(keyed_pose.key));
        loop_path.poses.push_back(pose_stamped);
    }
    pub_path_loop_.publish(loop_path);

    // 更新path，用于多次优化
    path_ = loop_path;

    ROS_INFO("publish result");
}

#if 0
bool LoopClosure::PerformICP(const PointCloud::ConstPtr &scan1,
                             const PointCloud::ConstPtr &scan2,
                             const gu::Transform3 &pose1,
                             const gu::Transform3 &pose2,
                             gu::Transform3 *delta)
{
    if (delta == NULL)
    {
        ROS_ERROR("%s: Output pointers are null.", name_.c_str());
        return false;
    }

    // Set up ICP.
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(icp_tf_epsilon_);
    icp.setMaxCorrespondenceDistance(icp_corr_dist_);
    icp.setMaximumIterations(icp_iterations_);
    icp.setRANSACIterations(0);

    // Filter the two scans. They are stored in the pose graph as dense scans for
    // visualization.
    PointCloud::Ptr scan1_filtered(new PointCloud);
    PointCloud::Ptr scan2_filtered(new PointCloud);

    //降采样会降低icp匹配精度，降采样前score 0.24,降采样后0.9
    //filter_.Filter(scan1, scan1_filtered);
    //filter_.Filter(scan2, scan2_filtered);
    filter_.AngleFilter(scan1, scan1_filtered);
    filter_.AngleFilter(scan2, scan2_filtered);

#if 1
    pcl::io::savePCDFileASCII("1.pcd", *scan1);
    pcl::io::savePCDFileASCII("2.pcd", *scan2);
    pcl::io::savePCDFileASCII("1filt.pcd", *scan1_filtered);
    pcl::io::savePCDFileASCII("2filt.pcd", *scan2_filtered);
#endif

    // Set source point cloud. Transform it to pose 2 frame to get a delta.
    const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
    const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
    Eigen::Matrix4d body1_to_world;
    body1_to_world.block(0, 0, 3, 3) = R1;
    body1_to_world.block(0, 3, 3, 1) = t1;

    const Eigen::Matrix<double, 3, 3> R2 = pose2.rotation.Eigen();
    const Eigen::Matrix<double, 3, 1> t2 = pose2.translation.Eigen();
    Eigen::Matrix4d body2_to_world;
    body2_to_world.block(0, 0, 3, 3) = R2;
    body2_to_world.block(0, 3, 3, 1) = t2;

    PointCloud::Ptr source(new PointCloud);
    pcl::transformPointCloud(*scan1_filtered, *source, body1_to_world);
    icp.setInputSource(source);

    // Set target point cloud in its own frame.
    PointCloud::Ptr target(new PointCloud);
    pcl::transformPointCloud(*scan2_filtered, *target, body2_to_world);
    icp.setInputTarget(target);

    // Perform ICP.
    PointCloud unused_result;
    icp.align(unused_result);

    // Get resulting transform.
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    gu::Transform3 delta_icp;
    delta_icp.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    delta_icp.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                  T(1, 0), T(1, 1), T(1, 2),
                                  T(2, 0), T(2, 1), T(2, 2));

    // Is the transform good?
    if (!icp.hasConverged())
    {
        ROS_INFO("icp hasn't converged");
        return false;
    }

    //if (icp.getFitnessScore() < 1)
    {
        ROS_INFO("icp score:%f", icp.getFitnessScore());
    }

    if (icp.getFitnessScore() > 5.0)
    {
        ROS_INFO("icp score is larger than tolerable:%f > %f",
                 icp.getFitnessScore(),
                 max_tolerable_fitness_);
        return false;
    }

    // Update the pose-to-pose odometry estimate using the output of ICP.
    const gu::Transform3 update =
        gu::PoseUpdate(gu::PoseInverse(pose1),
                       gu::PoseUpdate(gu::PoseInverse(delta_icp), pose1));

    *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));

    {
        ros::Time t1, t2;
        t1 = t1.fromNSec(scan1->header.stamp * 1000ull); // Convert from us to ns;
        t2 = t2.fromNSec(scan2->header.stamp * 1000ull);
        printf("timestamp:[%f,%f]\n", t1.toSec(), t2.toSec());
        std::cout << "icp_deltaT:\n"
                  << *delta << std::endl;
    }

    return true;
}
#endif

// 根据时间戳
void LoopClosure::GenerateNewMap(PointCloud *points,
                                 std::vector<geometry_msgs::PoseStamped> poses,
                                 std::vector<PointCloud> scans)
{
    if (points == NULL)
    {
        ROS_ERROR("Output point cloud container is null.");
        return;
    }
    points->points.clear();

    // Initialize the map octree.
    PointCloud::Ptr map_data;
    map_data.reset(new PointCloud);
    map_data->header.frame_id = "world";
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree;
    map_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.05)); //octree_resolution:0.05
    map_octree->setInputCloud(map_data);

    int n_pose = poses.size();
    int n_scan = scans.size();
    printf(" pose count: %d\n", n_pose);
    printf(" scan count: %d\n", n_scan);

    // camera和lidar不在同一个时钟下，这里通过第一帧的时间粗略做个转换
    double pose_start = poses[0].header.stamp.toSec();
    double pose_end = poses[n_pose - 1].header.stamp.toSec();
    //double pose_period = (pose_end - pose_start) / n_pose;
    //double pose_period = 0.05; //pose周期:20ms

    double scan_start = (double)scans[0].header.stamp / 1000000.f;
    double scan_end = (double)scans[n_scan - 1].header.stamp / 1000000.f;

    printf(" pose count: %d [%.3f %.3f]\n", n_pose, pose_start, pose_end);
    printf(" scan count: %d [%.3f %.3f]\n", n_scan, scan_start, scan_end);

    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    //loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    for (unsigned int i = 0; i < scans.size() - 1; i++)
    {
        double t0 = (double)scans[i].header.stamp / 1000000.f;
        double t1 = (double)scans[i + 1].header.stamp / 1000000.f;

        for (const auto &pose_stamped : poses)
        {
            double t = pose_stamped.header.stamp.toSec() + timeshift_;
            if (t >= t0 && t < t1)
            {
                // pose需要插值，但是这里暂时先不做
                Eigen::Matrix4d b2w = PosestampedToEigen(pose_stamped);

                PointCloud scan_filtered;
                scan_filtered = scans[i];
                //FiltPoints(scans[i], scan_filtered);

                PointCloud scan_world;
                pcl::transformPointCloud(scan_filtered, scan_world, b2w);
                // scan_world.header.frame_id = "world";
                // pub_laser.publish(scan_world);

                *regenerated_map += scan_world;
                break;
            }
        }
    }

    // oct tree
    for (size_t ii = 0; ii < regenerated_map->points.size(); ii++)
    {
        const pcl::PointXYZ p = regenerated_map->points[ii];
        if (!map_octree->isVoxelOccupiedAtPoint(p))
            map_octree->addPointToCloud(p, map_data);
    }

    // publish result
    //pub_map.publish(*points);
    pub_octmap_.publish(map_data);
    pcl::io::savePCDFileASCII("mymap.pcd", *map_data);

    // release pcl buffer
    map_data->clear();
    map_octree.reset();
}

void LoopClosure::PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
}

void LoopClosure::PointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    //points_buf.push_back(*msg);
}

Pose3 LoopClosure::RosToGtsam(geometry_msgs::Pose pose)
{
    Vector3 t;
    t(0) = pose.position.x;
    t(1) = pose.position.y;
    t(2) = pose.position.z;

    Eigen::Quaterniond q(pose.orientation.w,
                         pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z);

    Eigen::Matrix3d m = q.normalized().toRotationMatrix();

    Rot3 r(m(0, 0), m(0, 1), m(0, 2),
           m(1, 0), m(1, 1), m(1, 2),
           m(2, 0), m(2, 1), m(2, 2));

    return Pose3(r, t);
}

geometry_msgs::Pose LoopClosure::GtsamToRos(Pose3 pose)
{
    geometry_msgs::Pose msg;

    msg.position.x = pose.translation().x();
    msg.position.y = pose.translation().y();
    msg.position.z = pose.translation().z();

    gtsam::Quaternion q = pose.rotation().toQuaternion();
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();

    return msg;
}

geometry_msgs::Pose LoopClosure::EigenToRos(Eigen::Matrix4d transform)
{
    geometry_msgs::Pose msg;

    msg.position.x = transform(0, 3);
    msg.position.y = transform(1, 3);
    msg.position.z = transform(2, 3);

    Eigen::Matrix3d mat = transform.block(0, 0, 3, 3);
    Eigen::Quaterniond q(mat);
    msg.orientation.w = q.w();
    msg.orientation.x = q.x();
    msg.orientation.y = q.y();
    msg.orientation.z = q.z();

    return msg;
}

Eigen::Matrix4d LoopClosure::GtsamToEigen(Pose3 pose)
{
    Eigen::Matrix4d m;

    gtsam::Quaternion q0 = pose.rotation().toQuaternion();
    Eigen::Quaterniond q(q0.w(), q0.x(), q0.y(), q0.z());
    Eigen::Matrix3d Rmat = q.normalized().toRotationMatrix();
    Eigen::Vector3d Tvec;
    Tvec[0] = pose.translation().x();
    Tvec[1] = pose.translation().y();
    Tvec[2] = pose.translation().z();

    m.block(0, 0, 3, 3) = Rmat;
    m.block(0, 3, 3, 1) = Tvec;
    return m;
}

Eigen::Matrix4d LoopClosure::PosestampedToEigen(geometry_msgs::PoseStamped pose_stamped)
{
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond q(pose_stamped.pose.orientation.w,
                         pose_stamped.pose.orientation.x,
                         pose_stamped.pose.orientation.y,
                         pose_stamped.pose.orientation.z);
    Eigen::Vector3d t(pose_stamped.pose.position.x,
                      pose_stamped.pose.position.y,
                      pose_stamped.pose.position.z);

    m.block(0, 0, 3, 3) = q.normalized().toRotationMatrix();
    m.block(0, 3, 3, 1) = t;

    return m;
}

void LoopClosure::FiltPoints(PointCloud points_input,
                             PointCloud &points_filtered)
{
    points_filtered.header = points_input.header;
    points_filtered.height = points_input.height;

    for (auto &p : points_input.points)
    {
        // 上下边界
        // if (p.z > top_bound_ || p.z < down_bound_)
        //     continue;

        // 角度范围
        double theta = atan(p.y / (p.x == 0 ? 1e-5 : p.x)) * 180.f / 3.14;
        if (p.x < 0 && theta > -45 && theta < 45)
            continue;

        points_filtered.points.push_back(p);
        ++points_filtered.width;
    }
}