#include "loop_closure.h"
#include "tf/tf.h"

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

    std::string path_topic("/lio_map_builder/path_aft_mapped");

    //sub_path_ = nl.subscribe("//blam/blam_slam/path", 100, &LoopClosure::PathCallback, this);
    sub_path_ = nl.subscribe(path_topic, 100, &LoopClosure::PathCallback, this);
    sub_points_ = nl.subscribe("/rslidar_points", 1, &LoopClosure::PointCloudCallback, this);

    pub_path_loop_ = nl.advertise<nav_msgs::Path>("path_loop", 10, false);
    pub_path_3dof_ = nl.advertise<nav_msgs::Path>("path_loop_3dof", 10, false);
    pub_octmap_ = nl.advertise<sensor_msgs::PointCloud2>("/oct_map", 1);
    pub_octmap_3dof_ = nl.advertise<sensor_msgs::PointCloud2>("/oct_map_3dof", 1);

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
    std::vector<double> timestamp_buff;

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
    key++;

    isam->update(new_factor, new_value);
    initial_estimate = isam->calculateEstimate();

    timestamp_buff.push_back(path_.poses[0].header.stamp.toSec()); //时间戳
    ROS_INFO("ISAM2 add edges");

    // 2.ISAM2 add edges
    gtsam::noiseModel::Diagonal::shared_ptr prior_model(gtsam::noiseModel::Diagonal::Sigmas(noise));
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

        timestamp_buff.push_back(path_.poses[i].header.stamp.toSec()); //时间戳
        key++;
    }

    ROS_INFO("ISAM2 correct loop");

    // 3.ISAM2 loop closure
    // 4.TODO:ICP求解delta
    unsigned int start_key = 0;
    unsigned int loop_key = key - 1;
    PointCloud scan_start;
    PointCloud scan_end;
    if (!(FindPointCloud(timestamp_buff[key - 1], scan_end) && FindPointCloud(timestamp_buff[0], scan_start)))
    {
        ROS_ERROR("can't find scans: %lf %lf", timestamp_buff[0], timestamp_buff[key - 1]);
        return;
    }

    gtsam::Pose3 start_pose = initial_estimate.at<gtsam::Pose3>(start_key);
    //gtsam::Pose3 loop_pose = initial_estimate.at<gtsam::Pose3>(loop_key);
    Eigen::Matrix4d pose_start = GtsamToEigen(initial_estimate.at<gtsam::Pose3>(start_key));
    ROS_INFO("cloud size: %lu %lu", scan_start.points.size(), scan_end.points.size());

    if (PerformICP(scan_start.makeShared(), scan_end.makeShared(), pose_start, delta))
    {
        ROS_INFO("-----SUCCESS------");
        ROS_INFO("found loop closure with %d and %d", loop_key, start_key);
        std::cout << "icp delta:" << delta;

        // We found a loop closure. Add it to the pose graph.
        NonlinearFactorGraph loop_factor;
        loop_factor.add(BetweenFactor<gtsam::Pose3>(loop_key, start_key, delta, init_covariance));
        isam->update(loop_factor, Values());
        final_estimate = isam->calculateEstimate();
    }
    else
    {
        ROS_WARN("-----FAILED------");
        return;
    }

    // gtsam::NonlinearFactorGraph new_factor3;
    // new_factor3.add(BetweenFactor<gtsam::Pose3>(loop_key, start_key, delta, prior_model));
    // isam->update(new_factor3, Values());
    // final_estimate = isam->calculateEstimate();
    ROS_INFO("loop closure with %d and %d", start_key, loop_key);

    // 5.
    ROS_INFO("publish result");
    PublishResult(final_estimate, timestamp_buff);

    // 6.generate map
    ROS_INFO("generate new map");
    GenerateMap();

    ROS_INFO("generate 3dof map");
    Generate3DofMap();
}

bool LoopClosure::FindPointCloud(double timestamp, PointCloud &cloud)
{
    uint64_t t = (uint64_t)(timestamp * 1000000.f);
    //ROS_INFO("FindPointCloud: %lf %ld in [%lu]", timestamp, t, points_buf_.size());

    for (auto &scan : points_buf_)
    {
        //fprintf(stderr, "%ld %ld \n", t, scan.header.stamp);
        if (scan.header.stamp == t)
        {
            copyPointCloud(scan, cloud);
            return true;
        }
    }
    ROS_WARN("can't find pointcloud: %lf", timestamp);
    return false;
}

void LoopClosure::PublishResult(gtsam::Values final_estimate, std::vector<double> timestamp_buff)
{
    //1.原path
    //pub_path_.publish(path_);

    //2.loop path
    nav_msgs::Path loop_path;
    loop_path.header = path_.header;
    int n = 0;
    for (const auto &keyed_pose : final_estimate)
    {
        geometry_msgs::PoseStamped pose_stamped;
        ros::Time t(timestamp_buff[n++]);
        pose_stamped.header.stamp = t; //时间戳
        pose_stamped.pose = GtsamToRos(final_estimate.at<gtsam::Pose3>(keyed_pose.key));
        loop_path.poses.push_back(pose_stamped);
    }
    pub_path_loop_.publish(loop_path);

    // 更新path，用于多次优化
    // path_ = loop_path;
    path_loop_ = loop_path;

    //3.3dof loop path
    nav_msgs::Path path_3dof;
    path_3dof.header = path_.header;
    n = 0;
    for (const auto &keyed_pose : final_estimate)
    {
        geometry_msgs::PoseStamped pose_stamped;
        ros::Time t(timestamp_buff[n++]);
        pose_stamped.header.stamp = t; //时间戳
        // 平面假设：roll pitch z 置为0
        Eigen::Matrix4d trans = GtsamToEigen(final_estimate.at<gtsam::Pose3>(keyed_pose.key));
        tf::Matrix3x3 mat(trans(0, 0), trans(0, 1), trans(0, 2),
                          trans(1, 0), trans(1, 1), trans(1, 2),
                          trans(2, 0), trans(2, 1), trans(2, 2));
        tfScalar yaw, pitch, roll;
        mat.getEulerYPR(yaw, pitch, roll);
        mat.setEulerYPR(yaw, 0, 0);
        Eigen::Matrix4d trans_new;
        trans_new << mat[0][0], mat[0][1], mat[0][2], trans(0, 3),
            mat[1][0], mat[1][1], mat[1][2], trans(1, 3),
            mat[2][0], mat[2][1], mat[2][2], 0,
            0, 0, 0, 1;

        pose_stamped.pose = EigenToRos(trans_new);
        path_3dof.poses.push_back(pose_stamped);
    }
    pub_path_3dof_.publish(path_3dof);
    path_3dof_ = path_3dof;
}

bool LoopClosure::PerformICP(const PointCloud::Ptr reference, const PointCloud::Ptr query, const Eigen::Matrix4d &pose0, gtsam::Pose3 &delta)
{
    ROS_INFO("PerformICP %lf %lf", (double)reference->header.stamp / 1000000.f, (double)query->header.stamp / 1000000.f);

    std::cout << "pose0:\n"
              << pose0 << std::endl;

    // Set up ICP.
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(1e-10);
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(100);
    icp.setRANSACIterations(0);

    PointCloud::Ptr source(new PointCloud);
    pcl::transformPointCloud(*query, *source, pose0);
    icp.setInputSource(source);

    // Set target point cloud in its own frame.
    PointCloud::Ptr target(new PointCloud);
    pcl::transformPointCloud(*reference, *target, pose0);
    icp.setInputTarget(target);

    // Perform ICP.
    PointCloud::Ptr unused_result(new PointCloud);
    //PointCloud unused_result;
    icp.align(*unused_result);

    // Get resulting transform: 1 to 2 in 2
    // frome query to reference in reference
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    Eigen::Matrix4d trans;
    trans << T(0, 0), T(0, 1), T(0, 2), T(0, 3),
        T(1, 0), T(1, 1), T(1, 2), T(1, 3),
        T(2, 0), T(2, 1), T(2, 2), T(2, 3),
        0, 0, 0, 1;
    delta = EigenToGtsam(trans.inverse());
    // Is the transform good?
    if (!icp.hasConverged())
    {
        ROS_INFO("icp hasn't converged");
        return false;
    }

    ROS_INFO("icp score:%f", icp.getFitnessScore());
    if (icp.getFitnessScore() > 5.0)
    {
        ROS_INFO("icp score is larger than 5: %f", icp.getFitnessScore());
        //return false;
    }
    std::cout << "delta:" << delta << std::endl;

#if 1
    pcl::io::savePCDFileASCII("query.pcd", *query);
    pcl::io::savePCDFileASCII("reference.pcd", *reference);
    pcl::io::savePCDFileASCII("source.pcd", *source);
    pcl::io::savePCDFileASCII("target.pcd", *target);
#endif

    return true;
}

void LoopClosure::PathCallback(const nav_msgs::Path::ConstPtr &msg)
{
    path_ = *msg;
}

void LoopClosure::PointCloudCallback(const PointCloud::ConstPtr &msg)
{
    PointCloud::Ptr laserCloudWithoutNaN(new PointCloud());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*msg, *laserCloudWithoutNaN, indices);

    PointCloud::Ptr msg_filtered(new PointCloud);
    FiltPoints(laserCloudWithoutNaN, msg_filtered);
    points_buf_.push_back(*msg_filtered);
}

void LoopClosure::GenerateMap()
{
    ROS_INFO("GenerateMap");

    // Initialize the map octree.
    PointCloud::Ptr regenerated_map(new PointCloud);
    PointCloud::Ptr map_data;
    map_data.reset(new PointCloud);
    map_data->header.frame_id = "world";
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree;
    map_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.02)); //octree_resolution:0.05
    map_octree->setInputCloud(map_data);

    for (const auto &pose_stamped : path_loop_.poses)
    {
        bool find_scan = false;
        uint64_t t = (uint64_t)(pose_stamped.header.stamp.toSec() * 1000000.f);
        PointCloud::Ptr scan_filtered(new PointCloud);
        for (auto &scan : points_buf_)
        {
            // printf("%lf -----> %lf\n", (double)scan.header.stamp / 1000000.f, t);
            if (scan.header.stamp == t)
            {
                *scan_filtered = scan;
                find_scan = true;
                break;
            }
        }
        if (!find_scan)
            continue;

        PointCloud::Ptr scan_world(new PointCloud);
        Eigen::Matrix4d b2w = PosestampedToEigen(pose_stamped);
        pcl::transformPointCloud(*scan_filtered, *scan_world, b2w);
        *regenerated_map += *scan_world;
    }

    ROS_INFO("regenerated map points: %lu", regenerated_map->points.size());
    if (regenerated_map->points.size() == 0)
        return;

    // oct tree
    for (size_t ii = 0; ii < regenerated_map->points.size(); ii++)
    {
        const pcl::PointXYZ p = regenerated_map->points[ii];
        if (!map_octree->isVoxelOccupiedAtPoint(p))
            map_octree->addPointToCloud(p, map_data);
    }

    // publish result
    ROS_INFO("octomap points: %lu", map_data->points.size());
    pub_octmap_.publish(map_data);

    // save
    ROS_INFO("save map");
    pcl::io::savePCDFileASCII("map_loop.pcd", *map_data);

    // release pcl buffer
    map_data->clear();
    map_octree.reset();
}

void LoopClosure::Generate3DofMap()
{
    ROS_INFO("Generate3DofMap");

    // Initialize the map octree.
    PointCloud::Ptr regenerated_map(new PointCloud);
    PointCloud::Ptr map_data;
    map_data.reset(new PointCloud);
    map_data->header.frame_id = "world";
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>::Ptr map_octree;
    map_octree.reset(new pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(0.02)); //octree_resolution:0.05
    map_octree->setInputCloud(map_data);

    for (const auto &pose_stamped : path_3dof_.poses)
    {
        bool find_scan = false;
        uint64_t t = (uint64_t)(pose_stamped.header.stamp.toSec() * 1000000.f);
        PointCloud::Ptr scan_filtered(new PointCloud);
        for (auto &scan : points_buf_)
        {
            if (scan.header.stamp == t)
            {
                *scan_filtered = scan;
                find_scan = true;
                break;
            }
        }
        if (!find_scan)
            continue;

        PointCloud::Ptr scan_world(new PointCloud);
        Eigen::Matrix4d b2w = PosestampedToEigen(pose_stamped);
        pcl::transformPointCloud(*scan_filtered, *scan_world, b2w);
        *regenerated_map += *scan_world;
    }

    ROS_INFO("points: %lu", regenerated_map->points.size());
    if (regenerated_map->points.size() == 0)
        return;

    // oct tree
    for (size_t ii = 0; ii < regenerated_map->points.size(); ii++)
    {
        const pcl::PointXYZ p = regenerated_map->points[ii];
        if (!map_octree->isVoxelOccupiedAtPoint(p))
            map_octree->addPointToCloud(p, map_data);
    }

    // publish result
    pub_octmap_3dof_.publish(map_data);

    // save
    ROS_INFO("save map");
    pcl::io::savePCDFileASCII("map_3dof.pcd", *map_data);

    // release pcl buffer
    map_data->clear();
    map_octree.reset();
}

void LoopClosure::FiltPoints(const PointCloud::ConstPtr &points, PointCloud::Ptr points_filtered)
{
    // Copy input points.
    *points_filtered = *points;

    bool random_filter = true;
    double decimate_percentage = 0.7;
    if (random_filter)
    {
        const int n_points = static_cast<int>((1.0 - decimate_percentage) * points_filtered->size());
        pcl::RandomSample<pcl::PointXYZ> random_filter;
        random_filter.setSample(n_points);
        random_filter.setInputCloud(points_filtered);
        random_filter.filter(*points_filtered);
    }

    // Apply a voxel grid filter to the incoming point cloud.
    bool grid_filter = false;
    double grid_res = 0.05;
    if (grid_filter)
    {
        pcl::VoxelGrid<pcl::PointXYZ> grid;
        grid.setLeafSize(grid_res, grid_res, 0.0);
        grid.setInputCloud(points_filtered);
        grid.filter(*points_filtered);
    }

    bool angle_filter = true;
    if (angle_filter)
    {
        PointCloud::Ptr msg_filtered(new PointCloud);
        AngleFilter(points_filtered, msg_filtered);
        *points_filtered = *msg_filtered;
    }
}

void LoopClosure::AngleFilter(const PointCloud::ConstPtr &points, PointCloud::Ptr points_filtered)
{
    if (points_filtered == NULL)
    {
        return;
    }

    points_filtered->header = points->header;
    points_filtered->height = points->height;
    for (unsigned int i = 0; i < points->points.size(); i++)
    {
        pcl::PointXYZ point = points->points[i];

        // if (point.z < -0.6)
        //   continue;

        double theta = atan(point.y / (point.x == 0 ? 1e-5 : point.x)) * 180.f / 3.141592653;
        if (point.x < 0 && theta > -45 && theta < 45)
            continue;

        points_filtered->points.push_back(point);
        ++points_filtered->width;
    }
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

gtsam::Pose3 LoopClosure::EigenToGtsam(Eigen::Matrix4d transform)
{
    geometry_msgs::Pose pose = EigenToRos(transform);
    return RosToGtsam(pose);
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
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();

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