#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/ndt.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <termios.h>

#define LASER_POINT_COV (0.001)

enum InitializedFlag {

    NonInitialized = 0,
    Initializing = 1,
    Initialized = 2
};

KD_TREE<PointType> ikdtree;

mutex mtx_buffer;
condition_variable sig_buffer;

InitializedFlag init_flag = InitializedFlag::NonInitialized;

M4D init_pose = M4D::Identity();

double time_diff_lidar_to_imu = 0.0, timediff_lidar_wrt_imu = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double end_lidar_time = 0;
bool   ros_exit_flag = false, time_sync_flag = false;

deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr first_scan(new PointCloudXYZI());
PointCloudXYZI::Ptr rviz_global_map(new PointCloudXYZI());
PointCloudXYZI::Ptr raw_scan(new PointCloudXYZI());

esekfom::esekf kf;
state_ikfom state_point;

shared_ptr<Preprocess> lidar_process(new Preprocess());
shared_ptr<ImuProcess> imu_processs(new ImuProcess());

void sigHandle(int sig) {

    ros_exit_flag = true;
    ROS_WARN("Catch sig %d", sig);
    sig_buffer.notify_all();
}

void pointLidarToWorld(PointType const *const pi, PointType *const po) {

    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointLidarToIMU(PointType const *const pi, PointType *const po) {

    V3D p_lidar(pi->x, pi->y, pi->z);
    V3D p_imu(state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I);

    po->x = p_imu(0);
    po->y = p_imu(1);
    po->z = p_imu(2);
    po->intensity = pi->intensity;
}

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {

    if (!first_scan->points.empty()) {

        if (init_flag == InitializedFlag::NonInitialized) {

            init_flag = InitializedFlag::Initializing;

            const auto &pose = msg->pose.pose;
            QD q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            init_pose.block<3, 3>(0, 0) = q.toRotationMatrix();
            init_pose(0, 3) = pose.position.x;
            init_pose(1, 3) = pose.position.y;
            init_pose(2, 3) = pose.position.z;
        }
    }
}

void keyboardLoop() {

    // 配置终端为原始模式，以便直接读取按键
    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

    ROS_INFO("Keyboard Control");
    ROS_INFO("W: move forward | S: move backward | A: move left | D: move right | Q: turn left | E: turn right | F: finish");

    double resolution_dis = 0.05, resolution_rot = 1;

    while (true) {

        if (ros_exit_flag) {

            break;
        }

        char key = getchar();

        if (!first_scan->points.empty()) {

            if (init_flag == InitializedFlag::Initializing) {

                switch (key) {

                    case 'w':
                    case 'W':

                        init_pose(0, 3) += resolution_dis;
                        break;

                    case 's':
                    case 'S':

                        init_pose(0, 3) -= resolution_dis;
                        break;

                    case 'a':
                    case 'A':

                        init_pose(1, 3) += resolution_dis;
                        break;

                    case 'd':
                    case 'D':

                        init_pose(1, 3) -= resolution_dis;
                        break;

                    case 'q':
                    case 'Q':

                        // TODO:为什么是右乘
                        init_pose.block<3, 3>(0, 0) = init_pose.block<3, 3>(0, 0) * Eigen::AngleAxisd(resolution_rot * (M_PI / 180), V3D::UnitZ());
                        break;

                    case 'e':
                    case 'E':

                        init_pose.block<3, 3>(0, 0) = init_pose.block<3, 3>(0, 0) * Eigen::AngleAxisd(-resolution_rot * (M_PI / 180), V3D::UnitZ());
                        break;

                    case 'f':
                    case 'F':

                        init_flag = InitializedFlag::Initialized;
                        ROS_INFO("Success initialization!");
                        break;

                    default:

                        ROS_WARN("Invalid input");
                        continue;
                }
            }

            if (init_flag == InitializedFlag::Initialized) {

                break;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 恢复终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

void livoxCallback(const livox_ros_driver2::CustomMsg::ConstPtr &msg) {

    auto point_size = msg->point_num;

    if (!point_size) {

        return;
    }

    if (first_scan->points.empty()) {

        first_scan->resize(point_size);

        for (size_t i = 1; i < point_size; ++i) {

            if ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) {

                first_scan->points[i].x = msg->points[i].x;
                first_scan->points[i].y = msg->points[i].y;
                first_scan->points[i].z = msg->points[i].z;
                first_scan->points[i].intensity = msg->points[i].reflectivity;
            }
        }
    }

    raw_scan->clear();
    for (size_t i = 1; i < point_size; ++i) {

        if ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) {

            if (sqrt(pow(msg->points[i].x, 2) + pow(msg->points[i].y, 2)  + pow(msg->points[i].z, 2)) < 0.2) {

                continue;
            }

            PointType point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            point.intensity = msg->points[i].reflectivity;
            raw_scan->points.emplace_back(point);
        }
    }


    static bool timediff_set_flg = false;

    mtx_buffer.lock();

    if (msg->header.stamp.toSec() < last_timestamp_lidar) {

        ROS_WARN("Lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    last_timestamp_lidar = msg->header.stamp.toSec();
    
    if (!time_sync_flag && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() ) {

        ROS_WARN("IMU and LiDAR not Synced, IMU time: %lf, Lidar time: %lf \n", last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_flag && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty()) {

        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        ROS_WARN("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr pointCloud_ptr(new PointCloudXYZI());
    lidar_process->process(msg, pointCloud_ptr);
    lidar_buffer.push_back(pointCloud_ptr);
    time_buffer.push_back(last_timestamp_lidar);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr &msg_in) {

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_flag) {

        msg->header.stamp = ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {

        ROS_WARN("Imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.emplace_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool syncMessages(MeasureGroup &meas) {

    static double lidar_mean_scantime = 0.0;
    static int scan_num = 0;
    static bool lidar_pushed = false;

    if (lidar_buffer.empty() || imu_buffer.empty()) {

        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {

        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();

        if (meas.lidar->points.size() <= 1)  {  // time too little

            end_lidar_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");

        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {

            end_lidar_time = meas.lidar_beg_time + lidar_mean_scantime;

        } else {

            scan_num ++;
            end_lidar_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = end_lidar_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < end_lidar_time) {

        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < end_lidar_time)) {

        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > end_lidar_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

void publishResScan(const ros::Publisher &pub_resScan) {

    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++) {

        pointLidarToWorld(&feats_undistort->points[i], &feats_down_world->points[i]);
    }

    sensor_msgs::PointCloud2 ros_pointCloud;
    pcl::toROSMsg(*feats_down_world, ros_pointCloud);
    ros_pointCloud.header.stamp = ros::Time().fromSec(end_lidar_time);
    ros_pointCloud.header.frame_id = "camera_init";
    pub_resScan.publish(ros_pointCloud);
}

void publishRawScan(const ros::Publisher &pub_rawScan) {

    int size = raw_scan->points.size();
    PointCloudXYZI::Ptr raw_scan_world(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {

        pointLidarToWorld(&raw_scan->points[i], &raw_scan_world->points[i]);
    }

    sensor_msgs::PointCloud2 ros_pointCloud;
    pcl::toROSMsg(*raw_scan_world, ros_pointCloud);

    ros_pointCloud.header.stamp = ros::Time().fromSec(end_lidar_time);
    ros_pointCloud.header.frame_id = "camera_init";
    pub_rawScan.publish(ros_pointCloud);
}

void publishOdometry(const ros::Publisher &pub_odom) {

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    odom.header.stamp = ros::Time().fromSec(end_lidar_time);
    odom.pose.pose.position.x = state_point.pos(0);
    odom.pose.pose.position.y = state_point.pos(1);
    odom.pose.pose.position.z = state_point.pos(2);
    QD q(state_point.rot.matrix());
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++) {

        int k = i < 3 ? i + 3 : i - 3;
        odom.pose.covariance[i * 6 + 0] = P(k, 3);
        odom.pose.covariance[i * 6 + 1] = P(k, 4);
        odom.pose.covariance[i * 6 + 2] = P(k, 5);
        odom.pose.covariance[i * 6 + 3] = P(k, 0);
        odom.pose.covariance[i * 6 + 4] = P(k, 1);
        odom.pose.covariance[i * 6 + 5] = P(k, 2);
    }

    pub_odom.publish(odom);
}

void publishInitPose(const ros::Publisher &pub_initPose) {

    nav_msgs::Odometry odom;
    odom.header.frame_id = "camera_init";
    odom.child_frame_id = "body";
    odom.header.stamp =  ros::Time::now();
    odom.pose.pose.position.x = init_pose.block<3, 1>(0, 3).x();
    odom.pose.pose.position.y = init_pose.block<3, 1>(0, 3).y();
    odom.pose.pose.position.z = init_pose.block<3, 1>(0, 3).z();

    QD q(init_pose.block<3, 3>(0, 0));
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    pub_initPose.publish(odom);
}

void publishLaserCloud(const ros::Publisher &pub_laserCloud) {

    sensor_msgs::PointCloud2 ros_pointCloud;
    pcl::toROSMsg(*rviz_global_map, ros_pointCloud);
    ros_pointCloud.header.stamp = ros::Time::now();
    ros_pointCloud.header.frame_id = "camera_init";
    pub_laserCloud.publish(ros_pointCloud);
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "relocalization_lio");
    ros::NodeHandle nh;

    int NUM_MAX_ITERATIONS = 0;
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);

    string lid_topic, imu_topic;
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");

    nh.param<bool>("common/time_sync_en", time_sync_flag, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);

    double filter_surf_size = 0, filter_map_size = 0;
    nh.param<double>("filter_size_surf", filter_surf_size, 0.5);
    nh.param<double>("filter_size_map", filter_map_size, 0.5);

    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    nh.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);

    nh.param<double>("preprocess/blind", lidar_process->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_process->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", lidar_process->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", lidar_process->time_unit, US);
    nh.param<int>("preprocess/scan_rate", lidar_process->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", lidar_process->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", lidar_process->feature_enabled, false);

    bool extrinsic_est_flag = true;
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_flag, true);

    vector<double> extrin_t(3, 0.0), extrin_R(9, 0.0);
    nh.param<vector<double>>("mapping/extrinsic_T", extrin_t, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrin_R, vector<double>());

    int init_method = 0;
    nh.param<int>("init_method", init_method, 0);

    /*** ROS subscriber initialization ***/
    ros::Subscriber sub_lidar = nh.subscribe(lid_topic, 1000, livoxCallback);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 1000, imuCallback);
    ros::Subscriber sub_initPose = nh.subscribe("/initialpose", 10, initPoseCallback);

    /*** ROS publisher initialization ***/
    ros::Publisher pub_resScan = nh.advertise<sensor_msgs::PointCloud2>("/register_scan", 10);
    ros::Publisher pub_reloOdom = nh.advertise<nav_msgs::Odometry>("/relo_odom", 10);
    ros::Publisher pub_laserCloud = nh.advertise<sensor_msgs::PointCloud2>("/global_map", 10);
    ros::Publisher pub_initPose = nh.advertise<nav_msgs::Odometry>("/init_pose", 10);
    ros::Publisher pub_rawScan = nh.advertise<sensor_msgs::PointCloud2>("/raw_scan", 10);

    // 初始化过滤器
    pcl::VoxelGrid<PointType> filter_surf;
    filter_surf.setLeafSize(filter_surf_size, filter_surf_size, filter_surf_size);

    // 初始化外参和IMU噪声
    V3D lidar_wrt_IMU_t(Zero3d);
    M3D lidar_wrt_IMU_R(Eye3d);
    lidar_wrt_IMU_t <<VEC_FROM_ARRAY(extrin_t);
    lidar_wrt_IMU_R <<MAT_FROM_ARRAY(extrin_R);
    imu_processs->set_param(lidar_wrt_IMU_t, lidar_wrt_IMU_R,V3D(gyr_cov, gyr_cov, gyr_cov), V3D(acc_cov, acc_cov, acc_cov),V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov), V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // 初始化全局地图
    std::string map_path = ros::package::getPath("sfast_lio");
    std::string pcd_file = map_path + "/PCD/scans.pcd";

    PointCloudXYZI::Ptr global_map(new PointCloudXYZI());
    if (pcl::io::loadPCDFile(pcd_file, *global_map) == -1) {

        ROS_WARN("Can not load global map: %s!", pcd_file.c_str());
        return 0;
    }

    ROS_INFO("Load global map, map size: %zu!", global_map->points.size());

    if (ikdtree.Root_Node == nullptr) {

        ikdtree.set_downsample_param(filter_map_size); // 下采样
        ikdtree.Build(global_map->points);
    }

    // 滤波全局点云用于显示
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(global_map);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*global_map);

    // 过滤两米以上以及负一米以下的点云
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud(global_map);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-1.0, 2.0);
    pass.filter(*rviz_global_map);

    signal(SIGINT, sigHandle);

    // 显示全局地图
//    while (true) {
//
//        if (ros_exit_flag)
//            return 0;
//
//        if(pub_laserCloud.getNumSubscribers() != 0){
//
//            publishLaserCloud(pub_laserCloud);
//            break;
//        }
//    }

    if(init_method) {

        // 新建线程，监听键盘输入
        std::thread keyboard_thread(keyboardLoop);

        // 获取初始位姿
        ROS_INFO("Start get init pose!");
        while (true) {

            if (ros_exit_flag) {

                // 线程join
                keyboard_thread.join();
                return 0;
            }

            ros::spinOnce();

            if (!first_scan->points.empty() && init_flag == InitializedFlag::Initializing) {

                state_point.pos[0] = init_pose(0,3);
                state_point.pos[1] = init_pose(1,3);
                state_point.pos[2] = init_pose(2,3);

                Eigen::Matrix3d init_rot = init_pose.block<3, 3>(0,0);
                state_point.rot = Sophus::SO3(init_rot);

                if(feats_undistort != first_scan) {

                    feats_undistort = first_scan;
                }

                publishResScan(pub_resScan);
            }

            if(init_flag == InitializedFlag::Initialized) {

                break;
            }
        }

        // 线程join
        keyboard_thread.join();

        ROS_INFO("Start ndt align!");
        if (init_flag == InitializedFlag::Initialized) {

            auto first_scan_size = first_scan->size();
            PointCloudXYZI::Ptr feats_IMUFrame(new PointCloudXYZI(first_scan_size, 1));
            PointCloudXYZI::Ptr ndt_out(new PointCloudXYZI());

            for(size_t i = 0; i < first_scan_size; ++i) {

                pointLidarToIMU(&first_scan->points[i], &feats_IMUFrame->points[i]);
            }

            pcl::NormalDistributionsTransform<PointType, PointType> init_ndt;
            init_ndt.setTransformationEpsilon(0.01);
            init_ndt.setStepSize(0.1);
            init_ndt.setMaximumIterations(30);
            init_ndt.setInputSource(feats_IMUFrame);
            init_ndt.setInputTarget(global_map);
            init_ndt.setResolution(1.0);
            init_ndt.align(*ndt_out, init_pose.cast<float>());
            init_pose = init_ndt.getFinalTransformation().cast<double>();
            ROS_INFO("Align score: %f!", init_ndt.getFitnessScore());

            if (init_ndt.getFitnessScore() < 0.5) {

                ROS_INFO("Align success!");

                state_point.pos[0] = init_pose(0,3);
                state_point.pos[1] = init_pose(1,3);
                state_point.pos[2] = init_pose(2,3);
                M3D rot = init_pose.block<3, 3>(0, 0);
                state_point.rot = Sophus::SO3(rot);

                // 更新滤波器
                kf.change_x(state_point);

                // 发布配准的点云
                publishResScan(pub_resScan);

                // 清空已经订阅的数据
                lidar_buffer.clear();
                imu_buffer.clear();
                time_buffer.clear();

            } else {

                ROS_WARN("Align fail");
                return 0;
            }

        } else {

            return 0;
        }
    }

    ROS_INFO("Start localization based map!");

    MeasureGroup measures;
    bool flg_first_scan = true;
    double first_lidar_time = 0.0;

    ros::Rate rate(100);

    while (true) {

        if (ros_exit_flag) {

            return 0;
        }

        ros::spinOnce();

        if (syncMessages(measures)) {

            if (flg_first_scan) {

                first_lidar_time = measures.lidar_beg_time;
                imu_processs->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            // IMU预测和去点云畸变
            imu_processs->Process(measures, kf, feats_undistort);

            if (feats_undistort->points.empty() || (feats_undistort == nullptr)) {

                ROS_WARN("No point, skip this scan!");
                continue;
            }

            PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
            filter_surf.setInputCloud(feats_undistort);
            filter_surf.filter(*feats_down_body);

            int feats_down_size = feats_down_body->points.size();
            if (feats_down_size < 5) {

                ROS_WARN("No point, skip this scan!");
                continue;
            }

            vector<PointVector> Nearest_Points;
            Nearest_Points.resize(feats_down_size); //存储近邻点的vector
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, feats_down_body, ikdtree, Nearest_Points, NUM_MAX_ITERATIONS, extrinsic_est_flag);

            // 更新状态
            state_point = kf.get_x();

            publishInitPose(pub_initPose);
            publishOdometry(pub_reloOdom);
            publishResScan(pub_resScan);
            publishRawScan(pub_rawScan);
        }

        rate.sleep();
    }

    return 0;
}
