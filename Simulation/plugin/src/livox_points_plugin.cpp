//
// Created by lfc on 2021/2/28.
//

#include "livox_laser_simulation/livox_points_plugin.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include <pcl_conversions/pcl_conversions.h>

namespace gazebo
{

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

    LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas, std::vector<AviaRotateInfo> &avia_infos)
    {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        for (auto &data : datas)
        {
            if (data.size() == 3)
            {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2; // 转化成标准的右手系角度
            }
            else
            {
                ROS_INFO_STREAM("data size is not 3!");
            }
        }
    }

    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf)
    {
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");
        // 如果file_name是以package://开头，则替换为绝对路径
        if (file_name.find("package://") == 0)
        {
            // 截取第一个package的名称
            std::string package_name = file_name.substr(10, file_name.find("/", 10) - 10);
            file_name = file_name.substr(file_name.find("/", 10));
            // 获取package的绝对路径
            file_name = ros::package::getPath(package_name) + file_name;
        }
        ROS_INFO_STREAM("load csv file name:" << file_name);
        if (!CsvReader::ReadCsvFile(file_name, datas))
        {
            ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
            return;
        }
        sdfPtr = sdf;
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");

        int argc = 0;
        char **argv = nullptr;

        robot_namespace = "";
        if (sdf->HasElement("robotNamespace"))
        {
            robot_namespace =
                sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        }
        if(robot_namespace == "/" || robot_namespace == "")
        {
            robot_namespace = "livox";
        }
        ROS_INFO_STREAM("robot namespace: " << robot_namespace);

        auto curr_scan_topic = sdf->Get<std::string>("topic_name");
        // 如果curr_scan_topic为空，则设置为默认值
        if (curr_scan_topic.empty())
        {
            curr_scan_topic = "livox_points";
        }
        // 如果topic_name第一个字符是/，则去掉
        if (curr_scan_topic[0] == '/')
        {
            curr_scan_topic = curr_scan_topic.substr(1);
        }

        // curr_scan_topic = "~" + curr_scan_topic;
        ROS_INFO_STREAM("ros topic name:" << curr_scan_topic);
        ros::init(argc, argv, curr_scan_topic);
        rosNode.reset(new ros::NodeHandle(robot_namespace));
        rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);

        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        // SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
        // ROS_INFO_STREAM("raySensor parentName:" << raySensor->ParentName());
        // ROS_INFO_STREAM("raySensor Name:" << raySensor->Name());

        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);
        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
        maxPointSize = aviaInfos.size();

        RayPlugin::Load(_parent, sdfPtr);
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());
        parentEntity = this->world->EntityByName(_parent->ParentName());
        auto physics = world->Physics();
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples");
        downSample = sdfPtr->Get<int>("downsample");
        if (downSample < 1)
        {
            downSample = 1;
        }
        ROS_INFO_STREAM("sample:" << samplesStep);
        ROS_INFO_STREAM("downsample:" << downSample);
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample)
        {
            int index = j % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }

    void LivoxPointsPlugin::OnNewLaserScans()
    {
        if (rayShape)
        {
            std::vector<std::pair<int, AviaRotateInfo>> points_pair;
            InitializeRays(points_pair, rayShape);
            rayShape->Update();

            msgs::Set(laserMsg.mutable_time(), world->SimTime());

            // PublishPointCloud2XYZ(points_pair);
            msgs::LaserScan *scan = laserMsg.mutable_scan();
            InitializeScan(scan);
            std::string child_name = robot_namespace + "_" + raySensor->Name();
            // 去除所有的 /
            child_name.erase(std::remove(child_name.begin(), child_name.end(), '/'), child_name.end());
            std::string world_name = "world";
            SendRosTf(parentEntity->WorldPose(), world_name, child_name);

            auto rayCount = RayCount();
            auto verticalRayCount = VerticalRayCount();
            auto angle_min = AngleMin().Radian();
            auto angle_incre = AngleResolution();
            auto verticle_min = VerticalAngleMin().Radian();
            auto verticle_incre = VerticalAngleResolution();

            sensor_msgs::PointCloud2 scan_point;
            // scan_point.header.stamp = ros::Time::now();
            // scan_point.header.frame_id = raySensor->Name();
            // auto &scan_points = scan_point.points;
            pcl::PointCloud<pcl::PointXYZ> pc;
            pc.points.resize(points_pair.size());
            int pt_count = 0;
            for (auto &pair : points_pair)
            {
                // int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
                // int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
                // if (verticle_index < 0 || horizon_index < 0) {
                //    continue;
                // }
                // if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                //    auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= RangeMax())
                {
                    range = 0;
                }
                else if (range <= RangeMin())
                {
                    range = 0;
                }
                // scan->set_ranges(index, range);
                // scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();//转换成世界坐标系

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                // scan_points.emplace_back();
                // scan_points.back().x = point.X();
                // scan_points.back().y = point.Y();
                // scan_points.back().z = point.Z();
                pcl::PointXYZ pt;
                pt.x = point.X();
                pt.y = point.Y();
                pt.z = point.Z();
                // 如果这个点小于于0.2，并且大于-0.2，并且y小于0.2，并且大于-0.2，那么去除这个点
                if(pt.x < 0.2 && pt.x >-0.2 && pt.y<0.2 && pt.y>-0.2)
                {
                    continue;
                }
#pragma omp critical
            {
                pc[pt_count] = pt;
                ++pt_count;
            }
                //} else {

                //    //                ROS_INFO_STREAM("count is wrong:" << verticle_index << "," << verticalRayCount << ","
                //    //                << horizon_index
                //    //                          << "," << rayCount << "," << pair.second.zenith << "," <<
                //    //                          pair.second.azimuth);
                //}
            }
            if (scanPub && scanPub->HasConnections())
                scanPub->Publish(laserMsg);
            pc.resize(pt_count);
            pcl::toROSMsg(pc, scan_point);
            // scan_point.header.stamp = timestamp;
            // scan_point.header.frame_id = "livox";
            scan_point.header.stamp = ros::Time::now();
            // scan_point.header.frame_id = raySensor->Name();
            scan_point.header.frame_id = child_name;
            rosPointPub.publish(scan_point);
            // rosPointPub.publish(scan_point);
            ros::spinOnce();
        }
    }

    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
                                           boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape)
    {
        auto &rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        int64_t end_index = currStartIndex + samplesStep;
        int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());
        for (int k = currStartIndex; k < end_index; k += downSample)
        {
            auto index = k % maxPointSize;
            auto &rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size)
            {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex += samplesStep;
    }

    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan)
    {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j)
        {
            for (unsigned int i = 0; i < rangeCount; ++i)
            {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMin() const
    {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::AngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    double LivoxPointsPlugin::RangeMin() const
    {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    double LivoxPointsPlugin::RangeMax() const
    {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const
    {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const
    {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    int LivoxPointsPlugin::VerticalRangeCount() const
    {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        }
        else
            return -1;
    }

    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const
    {
        if (rayShape)
        {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        }
        else
            return -1;
    }

    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    double LivoxPointsPlugin::VerticalAngleResolution() const
    {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }
    void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d &pose, const std::string &father_frame,
                                      const std::string &child_frame)
    {
        if (!tfBroadcaster)
        {
            tfBroadcaster.reset(new tf::TransformBroadcaster);
        }
        tf::Transform tf;
        auto rot = pose.Rot();
        auto pos = pose.Pos();
        tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
        tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
        tfBroadcaster->sendTransform(
            tf::StampedTransform(tf, ros::Time::now(), father_frame, child_frame));
            // tf::StampedTransform(tf, ros::Time::now(), "world", raySensor->Name()));
    }

    void LivoxPointsPlugin::PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>> &points_pair) {
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        msgs::LaserScan *scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        sensor_msgs::PointCloud2 scan_point;

        pcl::PointCloud<pcl::PointXYZ> pc;
        pc.points.resize(points_pair.size());
        ros::Time timestamp = ros::Time::now();
        int pt_count = 0;
    #pragma omp parallel for
        for (int i = 0; i < points_pair.size(); ++i) {
            std::pair<int, gazebo::AviaRotateInfo> &pair = points_pair[i];
            int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            if (verticle_index < 0 || horizon_index < 0) {
                continue;
            }
            if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= RangeMax() || range <= RangeMin() || range < 0.1) continue;

                scan->set_ranges(index, range);
                scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();//转换成世界坐标系

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

                // if (range < 0.3) {
                //     ROS_WARN_STREAM("Small pt: range: " << range << ", axis: " << axis);
                // }
                auto point = range * axis;
                pcl::PointXYZ pt;
                pt.x = point.X();
                pt.y = point.Y();
                pt.z = point.Z();
                // if (pt_count < pc.size() && pt_count > 0)
    #pragma omp critical
                {
                    pc[pt_count] = pt;
                    ++pt_count;
                }
            }
        }
        pc.resize(pt_count);
        pcl::toROSMsg(pc, scan_point);
        scan_point.header.stamp = timestamp;
        scan_point.header.frame_id = raySensor->Name();
        rosPointPub.publish(scan_point);
        // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());
        ros::spinOnce();
        // if (scanPub && scanPub->HasConnections()) {
        //     scanPub->Publish(laserMsg);
        // }
    }
}