#pragma once

#include <vector>

#include <ros/ros.h>
#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>

#include <motion_planning/State/Pose2D.hpp>

namespace RosUtils {

double quaternionToTheta(const geometry_msgs::Quaternion & q) {
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    return tf2::impl::getYaw(quat);
}

Pose2D transformToPose(const geometry_msgs::Transform & transform) {
    Pose2D pose = {
            .x=transform.translation.x,
            .y=transform.translation.y,
            .theta=quaternionToTheta(transform.rotation)
    };
    return pose;
}

Pose2D rosToPose(const geometry_msgs::Pose & msg) {
    Pose2D pose {.x=msg.position.x, .y=msg.position.y, .theta=quaternionToTheta(msg.orientation)};

    return pose;
}

geometry_msgs::Pose poseToRos(const Pose2D & pose) {
    geometry_msgs::Pose p;
    p.position.x = pose.x;
    p.position.y = pose.y;
    tf2::Quaternion quat;
    quat.setEuler(0., 0., pose.theta);
    p.orientation.x = quat.x();
    p.orientation.y = quat.y();
    p.orientation.z = quat.z();
    p.orientation.w = quat.w();

    return p;
}

geometry_msgs::Transform poseToTransform(const Pose2D & pose) {
    geometry_msgs::Transform t;
    t.translation.x = pose.x;
    t.translation.y = pose.y;

    tf2::Quaternion quat;
    quat.setEuler(0., 0., pose.theta);
    t.rotation.x = quat.x();
    t.rotation.y = quat.y();
    t.rotation.z = quat.z();
    t.rotation.w = quat.w();

    return t;
}

visualization_msgs::Marker treeToRos(
    const std::vector<std::vector<Pose2D>> & tree,
    const std::vector<double> & weights = std::vector<double>(0)) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tree";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.;
    marker.id = 1;

    marker.type = visualization_msgs::Marker::LINE_LIST;

    marker.scale.x = 0.02;
    marker.color.r = 1.;
    marker.color.a = 1.;

    for (size_t pathIndex = 0; pathIndex < tree.size(); pathIndex++) {
        const std::vector<Pose2D> & path = tree[pathIndex];

        for (size_t i = 0; i + 1 < path.size(); i++) {
            geometry_msgs::Point p0, p1;
            p0.x = path[i].x;
            p0.y = path[i].y;
            p1.x = path[i+1].x;
            p1.y = path[i+1].y;

            if (weights.size() > 0) {
                std_msgs::ColorRGBA color;
                color.r = (1 - weights[pathIndex]);
                color.g = weights[pathIndex];
                color.a = 1.;
                marker.colors.push_back(color);
                marker.colors.push_back(color);
            }

            marker.points.push_back(p0);
            marker.points.push_back(p1);

        }
    }

    return marker;
}

visualization_msgs::Marker pathToRos(const std::vector<Pose2D> & path) {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "path";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.;
    marker.id = 0;

    marker.type = visualization_msgs::Marker::LINE_STRIP;

    marker.scale.x = 0.05;
    marker.color.g = 1.;
    marker.color.a = 1.;

    for (auto pose = path.begin(); pose < path.end(); pose++) {
        geometry_msgs::Point p;
        p.x = (*pose).x;
        p.y = (*pose).y;
        marker.points.push_back(p);
    }

    return marker;
}

geometry_msgs::PoseArray poseArrayToRos(const std::vector<Pose2D> & poses) {
    geometry_msgs::PoseArray array;
    array.header.frame_id = "map";
    array.header.stamp = ros::Time::now();
    for (const Pose2D & pose : poses) {
        array.poses.push_back(RosUtils::poseToRos(pose));
    }
    return array;
}

};
