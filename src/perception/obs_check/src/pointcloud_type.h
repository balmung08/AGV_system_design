#ifndef POINTCLOUD_TYPE_H
#define POINTCLOUD_TYPE_H

#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>

struct RsPointXYZIRT
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

typedef RsPointXYZIRT PointType;
// typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloud;

#endif
