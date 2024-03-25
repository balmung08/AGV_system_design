#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "pcl/filters/extract_indices.h"
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/search/kdtree.h"
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Core>
#include <vector>


// struct RsPointXYZI//RT
// {
//   PCL_ADD_POINT4D;
//   float intensity;
//   //uint16_t ring = 0;
//   //double timestamp = 0;
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity))//(
//                                                      //uint16_t, ring, ring)(double, timestamp, timestamp))

class TCloudEvalue
{
public:
    pcl::PointCloud<pcl::PointXYZI> cloud;
    vector<int> rings;
    float intensity_avg;
    float dx_avg, dy_avg, dz_avg;
    float value;

    // float intensity_sta=18;
    // float dx_sta = 0.2;
    // float dz_sta = 0.2;
    // float dy_sta = 0.1;

    float intensity_sta=10;
    float dx_sta = 0.20;
    float dz_sta = 0.05;
    float dy_sta = 0.5;
   
    // float intensity_sta=60;
    // float dx_sta = 0.13;
    // float dz_sta = 0.84;
    // float dy_sta = 0.1;
    
    TCloudEvalue()
    {
        cloud.clear();
    }

    // void GetRings()
    // {
    //     rings.clear();//xianxu
    //     for(auto it=cloud.begin(); it!=cloud.end(); ++it)
    //     {
    //         if(rings.empty())  rings.push_back(it->ring);//suoyoudianyun de ring
    //         else
    //         {
    //             vector<int>::iterator t=find(rings.begin(),rings.end(),it->ring);
    //             if(t==rings.end())  rings.push_back(it->ring);
    //         }
    //     }

    //     pcl::PointXYZI  minpoint, maxpoint;
    //     vector<float> dx,dy,dz;
    //     for(int i=0;i<rings.size();i++)
    //     {
    //         pcl::PointCloud<pcl::PointXYZI> line;
    //         for(auto it=cloud.begin(); it!=cloud.end(); ++it)
    //         {
    //             if(rings[i]==it->ring) line.push_back(*it);
    //         }

    //         if(line.size()>2)
    //         {
    //             pcl::getMinMax3D(line, minpoint, maxpoint);
    //             dx.push_back(maxpoint.x-minpoint.x);
    //             dy.push_back(maxpoint.y-minpoint.y);
    //             dz.push_back(maxpoint.z-minpoint.z);
    //         }
    //     } 

    //     dx_avg=0;

    //     for(int i=0;i<dx.size();i++) 
    //     {
    //         dx_avg+=dx[i];
    //         // cout << "x[i] = " << dx[i] << endl;
    //     }
    //     dx_avg/=dx.size();
    //     dz_avg=0;

    //     for(int i=0;i<dz.size();i++) 
    //     {
    //         dz_avg+=dz[i];
    //         // cout << "z[i] = " << dz[i] << endl;
    //     }
    //     dz_avg/=dz.size();

    //     //printf("%.2f %.2f\n",dx_avg,dz_avg);
    // }
    void Getsize()
    {
        vector<pcl::PointIndices> cluster_indices;
        pcl::PointXYZI minpoint, maxpoint;
        // for (auto it = cluster_indices.begin(); it != cluster_indices.end (); ++it)
        // {
                
            pcl::PointCloud<pcl::PointXYZI> cloud_cluster;


            for (auto it=cloud.begin(); it!=cloud.end(); ++it)
            {
                cloud_cluster.push_back(*it);
            }
        
            
            vector<float> dx,dy,dz;
            dx_avg = dy_avg = dz_avg =0;
            pcl::getMinMax3D(cloud_cluster, minpoint, maxpoint);
            {
                // dx.push_back(maxpoint.x-minpoint.x);
                // dy.push_back(maxpoint.y-minpoint.y);
                // dz.push_back(maxpoint.z-minpoint.z);co
                dx_avg = maxpoint.x-minpoint.x;
                dy_avg = maxpoint.y-minpoint.y;
                dz_avg = maxpoint.z-minpoint.z;
            }
        // }
    }
    void GetIntensity()
    {
        intensity_avg=0;
        for(auto it=cloud.begin(); it!=cloud.end(); ++it)
        {
            intensity_avg+=it->intensity;
        } 
        intensity_avg/=cloud.size();
    }

    float GetEvaValue()
    {
        // GetRings();
        Getsize();
        GetIntensity();
        // cout <<"dx" << dx_avg  << "dz is  = " <<  dz_avg <<endl;
        value=0;
        value+=abs(intensity_avg-intensity_sta)/intensity_sta;
        value+=abs(dx_avg-dx_sta)/dx_sta;
        // value+=abs(dy_avg-dy_sta)/dy_sta;
        value+=abs(dz_avg-dz_sta)/dz_sta;
        return value;
    }
 
};