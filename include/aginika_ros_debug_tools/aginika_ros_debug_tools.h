// -*- mode: C++ -*-
#ifndef __AGINIKA_ROS_DEBUG_TOOLS_H__
#define __AGINIKA_ROS_DEBUG_TOOLS_H__

//Definitions
#define ROS_LF ROS_INFO("%s : at %d",__func__,__LINE__)

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_broadcaster.h>

#define PUBLISH_NORMAL_MARKER_ARRAY(node_handle, topic_name, point_type, point_cloud, frame_id) \
  {                                                                     \
    static ros::Publisher aginika_ros_debug_tools_publisher = (node_handle).advertise<visualization_msgs::MarkerArray>((topic_name),1); \
    aginika_ros_debug_tools::publish_pointcloud_normal<point_type>((aginika_ros_debug_tools_publisher), (point_cloud), (frame_id)); \
      }

#define PUBLISH_DEBUG_POINTCLOUD(node_handle, topic_name, point_type, point_cloud, frame_id) \
  {                                                                     \
  static ros::Publisher aginika_ros_debug_tools_publisher = (node_handle).advertise<sensor_msgs::PointCloud2>((topic_name),1); \
    aginika_ros_debug_tools::publish_pointcloud<point_type>((aginika_ros_debug_tools_publisher), (point_cloud), (frame_id)); \
  }

#define PUBLISH_DEBUG_TF(br, tf_name, point_type, pt, target_frame_id)   \
{\
    aginika_ros_debug_tools::publish_debug_tf<point_type>(br, tf_name, pt, target_frame_id); \
}

#define PUBLISH_DEBUG_TF_XYZ(br, tf_name,point_type, pt, target_frame_id) \
{\
    aginika_ros_debug_tools::publish_debug_tf_xyz<point_type>(br, tf_name, pt, target_frame_id); \
}


#define PUBLISH_DEBUG_STRING_FOR_POINTCLOUD(node_handle, topic_name, message, point, point_type, frame_id) \
  {                                                                     \
    static ros::Publisher aginika_ros_debug_tools_publisher = (node_handle).advertise<visualization_msgs::MarkerArray>((topic_name),1); \
    aginika_ros_debug_tools::publish_string_for_pc<point_type>((aginika_ros_debug_tools_publisher), (message), (point), (frame_id)); \
      }


namespace aginika_ros_debug_tools
{
  template<typename PointT>
  void publish_pointcloud_normal(ros::NodeHandle& n, std::string topic_name, typename pcl::PointCloud<PointT>::Ptr& cloud_normals,std::string frame_id, std::string name_space=std::string("debug_cloud_normal"), double length_rate=5.0,int target_counter_times = 20)
  {
    static ros::Publisher aginika_ros_debug_tools_publisher = n.advertise<visualization_msgs::MarkerArray>(topic_name,1);
    if(cloud_normals->points.size()){
      visualization_msgs::MarkerArray marker_array;
      for(int i = 0; i < cloud_normals->points.size();i++){
        if(i % target_counter_times == 0 ){
          PointT cloud_normal = cloud_normals->points[i];
          visualization_msgs::Marker marker;
          marker.header.frame_id = frame_id;
          marker.header.stamp = ros::Time::now();
          marker.ns = name_space;
          marker.id = i;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;

          geometry_msgs::Point start_point;
          start_point.x = cloud_normal.x;
          start_point.y = cloud_normal.y;
          start_point.z = cloud_normal.z;

          //add Start point
          marker.points.push_back(start_point);

          geometry_msgs::Point end_point;
          end_point.x = cloud_normal.x + cloud_normal.normal_x/length_rate;
          end_point.y = cloud_normal.y + cloud_normal.normal_y/length_rate;
          end_point.z = cloud_normal.z + cloud_normal.normal_z/length_rate;

          //add Start point
          marker.points.push_back(end_point);

          marker.scale.x = 0.03/(length_rate/2);
          marker.scale.y = 0.07/(length_rate/2);
          //          marker.scale.z = 0.01;


          Eigen::Vector3f cloud_normal_vec(cloud_normal.normal_x, cloud_normal.normal_y, cloud_normal.normal_z);
          cloud_normal_vec = cloud_normal_vec.normalized();

          marker.color.a = 1.0;
          //1 /2 is for setting value 0 ~ 1
          marker.color.r = (cloud_normal_vec.dot(Eigen::Vector3f(1,0,0))+ 1) / 2;
          marker.color.g = (cloud_normal_vec.dot(Eigen::Vector3f(0,1,0)) + 1) / 2 ;
          marker.color.b = (cloud_normal_vec.dot(Eigen::Vector3f(0,0,1)) + 1) / 2;

          marker.lifetime = ros::Duration(1.0);
          marker_array.markers.push_back( marker );
        }
      }
      aginika_ros_debug_tools_publisher.publish( marker_array );
    }
  }


  template<typename PointT>
  void publish_pointcloud_normal(ros::Publisher aginika_ros_debug_tools_publisher, typename pcl::PointCloud<PointT>::Ptr& cloud_normals,std::string frame_id, std::string name_space=std::string("debug_cloud_normal"), double length_rate=5.0,int target_counter_times = 2)
  {
    if(cloud_normals->points.size()){
      visualization_msgs::MarkerArray marker_array;
      for(int i = 0; i < cloud_normals->points.size();i++){
        if(i % target_counter_times == 0 ){
          PointT cloud_normal = cloud_normals->points[i];
          visualization_msgs::Marker marker;
          marker.header.frame_id = frame_id;
          marker.header.stamp = ros::Time::now();
          marker.ns = name_space;
          marker.id = i;
          marker.type = visualization_msgs::Marker::ARROW;
          marker.action = visualization_msgs::Marker::ADD;

          geometry_msgs::Point start_point;
          start_point.x = cloud_normal.x;
          start_point.y = cloud_normal.y;
          start_point.z = cloud_normal.z;

          //add Start point
          marker.points.push_back(start_point);

          geometry_msgs::Point end_point;
          end_point.x = cloud_normal.x + cloud_normal.normal_x/length_rate;
          end_point.y = cloud_normal.y + cloud_normal.normal_y/length_rate;
          end_point.z = cloud_normal.z + cloud_normal.normal_z/length_rate;

          //add Start point
          marker.points.push_back(end_point);

          marker.scale.x = 0.03/(length_rate/2);
          marker.scale.y = 0.07/(length_rate/2);
          //          marker.scale.z = 0.01;


          Eigen::Vector3f cloud_normal_vec(cloud_normal.normal_x, cloud_normal.normal_y, cloud_normal.normal_z);
          cloud_normal_vec = cloud_normal_vec.normalized();

          marker.color.a = 1.0;
          //1 /2 is for setting value 0 ~ 1
          marker.color.r = (cloud_normal_vec.dot(Eigen::Vector3f(1,0,0))+ 1) / 2;
          marker.color.g = (cloud_normal_vec.dot(Eigen::Vector3f(0,1,0)) + 1) / 2 ;
          marker.color.b = (cloud_normal_vec.dot(Eigen::Vector3f(0,0,1)) + 1) / 2;

          marker.lifetime = ros::Duration(2.0);
          marker_array.markers.push_back( marker );
        }
      }
      aginika_ros_debug_tools_publisher.publish( marker_array );
    }
  }



  template<typename PointT>
  void publish_pointcloud(ros::Publisher &aginika_ros_debug_tools_publisher, typename pcl::PointCloud<PointT>::Ptr& cloud,std::string frame_id)
  {
    sensor_msgs::PointCloud2 pointcloud2;
    pcl::toROSMsg(*cloud, pointcloud2);
    pointcloud2.header.frame_id = frame_id;
    pointcloud2.header.stamp = ros::Time::now();
    aginika_ros_debug_tools_publisher.publish(pointcloud2);
  }

  template<typename Point>
  void publish_debug_tf(tf::TransformBroadcaster &br, std::string tf_name, Point pt, std::string target_frame_id){
    tf::Transform _transform;
    _transform.setOrigin(tf::Vector3(pt[0], pt[1], pt[2]));
    _transform.setRotation(tf::createIdentityQuaternion());
    br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                                          target_frame_id, tf_name));
  }

  template<typename Point>
  void publish_debug_tf_xyz(tf::TransformBroadcaster &br, std::string tf_name, Point pt, std::string target_frame_id){
    tf::Transform _transform;
    _transform.setOrigin(tf::Vector3(pt.x, pt.y, pt.z));
    _transform.setRotation(tf::createIdentityQuaternion());
    br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(),
                                          target_frame_id, tf_name));
  }


  template<typename PointT>
  void publish_string_for_pc(ros::Publisher aginika_ros_debug_tools_publisher, std::string ss, PointT min_pt,std::string frame_id, std::string name_space=std::string("debug_string_for_pc"),int target_counter_times = 2)
  {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = name_space;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = min_pt.x;
    marker.pose.position.y = min_pt.y;
    marker.pose.position.z = min_pt.z;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.text = ss;
    marker.lifetime = ros::Duration();
    marker_array.markers.push_back( marker );
    aginika_ros_debug_tools_publisher.publish( marker_array );
  }
}

#endif
