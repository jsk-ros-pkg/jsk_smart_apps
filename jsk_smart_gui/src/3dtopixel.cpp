#include <ros/ros.h>
#include <opencv/cv.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include "jsk_smart_gui/point2screenpoint.h"

class TransformPixel
{
  ros::NodeHandle nh_;
  ros::Subscriber camerainfosub_;
  ros::ServiceServer srv_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;

public:
  TransformPixel()
  {
    camerainfosub_ = nh_.subscribe<sensor_msgs::CameraInfo>(nh_.resolveName("camera_info"), 100, &TransformPixel::camerainfoCb, this);
    srv_ = nh_.advertiseService("TransformPixelRequest", &TransformPixel::objectSrv,this);
  }

  bool objectSrv(jsk_smart_gui::point2screenpoint::Request &req,
                 jsk_smart_gui::point2screenpoint::Response &res)
  {
    ROS_INFO("3dtopixel request:x=%lf,y=%lf,z=%lf",req.point.point.x,req.point.point.y,req.point.point.z);
    std::string camera_topic = nh_.resolveName("camera_topic");
    geometry_msgs::PointStamped point_transformed;
    tf_listener_.transformPoint(camera_topic, req.point, point_transformed);
    cv::Point3d xyz; cv::Point2d uv_rect;
    xyz.x = point_transformed.point.x;
    xyz.y = point_transformed.point.y;
    xyz.z = point_transformed.point.z;
    cam_model_.project3dToPixel(xyz,uv_rect);
    res.x=uv_rect.x;res.y=uv_rect.y;
    return true;
  }

  void camerainfoCb(const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    ROS_INFO("infocallback :shutting down camerainfosub");
    cam_model_.fromCameraInfo(info_msg);
    camerainfosub_.shutdown();
  }
};

int
main (int argc, char** argv)
{
  ros::init(argc,argv,"3dtopixel");
  TransformPixel r;
  ros::spin();
}

