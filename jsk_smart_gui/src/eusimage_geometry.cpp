#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <pthread.h>
#include <setjmp.h>
#include <errno.h>

#include <list>
#include <vector>
#include <set>
#include <string>
#include <map>
#include <sstream>

#include <cstdio>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/init.h>
#include <ros/time.h>
#include <ros/rate.h>
#include <ros/master.h>
#include <ros/this_node.h>
#include <ros/node_handle.h>
#include <ros/serialization.h>

// for eus.h
#define class   eus_class
#define throw   eus_throw
#define export  eus_export
#define vector  eus_vector
#define string  eus_string

#include "eus.h"
extern "C" {
  pointer ___eusimage_geometry(register context *ctx, int n, pointer *argv, pointer env);
  void register_eusimage_geometry(){
    char modname[] = "___eusimage_geometry";
    return add_module_initializer(modname, (pointer (*)())___eusimage_geometry);}
}

#undef class
#undef throw
#undef export
#undef vector
#undef string

#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>

pointer EUSPINHOLE_CAMERA_MODEL(register context *ctx,int n,pointer *argv)
{
  return (makeint((eusinteger_t)(new image_geometry::PinholeCameraModel())));
}

pointer EUSPINHOLE_CAMERA_MODEL_DISPOSE(register context *ctx,int n,pointer *argv)
{
  ckarg(1);
  image_geometry::PinholeCameraModel *pcm = (image_geometry::PinholeCameraModel *)(intval(argv[0]));
  delete(pcm);
  return(T);
}

pointer EUSPINHOLE_CAMERA_MODEL_FROM_CAMERA_INFO(register context *ctx,int n,pointer *argv)
{
  ckarg(3);

  uint8_t* serialized_camera_info = (uint8_t *)(argv[1]->c.str.chars);
  uint32_t serialization_length = intval(argv[2]);
  image_geometry::PinholeCameraModel *pcm = (image_geometry::PinholeCameraModel *)(intval(argv[0]));

  sensor_msgs::CameraInfo camera_info;
  boost::shared_array<uint8_t> buffer(new uint8_t[serialization_length]);

  for(int i=0; i<serialization_length; ++i){
    buffer[i] = serialized_camera_info[i];
  }

  ros::serialization::IStream stream(buffer.get(), serialization_length);
  ros::serialization::Serializer<sensor_msgs::CameraInfo>::read(stream, camera_info);

  pcm->fromCameraInfo(camera_info);
  return (T);
}

pointer EUSPINHOLE_CAMERA_MODEL_PROJECT_PIXEL_TO_3DRAY(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  image_geometry::PinholeCameraModel *pcm = (image_geometry::PinholeCameraModel *)(intval(argv[0]));
  if(!isvector(argv[1])) error(E_NOVECTOR);
  eusfloat_t *pixel = argv[1]->c.fvec.fv;

  cv::Point2d uv = cv::Point2d(pixel[0], pixel[1]);
  cv::Point3d xyz = pcm->projectPixelTo3dRay(uv);

  pointer vs = makefvector(3);
  vpush(vs);
  vs->c.fvec.fv[0] = xyz.x;
  vs->c.fvec.fv[1] = xyz.y;
  vs->c.fvec.fv[2] = xyz.z;
  vpop();
  return(vs);
}

pointer EUSPINHOLE_CAMERA_MODEL_PROJECT_3D_TO_PIXEL(register context *ctx,int n,pointer *argv)
{
  ckarg(2);
  image_geometry::PinholeCameraModel *pcm = (image_geometry::PinholeCameraModel *)(intval(argv[0]));
  if(!isvector(argv[1])) error(E_NOVECTOR);
  eusfloat_t *pos = argv[1]->c.fvec.fv;

  cv::Point3d xyz = cv::Point3d(pos[0]/1000.0, pos[1]/1000.0, pos[2]/1000.0);
  cv::Point2d uv = pcm->project3dToPixel(xyz);

  pointer vs = makefvector(2);
  vpush(vs);
  vs->c.fvec.fv[0] = uv.x;
  vs->c.fvec.fv[1] = uv.y;
  vpop();
  return(vs);
}

pointer ___eusimage_geometry(register context *ctx, int n, pointer *argv, pointer env)
{
  defun(ctx,"EUSPINHOLE-CAMERA-MODEL",argv[0],(pointer (*)())EUSPINHOLE_CAMERA_MODEL, NULL);
  defun(ctx,"EUSPINHOLE-CAMERA-MODEL-DISPOSE",argv[0],(pointer (*)())EUSPINHOLE_CAMERA_MODEL_DISPOSE, NULL);
  defun(ctx,"EUSPINHOLE-CAMERA-MODEL-FROM-CAMERA-INFO",argv[0],(pointer (*)())EUSPINHOLE_CAMERA_MODEL_FROM_CAMERA_INFO, NULL);
  defun(ctx,"EUSPINHOLE-CAMERA-MODEL-PROJECT-PIXEL-TO-3DRAY",argv[0],(pointer (*)())EUSPINHOLE_CAMERA_MODEL_PROJECT_PIXEL_TO_3DRAY, NULL);
  defun(ctx,"EUSPINHOLE-CAMERA-MODEL-PROJECT-3D-TO-PIXEL",argv[0],(pointer (*)())EUSPINHOLE_CAMERA_MODEL_PROJECT_3D_TO_PIXEL, NULL);

  return 0;
}
