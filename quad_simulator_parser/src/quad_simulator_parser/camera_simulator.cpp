#include "quad_simulator_parser/camera_simulator.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf_conversions/tf_eigen.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

CameraSimulator::CameraSimulator(ros::NodeHandle nh, ros::NodeHandle nh_private) :
  nh(nh)
{
  if (!nh_private.getParam ("world_frame_name", world_frame_name))
    world_frame_name = "world";
  if (!nh_private.getParam ("body_frame_name", body_frame_name))
    body_frame_name = "body";
  if (!nh_private.getParam ("model_name", model_name))
    model_name = "hackerman2.mesh";
  if (!nh_private.getParam ("resource_path", resource_path))
    resource_path = "./";
                
  CameraRenderApplication *renderer = new CameraRenderApplication(resource_path);
  renderer->go();
  renderer->loadModel("model", "hackerman2.mesh");
  vih = new VirtualImageHandler(renderer);
  K = vih->getCameraIntrinsics();

  cam_transform << 0,  0, 1, 0,
                   -1, 0, 0, 0,
                   0, -1, 0, 0,
                   0, 0, 0, 1;

  image_pub = nh.advertise<sensor_msgs::Image>("/camera_sim/image",1);
  cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera_sim/camera_info",1);
  depth_pub = nh.advertise<sensor_msgs::Image>("/camera_sim/depth_image",1);

  camera_timer =  nh.createTimer(ros::Rate(20), &CameraSimulator::cameraRenderCallback, this);
  camera_timer.start();
}

void CameraSimulator::cameraRenderCallback(const ros::TimerEvent&)
{
  tf::StampedTransform start_tf;
  start_tf.setOrigin(tf::Vector3(0,0,0));
  start_tf.setRotation(tf::Quaternion(0,0,0,1));
  try
  {
    bool result = tflistener.waitForTransform(world_frame_name, body_frame_name,
                    ros::Time(0), ros::Duration(1.0));
    tflistener.lookupTransform(world_frame_name, body_frame_name,
      ros::Time(0), start_tf);
  }
  catch(tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  Eigen::Quaterniond q_eig;
  tf::quaternionTFToEigen(start_tf.getRotation(), q_eig);  
  Eigen::Quaterniond q_img(q_eig.toRotationMatrix()*cam_transform.topLeftCorner<3,3>());                 

  Mat img, depth;
  vih->getVirtualImageAndDepth(img, depth, start_tf.getOrigin().x(), start_tf.getOrigin().y(),
     start_tf.getOrigin().z(), q_img.w(), q_img.x(), q_img.y(), q_img.z());
  ros::Time stamp = ros::Time::now();

  cv_bridge::CvImage cv_img;
  cv_img.image = img;
  cv_img.encoding = "mono8";
  cv_img.header.stamp = stamp;
  cv_img.header.frame_id = "camera";
  image_pub.publish(cv_img.toImageMsg());

  sensor_msgs::CameraInfo cam_info_msg;
  cam_info_msg.header.stamp = stamp;
  cam_info_msg.header.frame_id = "camera";
  cam_info_msg.height = img.rows;
  cam_info_msg.width = img.cols;
  cam_info_msg.distortion_model = "blumb_bob";
  cam_info_msg.D.resize(5,0);
  cam_info_msg.K[0] =  K.at<double>(0,0);
  cam_info_msg.K[1] = 0;
  cam_info_msg.K[2] =  K.at<double>(0,2);
  cam_info_msg.K[3] = 0;
  cam_info_msg.K[4] =  K.at<double>(1,1);
  cam_info_msg.K[5] =  K.at<double>(1,2);
  cam_info_msg.K[6] = 0;
  cam_info_msg.K[7] = 0;
  cam_info_msg.K[8] = 1;
  cam_info_pub.publish(cam_info_msg);

  publishDepthMat(depth, stamp);
}

void CameraSimulator::publishDepthMat(const Mat& depth, ros::Time stamp)
{
  static const float bad_point = std::numeric_limits<float>::quiet_NaN ();

  sensor_msgs::ImagePtr image(new sensor_msgs::Image);
  image->header.stamp = stamp;
  image->header.frame_id = "camera";

  image->width = depth.cols;
  image->height = depth.rows;
  image->is_bigendian = 0;
  image->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image->step = sizeof(float)*depth.cols;
  image->data.resize(depth.cols*depth.rows*sizeof(float));

  const float* in_ptr = reinterpret_cast<const float*>(&depth.data[0]);
  float* out_ptr = reinterpret_cast<float*>(&image->data[0]);
  for(int i = 0; i < depth.cols*depth.rows; i++, in_ptr++, out_ptr++)
  {
    if(*in_ptr == 0)
    {
      *out_ptr = bad_point;
    }
    else
    {
      *out_ptr = *in_ptr;
    }
  }
  depth_pub.publish(image);
}

