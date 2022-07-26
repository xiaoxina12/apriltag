/*
'''
@Time    : 2022/07/27 13:49
@Author  : ZHANG WEN HAO
@Contact : 821298794@qq.com
@Version : 0.1
@Language: c++
@Desc    : d430和usb相机整合在一起，apriltag的数据调用 */


#include "apriltag_detect_node.h"



void tag_detection(const cv_bridge::CvImagePtr& cv_src)
{
  result = tag_detector_->detectTags(cv_src,cameraInterMat);
  if(result.detections.size()!=0)
  {
    center_cx = result.detections[0].pose.pose.pose.position.x;
    center_cz = result.detections[0].pose.pose.pose.position.z;
    center_wx = result.detections[0].pose.pose.pose.orientation.x;
    cout<<"center_cx:"<<center_cx<<endl;
    cout<<"center_cz:"<<center_cz<<endl;
    cout<<"center_wx:"<<center_wx<<endl;
  }

}


void pixel2cam( float x, float y , float depth)
{
  point = (Mat_<float>(3,1)<<x*depth,y*depth,1*depth);
  point_world =  cameraInterMat_T* point;
}

//d430回调
void imageDepthCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth)
{
  try
  {
    cv::Mat cv_src;//消息类型转为opencv格式
    cv_bridge::CvImageConstPtr pCvImage;
    pCvImage = cv_bridge::toCvShare(imageColor, "rgb8");
    pCvImage->image.copyTo(cv_src);

    cv::Mat depth;
    cv_bridge::CvImageConstPtr pCvDepth;
    pCvDepth = cv_bridge::toCvCopy(imageDepth,sensor_msgs::image_encodings::TYPE_32FC1);
    depth = pCvDepth->image;  
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from 'msg' to 'rgb8'.");
  }
}


 void imageCallback(const sensor_msgs::Image::ConstPtr& msg) //usb相机部分回调函数
 {
   try
   {
      cv_bridge::CvImagePtr pCvImage;
      pCvImage = cv_bridge::toCvCopy(msg, "rgb8");
      tag_detection(pCvImage);
      std_msgs::Float32MultiArray message; //发布d430数据
      message.data.push_back(center_cx);
      message.data.push_back(center_cz);
      message.data.push_back(center_wx);
      pub_corner.publish(message);  
    }
    catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
   }
 }

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "detect_node");
  ros::NodeHandle nh("~");
  // tag_detector_ = apriltag_ros::TagDetector(nh);
  tag_detector_ = std::shared_ptr<apriltag_ros::TagDetector>(new apriltag_ros::TagDetector(nh));
  YAML::Node config = YAML::LoadFile(config_path);
  vector<float> inter = config["cameraMatrix"]["data"].as<vector<float>>();
  for (int row = 0; row < 3; row++)
      {
          for (int col = 0; col < 3; col++)
          {
              cameraInterMat.at<float>(row, col) = inter[col + row * 3];
          }
      }
  cv::invert(cameraInterMat,cameraInterMat_T,DECOMP_LU);

    //d430部分
  pub_corner= nh.advertise<std_msgs::Float32MultiArray>("/corner_data", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_image(nh,"/camera/infra1/image_rect_raw",1);
  message_filters::Subscriber<sensor_msgs::Image> sub_depth(nh,"/camera/depth/image_rect_raw",1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
  message_filters::Synchronizer<sync_policy> syncpolicy(sync_policy(10),sub_image,sub_depth);
  syncpolicy.registerCallback(boost::bind(&imageDepthCallback,_1,_2));

  //usb相机使用
  image_transport::ImageTransport it(nh);
  img_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
  ros::Rate loop_rate(100);

  while(nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  }  
  return 0;
}

