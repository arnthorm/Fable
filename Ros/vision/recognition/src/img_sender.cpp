
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <recognition/RecognizeImage.h>
#include <recognition/InsertImage.h>

static const char WINDOW[] = "Image window";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "img_sender");
  ros::NodeHandle nh_;
  if (argc < 3)
  {
    ROS_ERROR("Usage: img_sender database recognize|insert image_path [mask_path] [label]");
    return 1;
  }

  std::string database = argv[1];
  // Load image
  cv::Mat img = cv::imread(argv[3]);
  if (img.empty())
  {
    ROS_ERROR("Could not load filename: %s", argv[3]);
    return 1;
  }
  cv_bridge::CvImage cvImg;
  cvImg.header.stamp = ros::Time::now();
  cvImg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  cvImg.image = img;

  // Load mask
  cv::Mat mask;
  cv_bridge::CvImage cvMask;
  if (argc >= 5)
  {
    mask = cv::imread(argv[4], 0);
    cvMask.header.stamp = ros::Time::now();

    if (!mask.empty())
    {
      cvMask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      cvMask.image = mask;
    }
    else
    {
      cvMask.image = cv::Mat();
    }
  }
  if ((argc > 3) && std::strcmp("recognize", argv[2]) == 0)
  {
    ros::ServiceClient client = nh_.serviceClient<recognition::RecognizeImage>(
      "/recognize/" + database + "/image"
    );
    recognition::RecognizeImage srv;
    srv.request.image = *cvImg.toImageMsg();
    if (!mask.empty())
    {
      srv.request.mask = *cvMask.toImageMsg();
    }
    ros::Time begin = ros::Time::now();
    if(client.call(srv))
    {
      ros::Duration dt = ros::Time::now() - begin;
      ROS_INFO("This was recognized as \"%s\"", srv.response.label.data.c_str());
      ROS_INFO("Time: %f", dt.toSec());
    }
    else
    {
      ROS_ERROR("Failed to communicate with service!");
      return 1;
    }
  }
  else if (argc == 5 && argv[2] == "insert")
  {
    ros::ServiceClient client = nh_.serviceClient<recognition::InsertImage>(
      "/recognize/" + database + "/insert_image"
    );
    recognition::InsertImage srv;

    if (!mask.empty())
    {
      srv.request.label = argv[5];
      srv.request.mask = *cvMask.toImageMsg();
    }
    else
    {
      srv.request.label = argv[4];
    }
    srv.request.image = *cvImg.toImageMsg();
    if(client.call(srv))
    {
      if (srv.response.success)
        ROS_INFO("The image was set into the database.");
      else
        ROS_INFO("For some reason the image was not set into database.");
    }
    else
    {
      ROS_ERROR("Failed to communicate with service!");
      return 1;
    }
  }
  else
  {
    ROS_ERROR("Something went wrong here!");
  }

 
  return 0;
}
