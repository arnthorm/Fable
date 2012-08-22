
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <recognition/RecognizeFromTopic.h>
#include <recognition/RecognizeImage.h>
#include <recognition/RecognizeImageTest.h>
#include <recognition/InsertImage.h>
#include <recognition/InsertTopic.h>
#include <recognition/RectFromTopic.h>
#include "Surf.h"
#include "depth.h"
#include "helpers.h"

namespace enc = sensor_msgs::image_encodings;

static const char PACKAGE[] = "recognition";
static const char WINDOW[] = "Image window";

class Recognition
{
  ros::NodeHandle nh_;
  Surf *algorithm;
  bool debug;
  std::string imageTopic;
  std::string depthTopic;

  ros::ServiceServer srvImage;
  ros::ServiceServer srvImageTest;
  ros::ServiceServer srvInsertImg;
  ros::ServiceServer srvInsertTopic;
  ros::ServiceServer srvFromTopic;
  ros::ServiceServer srvRectFromTopic;
  ros::Subscriber subImage;
  ros::Subscriber subDepth;

  public:
    std::string database;
    static bool topicRegistered;
    static cv_bridge::CvImagePtr lastImage;
    static cv_bridge::CvImagePtr lastDepth;

    Recognition(ros::NodeHandle &nh, std::string database): nh_(nh), debug(false), database(database)
    {
      nh_.param<std::string>("image_topic", imageTopic, "");
      nh_.param<std::string>("depth_topic", depthTopic, "");
      nh_.param<bool>("debug", debug, false);
      if (debug)
        ROS_INFO("Recognition is in debug mode.");

      std::string databasePath = ros::package::getPath(PACKAGE) + "/databases/" + database;
      algorithm = new Surf(databasePath, debug);

      std::string baseNamespace = "/recognize/" + database;

      srvImage = nh_.advertiseService(
          baseNamespace + "/image", 
          &Recognition::recognizeImage, 
          this
        );

      srvImageTest = nh_.advertiseService(
          baseNamespace + "/image_test", 
          &Recognition::recognizeImageTest, 
          this
        );


      srvInsertImg = nh_.advertiseService(
          baseNamespace + "/insert_image", 
          &Recognition::insertImage, 
          this
        );

      if (imageTopic != "")
      {
        srvFromTopic = nh_.advertiseService(
          baseNamespace + "/from_topic",
          &Recognition::recognizeFromTopic,
          this
        );
        srvInsertTopic = nh_.advertiseService(
          baseNamespace + "/insert_topic", 
          &Recognition::insertTopic, 
          this
        );

        if (!Recognition::topicRegistered)
        {

          srvRectFromTopic = nh_.advertiseService(
            imageTopic + "/get_rect",
            &Recognition::getRectFromTopic,
            this
          );

          Recognition::topicRegistered = true;
          subImage = nh_.subscribe(
              imageTopic,
              1,
              &Recognition::imageCallback,
              this
          );
          if (depthTopic != "")
          {
            subDepth = nh_.subscribe(
                depthTopic, 
                1, 
                &Recognition::depthCallback, 
                this
            );
          }
        }
      }

      if (debug)
      {
        cv::namedWindow(WINDOW);
        cv::startWindowThread();
      }
    }

    ~Recognition()
    {
      if (debug)
      {
        cv::destroyWindow(WINDOW);
      }
    }

    void insert(cv::Mat &image, std::string label)
    {
      cv::Mat mask;
      algorithm->insertIntoDb(image, label, mask);
    }

    bool recognizeImage(
      recognition::RecognizeImage::Request &request,
      recognition::RecognizeImage::Response &response
    ) 
    {
      ROS_INFO("Database: %s", database.c_str());
      cv_bridge::CvImagePtr cv_ptr;
      if (!_getImageFromMessage(request.image, cv_ptr))
        return false;

      if (request.mask.height > 0)
      {
        cv_bridge::CvImagePtr mask_ptr;
        if (!_getImageFromMessage(request.mask, mask_ptr))
          return false;

        response.label.data = algorithm->recognize(cv_ptr->image, mask_ptr->image);
      }
      else
      {
        cv::Mat mask;
        response.label.data = algorithm->recognize(cv_ptr->image, mask);
      }
      
      return true;
    }

    bool recognizeImageTest(
      recognition::RecognizeImageTest::Request &request,
      recognition::RecognizeImageTest::Response &response
    ) 
    {
      ROS_INFO("Database: %s", database.c_str());
      cv_bridge::CvImagePtr cv_ptr;
      if (!_getImageFromMessage(request.image, cv_ptr))
        return false;

      if (request.mask.height > 0)
      {
        cv_bridge::CvImagePtr mask_ptr;
        if (!_getImageFromMessage(request.mask, mask_ptr))
          return false;

        response.label.data = algorithm->recognize(cv_ptr->image, mask_ptr->image);
      }
      else
      {
        cv::Mat mask;
        response.label.data = algorithm->recognize(cv_ptr->image, mask);
      }
      response.testNumber = algorithm->testNumber;
      
      return true;
    }


    cv::Rect safeRect(cv::Mat &img, int x, int y, int width, int height)
    {
      int x1 = x + width;
      int y1 = y + height;
      if (x < 0)
        x = 0;
      if (y < 0)
        y = 0;
      if (x > img.cols-2)
        x = img.cols - 2;
      if (y > img.rows-2)
        y = img.rows - 2;
      if (x1 < 0)
        x1 = 0;
      if (y1 < 0)
        y1 = 0;
      if (x1 > img.cols-1)
        x1 = img.cols - 1;
      if (y1 > img.rows-1)
        y1 = img.rows - 1;
      return cv::Rect(x, y, x1-x, y1-y);
    }

    bool recognizeFromTopic(
      recognition::RecognizeFromTopic::Request &request,
      recognition::RecognizeFromTopic::Response &response
    )
    {
      if (lastImage == NULL)
        return false;

      cv::Mat newImg = lastImage->image(
        safeRect(
          lastImage->image,
          request.x, 
          request.y, 
          request.width, 
          request.height
        )
      );

      cv::Mat mask;
      if (lastDepth != NULL)
      {
        cv::Mat newDepth = lastDepth->image(
          safeRect(
            lastDepth->image,
            request.x, 
            request.y, 
            request.width, 
            request.height
          )
        );
        _getMaskFromDepth(newDepth, mask);

      }
      response.label = algorithm->recognize(newImg, mask);
      return true;
    }

    bool getRectFromTopic(
      recognition::RectFromTopic::Request &request,
      recognition::RectFromTopic::Response &response
    )
    {
      if (lastImage == NULL)
        return false;

      cv_bridge::CvImage cvImg, cvMask;
      cvImg.header.stamp = ros::Time::now();
      cvImg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;

      cvImg.image = lastImage->image(
        safeRect(
          lastImage->image,
          request.x, 
          request.y, 
          request.width, 
          request.height
        )
      );
      response.image = *cvImg.toImageMsg();
      ROS_INFO("Image");
      cv::Mat mask;
      if (lastDepth != NULL)
      {
        ROS_INFO("Depth");
        cv::Mat newDepth = lastDepth->image(
          safeRect(
            lastDepth->image,
            request.x, 
            request.y, 
            request.width, 
            request.height
          )
        );
        _getMaskFromDepth(newDepth, mask);
        cvMask.header.stamp = ros::Time::now();
        cvMask.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
        cvMask.image = mask;
        response.mask = *cvMask.toImageMsg();
      }
      return true;
    }


    bool insertImage(
      recognition::InsertImage::Request &request,
      recognition::InsertImage::Response &response
    ) 
    {
      cv_bridge::CvImagePtr cv_ptr;
      if (!_getImageFromMessage(request.image, cv_ptr))
        return false;

      if (request.mask.height > 0)
      {
        cv_bridge::CvImagePtr mask_ptr;
        if (!_getImageFromMessage(request.mask, mask_ptr))
          return false;

        algorithm->insertIntoDb(cv_ptr->image, request.label, mask_ptr->image);
      }
      else
      {
        cv::Mat mask;
        algorithm->insertIntoDb(cv_ptr->image, request.label, mask);
      }

      response.success = true;
      return true;
    }

    bool insertTopic(
      recognition::InsertTopic::Request &request,
      recognition::InsertTopic::Response &response
    )
    {
      if (lastImage == NULL)
        return false;

      cv::Mat newImage = lastImage->image(
        safeRect(
          lastImage->image,
          request.x, 
          request.y, 
          request.width, 
          request.height
        )
      );

      cv::Mat mask;
      if (lastDepth != NULL)
      {
        cv::Mat newDepth = lastDepth->image(
          safeRect(
            lastDepth->image,
            request.x, 
            request.y, 
            request.width, 
            request.height
          )
        );
        _getMaskFromDepth(newDepth, mask);
      }

      algorithm->insertIntoDb(newImage, request.label, mask);

      response.success = true;

      return true;
    }

    void imageCallback(sensor_msgs::ImageConstPtr image)
    {
      _getImageFromMessage(image, Recognition::lastImage);
    }

    void depthCallback(sensor_msgs::ImageConstPtr depth)
    {
      _getImageFromMessage(depth, Recognition::lastDepth);
    }

  private:
    bool _getImageFromTopic(std::string topic, cv_bridge::CvImagePtr &cv_ptr)
    {
      sensor_msgs::ImageConstPtr img = ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh_, ros::Duration(2));
      if (img == NULL)
      {
        ROS_ERROR("Could not receive image from %s.", topic.c_str());
        return false;
      }

      return _getImageFromMessage(img, cv_ptr);
    }

    bool _getImageFromMessage(sensor_msgs::ImageConstPtr &image, cv_bridge::CvImagePtr &cv_ptr)
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, image->encoding);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }
      return true;
    }

    bool _getImageFromMessage(sensor_msgs::Image &image, cv_bridge::CvImagePtr &cv_ptr)
    {
      try
      {
        cv_ptr = cv_bridge::toCvCopy(image, image.encoding);
      }
      catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
      }
      return true;
    }

    void _getMaskFromDepth(cv::Mat &depth, cv::Mat &mask)
    {
      float depthValue = depth.at<float>(round(depth.rows/2), round(depth.cols/2));
      if (depthValue == depthValue)
      {
        getMaskFromDepth(depth, mask, depthValue);
      }
    }
};

bool Recognition::topicRegistered = false;
cv_bridge::CvImagePtr Recognition::lastImage;
cv_bridge::CvImagePtr Recognition::lastDepth;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "recognition");
  ros::NodeHandle nh("~");

  std::string database;
  nh.param<std::string>("database", database, "database");

  std::vector<std::string> list = split(database, ',');
  Recognition* rlist[list.size()];
  int i = 0;
  for (std::vector<std::string>::iterator it = list.begin();
      it != list.end(); ++it)
  {
    ROS_INFO("Loading database: %s", it->c_str());
    rlist[i] = new Recognition(nh, *it);
  }
  
  ros::spin();
  return 0;
}
