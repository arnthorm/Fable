
#include <ros/ros.h>
#include "depth.h"

int histSize[] = {256};
const float rangeMax = 10.0;

cv::Mat drawHistogram(cv::MatND &hist)
{
  // Get min and max bin values
  double maxVal = 0;
  double minVal = 0;
  cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);

  // Image on which to display histogram
  cv::Mat histImg(
    histSize[0], 
    histSize[0],
    CV_8U,
    cv::Scalar(255)
  );

  // set highest point at 90% of nbins
  int hpt = static_cast<int>(0.9*histSize[0]);
  // Draw a vertical line for each bin
  for( int h = 0; h < histSize[0]; h++ ) {
    float binVal = hist.at<float>(h);
    int intensity = static_cast<int>(binVal*hpt/maxVal);
    // This function draws a line between 2 points
    cv::line(histImg,cv::Point(h,histSize[0]),
    cv::Point(h,histSize[0]-intensity),
    cv::Scalar::all(0));
  }

  return histImg;
}

cv::MatND getHistogram(const cv::Mat &depth)
{
  cv::MatND hist;

  float max = 0;
  float min = 100;
    
  for (int i = 0; i < depth.rows; i++)
  {
    for (int j = 0; j < depth.cols; j++)
    {
      float value = depth.at<float>(i, j);
      if (value < min)
        min = value;
      else if (value > max)
        max = value;
    }
  }
  //ROS_INFO("min %f", min);
  //ROS_INFO("max %f", max);

  int channels[] = {0};
  float hranges[] = {0.0, rangeMax}; 
  const float* ranges[] = {hranges};

  cv::calcHist(
    &depth, 
    1,
    channels,
    cv::Mat(),
    hist,
    1,
    histSize,
    ranges
  );

  return hist;
}

void getSegmentThresholds(cv::MatND hist, float value, float &upper, float &lower)
{
  int idx = value*histSize[0]/rangeMax;
  int highIdx = 0;
  int lowIdx = 0;

  if (hist.at<float>(idx) != 0)
  {
    lowIdx = idx;
    for (int i = idx; 0 <= i; i--)
    {
      if (hist.at<float>(i) == 0)
        break;
      lowIdx = i;
    }

    highIdx = idx;
    for (int i = idx; i < histSize[0]; i++)
    {
      if (hist.at<float>(i) == 0)
        break;         
      highIdx = i;
    }
  }

  upper = highIdx*rangeMax/histSize[0];
  lower = lowIdx*rangeMax/histSize[0];
}

void threshold(cv::Mat &src, cv::Mat &dest, float upper, float lower)
{
  dest = cv::Mat(src.rows, src.cols, CV_8UC1);
  for (int i = 0; i < src.rows; ++i)
  {
    for (int j = 0; j < src.cols; ++j)
    {
      float value = src.at<float>(i,j);
      if (lower <= value && value <= upper)
        dest.at<uchar>(i,j) = 255;
      else
        dest.at<uchar>(i,j) = 0;
    }
  }
}

void getMaskFromDepth(cv::Mat &src, cv::Mat &dest, float depthValue)
{
  cv::MatND hist = getHistogram(src);
  float upper, lower;
  getSegmentThresholds(hist, depthValue, upper, lower);
  cv::Mat mask;
  threshold(src, mask, upper, lower);

  int morph_size = 4;
  cv::Mat element = cv::getStructuringElement( 0, cv::Size( 2*morph_size + 1, 2*morph_size+1 ), cv::Point( morph_size, morph_size ) ); 
  morphologyEx(mask, dest, cv::MORPH_CLOSE, element);
}

