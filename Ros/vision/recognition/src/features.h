
#include <ros/ros.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>

void secondNNTest(std::vector<std::vector<cv::DMatch> > &matches, std::vector<cv::DMatch> &goodMatches);
void matchesTest(std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &goodMatches);
//void matchesTest();
void filterKeyPointsFromMask(std::vector<cv::KeyPoint> &keypoints, cv::Mat &mask);

void ransacTest(
  std::vector<cv::DMatch> &matches,
  int imgIdx,
  std::vector<cv::KeyPoint> &keypoints1,
  std::vector<cv::KeyPoint> &keypoints2,
  std::vector<cv::DMatch> &goodMatches
);
