
#include "features.h"


void secondNNTest(std::vector<std::vector<cv::DMatch> > &matches, std::vector<cv::DMatch> &goodMatches)
{
  for (std::vector<std::vector<cv::DMatch> >::iterator
      matchIterator= matches.begin();
      matchIterator!= matches.end(); ++matchIterator) {
    // if 2 NN has been identified
    if (matchIterator->size() > 1) {
      // check distance ratio
      if ((*matchIterator)[0].distance <
         (*matchIterator)[1].distance*0.8) {
        //matchIterator->clear(); // remove match
        goodMatches.push_back((*matchIterator)[0]);
        //if ((*matchIterator)[0].distance < 0.35)
        //ROS_INFO("dist1: %f", (*matchIterator)[0].distance);
        //ROS_INFO("dist2: %f", (*matchIterator)[1].distance);
        //removed++;
      }
    } else { // does not have 2 neighbours
      //matchIterator->clear(); // remove match
    }
  }
}

void matchesTest(std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &goodMatches)
{
  // Filter out good matches to return, where
  // distance must be less than 2*min_dist.
  double max_dist = 0; double min_dist = 100;

  for(std::vector<cv::DMatch>::size_type i = 0; i < matches.size(); i++)
  {
    ROS_INFO("trainIdx %d, imgIdx %d", matches[i].trainIdx, matches[i].imgIdx);
    double dist = matches[i].distance;
    if (dist < min_dist) min_dist = dist;
    if (dist > max_dist) max_dist = dist;
  }

  //double distRad = 2*min_dist < 0.3 ? 0.3 : 2*min_dist;
  double distRad = 2*min_dist;
  //double distRad = 0.35;

  for(std::vector<cv::DMatch>::size_type i = 0; i < matches.size(); i++)
  { 
    if(matches[i].distance < distRad)
    { 
      goodMatches.push_back(matches[i]);
    }
  }

}

void filterKeyPointsFromMask(std::vector<cv::KeyPoint> &keypoints, cv::Mat &mask)
{
  for (std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); it != keypoints.end();)
  {
    if (mask.at<uchar>(it->pt.y, it->pt.x) == 0)
    {
      it = keypoints.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void ransacTest(
  std::vector<cv::DMatch> &matches,
  int imgIdx,
  std::vector<cv::KeyPoint> &keypoints1,
  std::vector<cv::KeyPoint> &keypoints2,
  std::vector<cv::DMatch> &goodMatches
)
{
  std::vector<cv::Point2f> points1, points2;
  std::vector<cv::DMatch> imgMatches;

  for (std::vector<cv::DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it) {
    if (it->imgIdx == imgIdx)
    {
      points1.push_back(cv::Point2f(
          keypoints1[it->queryIdx].pt.x,
          keypoints1[it->queryIdx].pt.y
      ));
      points2.push_back(cv::Point2f(
          keypoints2[it->trainIdx].pt.x,
          keypoints2[it->trainIdx].pt.y
      ));

      imgMatches.push_back(*it);
    }
  }
  if (points1.size() == 0)
    return;
  double confidence = 0.99;
  double distance = 3.0;

  std::vector<uchar> inliers(points1.size(), 0);
  cv::Mat fundemental = cv::findFundamentalMat(
    cv::Mat(points1),cv::Mat(points2), // matching points
    inliers,      // match status (inlier ou outlier)  
    CV_FM_RANSAC, // RANSAC method
    distance,     // distance to epipolar line
    confidence    // confidence probability
  );

  // extract the surviving (inliers) matches
  std::vector<uchar>::const_iterator itIn = inliers.begin();
  std::vector<cv::DMatch>::const_iterator itM = imgMatches.begin();

  // for all matches
  for ( ;itIn!= inliers.end(); ++itIn, ++itM) 
  {
    if (*itIn)
    { // it is a valid match
      goodMatches.push_back(*itM);
    }
  }
}
