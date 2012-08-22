
#include "Surf.h"
#include "helpers.h"
#include "features.h"
#include "depth.h"

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <map>
#include <dirent.h>

static const char WINDOW[] = "Image debug";
static const std::string MASK_FIX = "mask_";

Surf::Surf(std::string databasePath, bool debug) : debug(debug), databasePath(databasePath), window(databasePath)
{
  //detector = new cv::SurfFeatureDetector();
  detector = new cv::FastFeatureDetector();
  //detector = new cv::SiftFeatureDetector();
  
  //extractor = new cv::SurfDescriptorExtractor();
  //extractor = new cv::SurfDescriptorExtractor(400, 4, 2, true, true);
  extractor = new cv::SiftDescriptorExtractor();
  
  //matcher = new cv::BFMatcher(cv::NORM_L2);
  matcher = new cv::FlannBasedMatcher;

  _loadDatabase();

  if (debug)
  {
    cv::namedWindow(WINDOW);
    cv::startWindowThread();
  }
};

Surf::~Surf()
{
  if (debug)
  {
    cv::destroyWindow(WINDOW);
  }
}

void Surf::_loadDatabase()
{
  unsigned char isFile = 0x8;

  struct dirent *entry, *entry_l;
  createDirectory(databasePath);
  DIR* dp = opendir(databasePath.c_str());
  int labelCnt = 0;
  int imgCnt = 0;

  std::string label = "";
  while((entry = readdir(dp)))
  {
    if (entry->d_type != isFile && entry->d_name[0] != '.')
    {
      std::string label = std::string(entry->d_name);
      ROS_DEBUG("Found label %s", label.c_str());
      labelCnt++;
      DIR* dp_l = opendir((databasePath + "/" + label).c_str());
      while((entry_l = readdir(dp_l)))
      {
        std::string filename = std::string(entry_l->d_name);
        if (entry_l->d_type == isFile && filename.find(MASK_FIX) == std::string::npos)
        {
          std::string img_path = databasePath + "/" + label + "/" + filename;
          std::string mask_path = databasePath + "/" + label + "/" + MASK_FIX + filename;
          ROS_DEBUG("  Img: %s", img_path.c_str());
          imgCnt++;
          cv::Mat img = cv::imread(img_path);
          cv::Mat mask = cv::imread(mask_path, 0);
          _loadImage(img, label, mask);
        }
      }
      closedir(dp_l);
    }
  }
  closedir(dp);
  ROS_INFO("  Total labels loaded: %d", labelCnt);
  ROS_INFO("  Total image loaded: %d", imgCnt);
  if (!matcher->empty())
    matcher->train();
}

void Surf::_loadImage(cv::Mat &img, std::string label, cv::Mat &mask)
{
  dbImages.push_back(img);
  
  int labelIdx = findIdx(dbLabels, label);

  if (labelIdx == -1)
  {
    // Label does not yet exists
    dbLabels.push_back(label);
    labelIdx = dbLabels.size() - 1;
  }

  dbImgLabel.push_back(labelIdx);

  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

  if (!mask.empty())
  {
    detector->detect(img, keypoints, mask);
  }
  else
  {
    detector->detect(img, keypoints);
  }
  dbKeypoints.push_back(keypoints);

  extractor->compute(img, keypoints, descriptors);
  std::vector<cv::Mat> desc;
  desc.push_back(descriptors);
  matcher->add(desc);
}

void Surf::match(cv::Mat &descriptors, std::vector<cv::DMatch> &goodMatches)
{
  std::vector<std::vector<cv::DMatch> > matches;
  std::vector<cv::DMatch> goodMatches1;
  matcher->knnMatch(descriptors, matches, 2);
  //matcher->match(descriptors, goodMatches);

  secondNNTest(matches, goodMatches);
  //matchesTest(goodMatches1, goodMatches);
}


std::vector<Match> Surf::_getMostMatches(std::vector<cv::DMatch> &matches)
{
  // Create memory for counting.
  std::vector<int> labelCnt;
  std::vector<int> imgCnt;
  for (std::vector<std::string>::size_type i = 0; i < dbLabels.size(); ++i)
  {
    labelCnt.push_back(0);
  }
  for (std::vector<cv::Mat>::size_type i = 0; i < dbImages.size(); ++i)
  {
    imgCnt.push_back(0);
  }
  
  // Count the labels and images
  for (std::vector<cv::DMatch>::iterator it = matches.begin(); it != matches.end(); ++it)
  {
    labelCnt[dbImgLabel[it->imgIdx]]++;
    imgCnt[it->imgIdx]++;
  }

  Match m1, m2;
  m1.matches = 0;
  m2.matches = 0;
  for (std::vector<int>::size_type i = 0; i < labelCnt.size(); ++i)
  {
    if (m1.matches < labelCnt[i])
    {
      m2.matches = m1.matches;
      m1.matches = labelCnt[i];
      m2.labelIdx = m1.labelIdx;
      m1.labelIdx = i;
    }
    else if (m2.matches < labelCnt[i])
    {
      m2.matches = labelCnt[i];
      m2.labelIdx = i;
    }
  }

  m1.imgCnt = 0;
  m2.imgCnt = 0;
  for (std::vector<int>::size_type i = 0; i < imgCnt.size(); i++)
  {
    if (dbImgLabel[i] == m1.labelIdx && m1.imgCnt < imgCnt[i])
    {
      m1.imgCnt = imgCnt[i];
      m1.imgIdx = i;
    }
    else if (dbImgLabel[i] == m2.labelIdx && m2.imgCnt < imgCnt[i])
    {
      m2.imgCnt = imgCnt[i];
      m2.imgIdx = i;
    }
  }
  std::vector<Match> m;
  m.push_back(m1);
  m.push_back(m2);
  return m;
}

std::string Surf::recognize(cv::Mat &img, cv::Mat &mask)
{
  if (!matcher->empty())
  {
    // Extract features from given image
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    if (!mask.empty())
    {
      detector->detect(img, keypoints, mask);
    }
    else
    {
      detector->detect(img, keypoints);
    }

    extractor->compute(img, keypoints, descriptors);

    // Find matches in database
    std::vector<cv::DMatch> matches;
    match(descriptors, matches);

    std::vector<Match> bestMatches = _getMostMatches(matches);

    /*std::vector<cv::DMatch> goodMatches2;
    ransacTest(
        matches, 
        bestMatches[1].imgIdx, 
        keypoints,
        dbKeypoints[bestMatches[1].imgIdx],
        goodMatches2
      );*/

    ROS_INFO("- - - - - - - - - - - - -");
    ROS_INFO("Best: %d", bestMatches[0].matches);
    ROS_INFO("Next: %d", bestMatches[1].matches);
    //ROS_INFO("Ransac-best: %d/%d", goodMatches.size(), bestMatches[0].imgCnt);
    //ROS_INFO("Ransac-next: %d/%d", goodMatches2.size(), bestMatches[1].imgCnt);
    //showMatches(img, keypoints, bestMatches[0].imgIdx, matches);
    //showMatches(img, keypoints, bestMatches[0].imgIdx, goodMatches);

    // TODO: Remove when not testing.
    std::vector<cv::DMatch> goodMatches;
    ransacTest(
      matches, 
      bestMatches[0].imgIdx, 
      keypoints,
      dbKeypoints[bestMatches[0].imgIdx],
      goodMatches
    );

    this->testNumber = goodMatches.size();

    size_t threshold = 10;

    if (bestMatches[0].matches > threshold)
    {
      if (bestMatches[0].matches > 2*std::max(15, bestMatches[1].matches))
      {
        showMatches(img, keypoints, bestMatches[0].imgIdx, matches);
        ROS_INFO("Recognized: %s", dbLabels[bestMatches[0].labelIdx].c_str());
        return dbLabels[bestMatches[0].labelIdx];
      }
      else
      {
        std::vector<cv::DMatch> goodMatches;
        ransacTest(
            matches, 
            bestMatches[0].imgIdx, 
            keypoints,
            dbKeypoints[bestMatches[0].imgIdx],
            goodMatches
          );
        showMatches(img, keypoints, bestMatches[0].imgIdx, goodMatches);
        ROS_INFO("Ransac matches: %d", goodMatches.size());
        if (goodMatches.size() > threshold)
        {
          ROS_INFO("Recognized: %s", dbLabels[bestMatches[0].labelIdx].c_str());
          return dbLabels[bestMatches[0].labelIdx];
        }

        if (bestMatches[1].matches > 10 && bestMatches[0].matches - 10 < bestMatches[1].matches)
        {
          ROS_WARN("The second best had similar result.");
        }
      }
    }
    if (bestMatches[0].matches > 0)
      showMatches(img, keypoints, bestMatches[0].imgIdx, matches);

    ROS_WARN("NOT recognized!");
  }
  else
  {
    ROS_ERROR("Matcher is empty!");
  }
  return "";
}


void Surf::insertIntoDb(cv::Mat &img, std::string label, cv::Mat &mask)
{
  std::string newPath = databasePath + "/" + label;
  createDirectory(newPath);

  std::string filename = random_filename();
  std::string file_path = newPath + "/" + filename + ".jpg";
  std::string mask_path = newPath + "/" + MASK_FIX + filename + ".jpg";
  cv::imwrite(file_path, img);
  if (!mask.empty())
    cv::imwrite(mask_path, mask);
  _loadImage(img, label, mask);
  matcher->train();
}


void Surf::showKeypoints(cv::Mat &img, std::vector<cv::KeyPoint> &keypoints)
{
  if (debug)
  {
    cv::Mat draw;
    cv::drawKeypoints(img, keypoints, draw);
    cv::imshow(WINDOW, draw);
  }
}


void Surf::showMatches(
  cv::Mat &img1, 
  std::vector<cv::KeyPoint> &keypoints1, 
  int imgIdx,
  //cv::Mat &img2, 
  //std::vector<cv::KeyPoint> &keypoints2, 
  std::vector<cv::DMatch> &matches
)
{
  if (debug)
  {
    cv::Mat draw;
    cv::drawMatches(img1, keypoints1, dbImages[imgIdx], dbKeypoints[imgIdx], matches, draw);
    cv::imshow(WINDOW, draw);
  }
}


void Surf::showMatches(
  cv::Mat &img1, 
  std::vector<cv::KeyPoint> &keypoints1, 
  cv::Mat &img2, 
  std::vector<cv::KeyPoint> &keypoints2, 
  std::vector<std::vector<cv::DMatch> > &matches
)
{
  if (debug)
  {
    cv::Mat draw;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, draw);
    cv::imshow(WINDOW, draw);
  }
}
