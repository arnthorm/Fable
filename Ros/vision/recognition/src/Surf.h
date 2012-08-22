
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

struct Match
{
  int matches;
  int labelIdx;
  int imgIdx;
  int imgCnt;
};

class Surf
{
  bool debug;
  std::string databasePath;
  const std::string window;

  cv::FeatureDetector* detector;
  cv::DescriptorExtractor* extractor;
  cv::DescriptorMatcher* matcher;

  std::vector<cv::Mat> dbImages; // [imgIdx] = image
  std::vector<std::string> dbLabels; // [labelIdx] = label
  std::vector<int> dbImgLabel; // [imgIdx] = labelIdx
  std::vector<std::vector<cv::KeyPoint> > dbKeypoints; // [imgIdx] = keypoints
  std::vector<cv::Mat> dbDescriptors; // [imgIdx] = descriptors

  public:
    int testNumber;
    Surf(std::string databasePath, bool debug=false);
    ~Surf();
    std::string recognize(cv::Mat &img, cv::Mat &mask);
    void match(cv::Mat &descriptors, std::vector<cv::DMatch> &matches);
    void insertIntoDb(cv::Mat &, std::string, cv::Mat &);
    void showKeypoints(cv::Mat &, std::vector<cv::KeyPoint> &);
    void showMatches(cv::Mat &, std::vector<cv::KeyPoint> &, int, std::vector<cv::DMatch> &);
    void showMatches(cv::Mat &, std::vector<cv::KeyPoint> &, cv::Mat &, std::vector<cv::KeyPoint> &, std::vector<std::vector<cv::DMatch> > &);

  private:
    void _loadDatabase();
    void _loadImage(cv::Mat&, std::string, cv::Mat &mask);
    int _getImageWithMostMatches(std::vector<cv::DMatch> &matches);
    //std::string _getLabelWithMostMatches(std::vector<cv::DMatch> &matches, int&);
    std::vector<Match> _getMostMatches(std::vector<cv::DMatch> &matches);
    //std::string _getLabelFromIdx(int);
};
