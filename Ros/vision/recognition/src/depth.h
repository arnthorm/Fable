

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


cv::Mat drawHistogram(cv::MatND &hist);
cv::MatND getHistogram(const cv::Mat &depth);

void getSegmentThresholds(cv::MatND hist, float value, float &upper, float &lower);
void threshold(cv::Mat &src, cv::Mat &dest, float upper, float lower);
void getMaskFromDepth(cv::Mat &src, cv::Mat &dest, float depthValue);
