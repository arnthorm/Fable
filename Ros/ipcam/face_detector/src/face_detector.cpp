#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>
#include <iostream>
using namespace cv;
using namespace std;

/* Create memory for calculations */
static CvMemStorage *storage = 0;

/* Create a new Haar classifier */
static CvHaarClassifierCascade *cascade = 0;

/* Function prototype for detecting and drawing an object from an image */
void DetectAndDraw(IplImage *image);

/* Create a string that contains the cascade name "haarcascade_profileface.xml" */
const char *cascade_name = "/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_frontalface_alt.xml";

/* Store all constants for image encodings in the enc namespace to be used later. */
namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image Processed";

/* Use method of ImageTransport to create image publisher */
image_transport::Publisher pub;

/* This function is called everytime a new image is published */
void ImageCallback(const sensor_msgs::ImageConstPtr &original_image) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
                cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
                IplImage image(cv_ptr->image);

                //cvShowImage(WINDOW, &image);
                //cvWaitKey(0);

                if (!(cascade = (CvHaarClassifierCascade *) cvLoad(cascade_name, 0, 0, 0))) {
                        cerr << "ERROR: COULD NOT LOAD CLASSIFIER CASCADE\n" << endl;
                }
                DetectAndDraw(&image);
                cv::waitKey(3);
        } catch (cv_bridge::Exception &e) {
                /* if there is an error during conversion, display it */
                ROS_ERROR( "image_filter::main.cpp::cv_bridge exception: %s", e.what() );
                return;
        }

}         /* ImageCallback */
/* OpenCV Sample Application: facedetect.c */

/* Function to detect and draw any faces that is present in an image */
void DetectAndDraw(IplImage *img) {
        int scale = 1;

        /* Create a new image based on the input image */
        IplImage *temp = cvCreateImage(cvSize(img->width / scale, img->height / scale), img->depth, img->nChannels);

        /* Create two points to represent the face locations */
        CvPoint pt1, pt2;

        /* Clear the memory storage which was used before */
        storage = cvCreateMemStorage(0);
        cvClearMemStorage(storage);

        /* Find whether the cascade is loaded, to find the faces. If yes, then: */
        if (cascade) {
                CvSeq *faces = cvHaarDetectObjects( img, cascade, storage, 1.1, 2, CV_HAAR_DO_CANNY_PRUNING, cvSize(40, 40) );

                /* Loop the number of faces found. */
                for (int i = 0; i < (faces ? faces->total : 0); i++) {
                        /* Create a new rectangle for drawing the face */
                        CvRect *r = (CvRect *) cvGetSeqElem(faces, i);
                        
                        pt1.x = r->x * scale;
                        pt2.x = (r->x + r->width) * scale;
                        pt1.y = r->y * scale;
                        pt2.y = (r->y + r->height) * scale;

                        /* Draw the rectangle in the input image */
                        cvRectangle(img, pt1, pt2, CV_RGB(255, 0, 0), 3, 8, 0);
                }
        }

        /* Show the image in the window named "result" */
        cvShowImage(WINDOW, img);

        /* Release the temp image created. */
        cvReleaseImage(&temp);
}         /* DetectAndDraw */

int main(int argc, char **argv) {
        ros::init(argc, argv, "face_detector");
        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        
        cvNamedWindow(WINDOW);

        image_transport::Subscriber sub = it.subscribe("/ipcam_image_topic", 1, ImageCallback);
        
        ros::spin();
        
        ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
        cvDestroyWindow(WINDOW);
}         /* main */
