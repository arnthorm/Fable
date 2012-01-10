#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <curl/curl.h>
#include <cv_bridge/CvBridge.h>
#include <opencv/cvwimage.h>
//#include <curl/types.h>
#include <curl/easy.h>
#include <boost/program_options/option.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/program_options/parsers.hpp>
#include <string>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <exception>
#include <vector>
using namespace std;
namespace po = boost::program_options;

static const char WINDOW[] = "Main Window";


class MemoryBuffer {
        public:
                char *memory;
                size_t size;
                MemoryBuffer() : memory(NULL), size(0) {}
                ~MemoryBuffer() {}
                void clear() {
                        free(memory);
                        memory = NULL;
                        size = 0;
                }
};

void *curlRealloc(void *ptr, size_t size) {
        if (size <= 0) {
                throw string("Realloc size is less than or equal to 0");
        }
        return realloc(ptr, size);
}

size_t writeMemoryCallback(void *ptr, size_t size, size_t nmemb, void *data){
        size_t realsize = size * nmemb;
        struct MemoryBuffer *mem = (struct MemoryBuffer*)data;

        mem->memory = (char*)curlRealloc(mem->memory, mem->size + realsize + 1);
        if (mem->memory) {
                memcpy(&(mem->memory[mem->size]), ptr, realsize + 1);
                mem->size += realsize;
                mem->memory[mem->size] = 0;
        } else {
                throw string("mem->memory is NULL");
        }
        return realsize;
}

bool placeIpImageInBuffer(CURL *curl, MemoryBuffer *buf, string url) {
        buf->clear();
        CURLcode res; 
        if (curl) {
                curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void*)buf);
                res = curl_easy_perform(curl);
                if (res == CURLE_OK) {
                        return true;
                } else {
                        cerr << "Could not fetch the image from " << url << endl;
                }
                buf->clear();
        }
        return false;
}

void convertBufferDataToIplImage(MemoryBuffer *buf, image_transport::Publisher *pub) {
        if (buf->size > 0) {
                cv::Mat mt = cv::imdecode(cv::Mat(1, buf->size, CV_8UC3, buf->memory), CV_LOAD_IMAGE_UNCHANGED);
                IplImage imgDl = mt;
                //cv::WImageBuffer3_b image(&imgDl);
                sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(&imgDl, "bgr8");
                pub->publish(msg);
                cvShowImage(WINDOW, &imgDl);
        }
}

void curlInit(CURL *curl, string url, void *buf) {
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeMemoryCallback);
        curl_easy_setopt(curl, CURLOPT_BUFFERSIZE, 100000);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 2);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 2000);
}

void curlDeInit(CURL *curl) {
        curl_easy_cleanup(curl);
}

int main(int argc, char **argv) {
        po::options_description desc("Allowed options");
        desc.add_options()
                ("url", po::value<string>(), "The url of the ipcam images (http://1.2.3.4:8086/shot.jpg). If url is set to \'def\' then the url will point to a still image of some faces.")
                ("help", "produce help message")
        ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);

        if (vm.count("help")) {
                cout << desc << endl;
                return 1;
        }

        //string url = "http://10.16.19.108:8086/shot.jpg";
        //string url = "http://www.dance-classes.ca/photos/faces/faces_2004W_comp1.jpg";
        string url = "";
        if (vm.count("url")) {
                url = vm["url"].as< string >();
                if (url == "def") {
                        url = "http://www.dance-classes.ca/photos/faces/faces_2004W_comp1.jpg";
                        cout << "url was set to " << url << endl;
                }
        } else {
                cout << "url address was not set" << endl;
                return -1;
        }
        
        ros::init(argc, argv, "ipcamstream");

        ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);
        image_transport::Publisher pub = it.advertise("ipcam_image_topic", 1);

        ros::Rate loop_rate(20);

        cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
        CURL *curl = curl_easy_init();


        MemoryBuffer buf;
        curlInit(curl, url, &buf);
        
        try {
                while(nh.ok()) {
                        try {
                                if (placeIpImageInBuffer(curl, &buf, url) == true) {
                                        convertBufferDataToIplImage(&buf, &pub);
                                        cvWaitKey(3);
                                }
                        } catch (cv::Exception &exp) {
                                cout << "CV::EXCEPTION: " << exp.what() << endl;
                                cout << "BUFFER SIZE: " << buf.size << endl;
                        }
                        ros::spinOnce();
                        loop_rate.sleep();
                }
        } catch(string &caught) {
                cout << "EXCEPTION: " << caught << endl;
        }

        curlDeInit(curl);
        cv::destroyWindow(WINDOW);

        return 0;
}
