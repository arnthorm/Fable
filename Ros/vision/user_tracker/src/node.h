
#include <ros/ros.h>
#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnCppWrapper.h>

void setup(int argc, char **argv, xn::UserGenerator *userGenerator);
void publishUserDetected(XnUserID user);
void publishUserLost(XnUserID user);
void processAndPublish(std::vector<XnUserID> users, uint32_t frameId);
std::string getUserNameFromId(int userId);
