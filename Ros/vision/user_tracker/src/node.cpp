#include <ros/ros.h>
#include <ros/package.h>
#include "Eigen/Geometry"

#include "node.h"

#include <user_tracker/UserPos.h>
#include <user_tracker/UserAng.h>
#include <user_tracker/UserRecognized.h>
#include <std_msgs/Int32.h>

#include <map>

ros::Publisher userPosPub, userAngPub, userDetectedPub, userLostPub;
ros::Subscriber userNameSub;
xn::UserGenerator *pUserGenerator;

std::map<int, std::string> usernames;

/** Gets the username from userId, if any.
 */
std::string getUserNameFromId(int userId)
{
  if (usernames.count(userId))
    return usernames[userId];
  else
    return "";
}

/** Converts geometry point to Eigen3 vector
 */
Eigen::Vector3f pointToEigen(geometry_msgs::Point point)
{
  Eigen::Vector3f result;
  result(0) = point.x;
  result(1) = point.y;
  result(2) = point.z;
  return result;
}

/** Takes two joint positions, subtracts them and returns 
 * them as a Eigen3 vector for convenience.
 */
Eigen::Vector3f subtractPos(user_tracker::JointPos pos1, user_tracker::JointPos pos2)
{
  return pointToEigen(pos1.position) - pointToEigen(pos2.position);
}

/** Calculates the shoulder and elbow angles and returns a ArmAng message.
 */
user_tracker::ArmAng calculateArmAngles(
  user_tracker::ArmPos &arm, 
  user_tracker::ArmPos &secondArm, 
  user_tracker::UserPos &body
)
{
  Eigen::Vector3f bodyVertical, bodyHorizontal;
  Eigen::Vector3f bodyDiagonal, bodyPerpendicular;
  Eigen::Vector3f vElbShoulder, vHandElb;
  Eigen::Vector3f vShoulderElb;

  bodyHorizontal = subtractPos(secondArm.shoulder, arm.shoulder);
  bodyDiagonal = subtractPos(arm.shoulder, body.torso);
  bodyPerpendicular = bodyHorizontal.cross(bodyDiagonal);
  bodyVertical = bodyHorizontal.cross(bodyPerpendicular);

  vShoulderElb = subtractPos(arm.shoulder, arm.elbow);
  vHandElb = subtractPos(arm.hand, arm.elbow);

  bodyVertical.normalize();
  bodyHorizontal.normalize();
  bodyPerpendicular.normalize();

  vShoulderElb.normalize();
  vHandElb.normalize();

  user_tracker::ArmAng armAng;
  armAng.shoulder_yaw.angle = acos(bodyVertical.dot(-vShoulderElb));
  armAng.shoulder_yaw.confidence = std::min(
    secondArm.shoulder.confidence,
    std::min(
      arm.shoulder.confidence,
      std::min(
        arm.elbow.confidence,
        body.torso.confidence
      )
    )
  );
  armAng.shoulder_pitch.angle = acos(bodyHorizontal.dot(-vShoulderElb));
  armAng.shoulder_pitch.confidence = armAng.shoulder_yaw.confidence;
  //armAng.shoulder_pitch = acos((-bodyPerpendicular).dot(-vShoulderElb));
  armAng.elbow_pitch.angle = acos(vShoulderElb.dot(vHandElb));
  armAng.elbow_pitch.confidence = std::min(
    arm.shoulder.confidence,
    std::min(
      arm.elbow.confidence,
      arm.hand.confidence 
    )
  );
  armAng.shoulder_roll.angle = acos(bodyVertical.dot(vHandElb));
  return armAng;
}

/** Calculates the hip and knee angles and returns a LegAng message.
 */
user_tracker::LegAng calculateLegAngles(
  user_tracker::LegPos &leg, 
  user_tracker::LegPos &secondLeg,
  user_tracker::UserPos &body
)
{
  Eigen::Vector3f hipsHorizontal, hipsVertical;
  Eigen::Vector3f bodyDiagonal, hipsPerpendicular;
  Eigen::Vector3f vHipKnee, vFootKnee;

  user_tracker::LegAng legAng;

  hipsHorizontal = subtractPos(secondLeg.hip, leg.hip);
  bodyDiagonal = subtractPos(leg.hip, body.torso);
  hipsPerpendicular = hipsHorizontal.cross(bodyDiagonal);
  hipsVertical = hipsHorizontal.cross(hipsPerpendicular);
  vHipKnee = subtractPos(leg.hip, leg.knee);
  vFootKnee = subtractPos(leg.foot, leg.knee);

  hipsHorizontal.normalize();
  bodyDiagonal.normalize();
  hipsPerpendicular.normalize();
  hipsVertical.normalize();
  vHipKnee.normalize();
  vFootKnee.normalize();
  
  legAng.knee_pitch.angle = acos(vHipKnee.dot(vFootKnee));
  legAng.knee_pitch.confidence = std::min(
    leg.hip.confidence,
    std::min(
      leg.knee.confidence,
      leg.foot.confidence 
    )
  );
  legAng.hip_pitch.angle = acos(hipsVertical.dot(-vHipKnee));
  legAng.hip_pitch.confidence = std::min(
    secondLeg.hip.confidence,
    std::min(
      leg.hip.confidence,
      std::min(
        leg.knee.confidence,
        body.torso.confidence
      )
    )
  );
  legAng.hip_yaw.angle = acos(hipsHorizontal.dot(-vHipKnee));
  legAng.hip_yaw.confidence = legAng.hip_pitch.confidence;
  return legAng;
}

/** Converts a XnVector to a geometry point.
 */
geometry_msgs::Point xnVectorToPoint(XnVector3D value){
  geometry_msgs::Point point;
  point.x = value.X/1000.0;
  // TODO: Why is there a minus here ???
  point.y = -value.Y/1000.0;
  point.z = value.Z/1000.0;
  return point;
}

/** Gets and returns the position of a joint.
 */
void getJointPos(XnUserID user, user_tracker::JointPos &jointPos, XnSkeletonJoint name)
{
   XnSkeletonJointPosition joint;
   pUserGenerator->GetSkeletonCap().GetSkeletonJointPosition(user, name, joint);
   jointPos.position = xnVectorToPoint(joint.position);
   jointPos.confidence = joint.fConfidence;
}

//TODO: this is a temp variable for testing only:
float bodyRotation = 0;

/** Gets and returns all the joint positions as a UserPos message.
 */
void getUserPos(XnUserID user, user_tracker::UserPos &userPos)
{
  userPos.user_id = user;
  userPos.username = getUserNameFromId(user);
  getJointPos(user, userPos.head, XN_SKEL_HEAD);
  getJointPos(user, userPos.neck, XN_SKEL_NECK);

  // BUG: For some reason the left and right hand have been switched
  getJointPos(user, userPos.right_arm.shoulder, XN_SKEL_LEFT_SHOULDER);
  getJointPos(user, userPos.right_arm.elbow, XN_SKEL_LEFT_ELBOW);
  getJointPos(user, userPos.right_arm.hand, XN_SKEL_LEFT_HAND);
  getJointPos(user, userPos.left_arm.shoulder, XN_SKEL_RIGHT_SHOULDER);
  getJointPos(user, userPos.left_arm.elbow, XN_SKEL_RIGHT_ELBOW);
  getJointPos(user, userPos.left_arm.hand, XN_SKEL_RIGHT_HAND);

  /*getJointPos(user, userPos.left_arm.shoulder, XN_SKEL_LEFT_SHOULDER);
  getJointPos(user, userPos.left_arm.elbow, XN_SKEL_LEFT_ELBOW);
  getJointPos(user, userPos.left_arm.hand, XN_SKEL_LEFT_HAND);
  getJointPos(user, userPos.right_arm.shoulder, XN_SKEL_RIGHT_SHOULDER);
  getJointPos(user, userPos.right_arm.elbow, XN_SKEL_RIGHT_ELBOW);
  getJointPos(user, userPos.right_arm.hand, XN_SKEL_RIGHT_HAND);*/

  getJointPos(user, userPos.torso, XN_SKEL_TORSO);
  getJointPos(user, userPos.left_leg.hip, XN_SKEL_LEFT_HIP);
  getJointPos(user, userPos.left_leg.knee, XN_SKEL_LEFT_KNEE);
  getJointPos(user, userPos.left_leg.foot, XN_SKEL_LEFT_FOOT);
  getJointPos(user, userPos.right_leg.hip, XN_SKEL_RIGHT_HIP);
  getJointPos(user, userPos.right_leg.knee, XN_SKEL_RIGHT_KNEE);
  getJointPos(user, userPos.right_leg.foot, XN_SKEL_RIGHT_FOOT);
  userPos.body.angle = bodyRotation;
}

/** Gets and returns all the joint angles as a UserAng message.
 */
void getUserAng(user_tracker::UserPos &userPos, user_tracker::UserAng &userAng)
{
  userAng.left_arm = calculateArmAngles(userPos.left_arm , userPos.right_arm, userPos);
  userAng.right_arm = calculateArmAngles(userPos.right_arm , userPos.left_arm, userPos);
  userAng.left_leg = calculateLegAngles(userPos.left_leg, userPos.right_leg, userPos);
  userAng.right_leg = calculateLegAngles(userPos.right_leg, userPos.left_leg, userPos);

  Eigen::Vector3f bodyVertical, bodyHorizontal;
  Eigen::Vector3f bodyDiagonal, bodyPerpendicular;

  bodyHorizontal = subtractPos(userPos.left_arm.shoulder, userPos.right_arm.shoulder);
  bodyDiagonal = subtractPos(userPos.left_arm.shoulder, userPos.torso);
  bodyPerpendicular = bodyHorizontal.cross(bodyDiagonal);

  bodyPerpendicular.normalize();

  Eigen::Vector3f zVec, xVec, plane, projected;
  zVec(0) = 0;
  zVec(1) = 0;
  zVec(2) = 1;
  xVec(0) = 1;
  xVec(1) = 0;
  xVec(2) = 0;

  plane = zVec.cross(xVec);
  plane.normalize();

  //projected = bodyPerpendicular - bodyPerpendicular.dot(plane)*plane;
  projected = bodyHorizontal - bodyHorizontal.dot(plane)*plane;

  userAng.body.angle = acos(projected.dot(zVec))*180/3.14159;

  // TODO: Remove: this is for testing only:
  bodyRotation = userAng.body.angle;
  userAng.body.confidence = 1;

/* Not using this one, will only get 1 or 2 degrees in each direction when the head is 
 * fully tilted or paned.
  Eigen3::Vector3f bodyVertical, bodyHorizontal;
  Eigen3::Vector3f bodyDiagonal, bodyPerpendicular;
  Eigen3::Vector3f vHeadNeck;

  bodyHorizontal = subtractPos(userPos.left_arm.shoulder, userPos.right_arm.shoulder);
  bodyDiagonal = subtractPos(userPos.left_arm.shoulder, userPos.torso);
  bodyPerpendicular = bodyHorizontal.cross(bodyDiagonal);
  bodyVertical = bodyHorizontal.cross(bodyPerpendicular);

  bodyVertical.normalize();
  bodyHorizontal.normalize();
  bodyPerpendicular.normalize();

  vHeadNeck = subtractPos(userPos.head, userPos.neck);

  vHeadNeck.normalize();

  userAng.neck_pan.angle = acos(bodyPerpendicular.dot(vHeadNeck));
  userAng.neck_tilt.angle = acos(bodyHorizontal.dot(vHeadNeck));
  
  ROS_INFO("Pan %f", userAng.neck_pan.angle*180/M_PI);
  ROS_INFO("Tilt: %f", userAng.neck_tilt.angle*180/M_PI);*/
}

/** Returns the magnitude of geometry point
 */
double magnitude(geometry_msgs::Point point)
{
  return sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
}

/** Prints the joint position.
 *
 * This function is for convenience for developing.
 */
void printPos(user_tracker::JointPos pos)
{
  geometry_msgs::Point pt = pos.position;
  ROS_INFO("x: %f, y: %f, z: %f", pt.x, pt.y, pt.z);
}

/** Creates and returns a JointPos message for given coordinates.
 *
 * This function is for convenience for developing.
 */
user_tracker::JointPos createPoint(double x, double y, double z)
{
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  user_tracker::JointPos pos;
  pos.position = p;
  pos.confidence = 1;
  return pos;
}

/** Prints the x, y and z value of a Eigen3 vector. 
 *
 * This is a convenience function intended for debugging
 * or development.
 */
void printVector(Eigen::Vector3f vec)
{
  ROS_INFO("[%f, %f, %f]", vec.x(), vec.y(), vec.z());
}

/** Publishes a ROS message when a new user is detected.
 */
void publishUserDetected(XnUserID user)
{
  std_msgs::Int32 userDetected;
  userDetected.data = user;
  userDetectedPub.publish(userDetected);
}

/** Publishes a ROS message, UserLost for given user.
 */
void publishUserLost(XnUserID user)
{
  std_msgs::Int32 userLost;
  userLost.data = user;
  userLostPub.publish(userLost);
  usernames[user] = "";
}

/** Process all detected users and publish their joint positions
 * and angle positions.
 */
void processAndPublish(std::vector<XnUserID> users, uint32_t frameId)
{
  if (userPosPub.getNumSubscribers() > 0 || userAngPub.getNumSubscribers() > 0)
  {
    ros::Time timeStamp = ros::Time::now();
    std::vector<user_tracker::UserPos> usersPos;
    std::vector<user_tracker::UserAng> usersAng;

    for (unsigned int i = 0; i < users.size(); ++i)
    {
      user_tracker::UserPos userPos;
      getUserPos(users[i], userPos);
      userPos.header.stamp = timeStamp;
      userPos.header.seq = frameId;
      userPos.header.frame_id = "/camera_depth_optical_frame";

      usersPos.push_back(userPos);
      userPosPub.publish(usersPos[i]);
    }

    if (userAngPub.getNumSubscribers() > 0)
    {
      for (unsigned int i = 0; i < usersPos.size(); ++i)
      {
        user_tracker::UserPos userPos = usersPos[i];
        user_tracker::UserAng userAng;
        userAng.user_id = userPos.user_id;
        userAng.username = userPos.username;
        userAng.header = userPos.header;

        getUserAng(userPos, userAng);
        
        userAngPub.publish(userAng);
      }
    }
  }
}

/** User recognized message.
 *
 * Assigns the username recognized to the user id.
 */
void userRecognized_callback(user_tracker::UserRecognized msg)
{
  ROS_INFO("UserId %d now has username %s", msg.user_id, msg.username.c_str());
  usernames[msg.user_id] = msg.username;
}

/** Sets up ROS publishers and other initializations.
 */
void setup(int argc, char **argv, xn::UserGenerator *userGenerator)
{
  pUserGenerator = userGenerator;
  ros::init(argc, argv, "user_tracker");
  ros::NodeHandle nh_;

  userPosPub = nh_.advertise<user_tracker::UserPos> ("user_pos", 100);
  userAngPub = nh_.advertise<user_tracker::UserAng> ("user_ang", 100);
  userDetectedPub = nh_.advertise<std_msgs::Int32> ("user_detected", 100);
  userLostPub = nh_.advertise<std_msgs::Int32> ("user_lost", 100);
  userNameSub = nh_.subscribe("user_recognized", 100, userRecognized_callback);
}
