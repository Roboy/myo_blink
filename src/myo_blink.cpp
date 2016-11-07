#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "ros/ros.h"

/*
* message definitions from the std_msgs package. This package defines the
* fundamental message types.
*/
#include "myo_blink/moveMotor.h"
#include "myo_blink/setupMotor.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <sstream>

class MyoMotor {

public:
  /*
  * Setup motor in position, velocity or force control mode.
  * position control mode: 0
  * velocity control mode: 1
  * effort / force control mode: 2
  */
  bool setupMotor(myo_blink::setupMotor::Request &req,
                  myo_blink::setupMotor::Response &res) {
    enum controlMode { position, velocity, effort = 2, force = 2 };
    switch (req.controlMode) {
    case position:
      flexray.initPositionControl((uint)0, (uint)0);
      ROS_INFO(
          "myo_blink: Set up motor 0 on ganglion 0 in position control mode.");
      break;
    case velocity:
      flexray.initVelocityControl((uint)0, (uint)0);
      ROS_INFO(
          "myo_blink: Set up motor 0 on ganglion 0 in velocity control mode.");
      break;
    case effort:
      flexray.initForceControl((uint)0, (uint)0);
      ROS_INFO(
          "myo_blink: Set up motor 0 on ganglion 0 in force control mode.");
      break;
    default:
      ROS_ERROR("myo_blink: Received an unknown control mode. Check the enum: "
                "position control mode: 0, velocity control mode: 1, effort / "
                "force control mode: 2 ");
      res.is_success = false;
      return true;
    }
    res.is_success = true;
    return true;
  }

  /*
  * Implements the service to move the motors.
  */
  bool moveMotor(myo_blink::moveMotor::Request &req,
                 myo_blink::moveMotor::Response &res) {
    if (flexray.commandframe0[0].sp[0] = req.setpoint) {
      res.is_success = true;
    } else {
      res.is_success = false;
    }
    return true;
  }

  FlexRayHardwareInterface flexray;
};
/**
 * This tutorial demonstrates simple sending of messages over the ROS
 * system.
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command
   * line.
   * For programmatic remappings you can use a different version of init() which
   * takes
   * remappings directly, but for most command-line programs, passing argc and
   * argv is
   * the easiest way to do it.  The third argument to init() is the name of the
   * node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /*
  * Instantiate the MyoMotor class. The class only really exists so that we do
  * not have to have the FlexRayHardwareInterface instance a global - to use
  * them in the functions outside of main.
  */
  MyoMotor myo_control;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher numOGang_pubber = n.advertise<std_msgs::String>(
      "/myo_blink/numberOfGanglionsConnected", 1000);
  ros::Publisher displacement_pubber = n.advertise<std_msgs::Float32>(
      "/myo_blink/muscles/0/sensors/displacement", 1000);

  /*
  * This advertises the services with the roscore, making them available to call
  * from other ROS nodes or via the command line. For this demo it is likely
  * convenient to call them from the command line:
  * rosservice call /myo_blink/setup /myo_blink/*tab - will autocomplete*
  */
  ros::ServiceServer moveMotor_service =
      n.advertiseService("/myo_blink/move", &MyoMotor::moveMotor, &myo_control);
  ros::ServiceServer setupMotor_service = n.advertiseService(
      "/myo_blink/setup", &MyoMotor::setupMotor, &myo_control);
  /*
  * The actual flexrayusbinterface. It resides in the ROS package
  * flexrayusbinterface. If you look into both the CMakeLists.txt and
  * package.xml you will find references to it. If catkin cannot find it at
  * buildtime it will fail the checks before actually starting the compilation.
  */
  double pos = 0;
  double vel = 0;
  double eff = 0;

  /**
  * Defines the update rate of this ROS node
  * It will be used by loop_rate.sleep();
  */
  ros::Rate loop_rate(1);

  /**
     * This is a message object. You stuff it with data, and then publish it.
     */
  std_msgs::String msg;

  /**
  * Read data from the flexray bus
  */
  myo_control.flexray.exchangeData();

  /**
  * We are 'stuffing' the message with data
  */
  std::stringstream ss;
  ss << "We currently have "
     << myo_control.flexray.checkNumberOfConnectedGanglions()
     << "ganglia connected.";
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());

  /**
   * The publish() function is how you send messages. The parameter
   * is the message object. The type of this object must agree with the type
   * given as a template parameter to the advertise<>() call, as was done
   * in the constructor above.
   */
  numOGang_pubber.publish(msg);

  /*
  * This while loop uses 'ros::ok()' to check if ROS/roscore is still running
  * and ctrl-c hasn't been pressed on this node.
  */
  while (ros::ok()) {
    /**
    * Read data from the flexray bus
    */
    myo_control.flexray.exchangeData();

    /*
    * Access the motor connected on SPI 0 at ganglion 0
    */
    pos = myo_control.flexray.GanglionData[0].muscleState[0].actuatorPos *
          myo_control.flexray.controlparams.radPerEncoderCount;
    vel = myo_control.flexray.GanglionData[0].muscleState[0].actuatorVel *
          myo_control.flexray.controlparams.radPerEncoderCount;

    float polyPar[4];
    polyPar[0] = 0;
    polyPar[1] = 0.237536;
    polyPar[2] = -0.000032;
    polyPar[3] = 0;
    float tendonDisplacement =
        myo_control.flexray.GanglionData[0].muscleState[0].tendonDisplacement;
    eff = polyPar[0] + polyPar[1] * tendonDisplacement +
          polyPar[2] * powf(tendonDisplacement, 2.0f) +
          polyPar[3] * powf(tendonDisplacement, 3.0f);

    /*
    * Publish the spring displacement.
    */
    std_msgs::Float32 msg_displacement;
    msg_displacement.data =
        myo_control.flexray.GanglionData[0].muscleState[0].tendonDisplacement /
        32768.0f;
    displacement_pubber.publish(msg_displacement);
    /**
    * This lets ROS read all messages, etc. There are a number of these
    * 'spinners'
    */
    ros::spinOnce();

    loop_rate.sleep();
  }

  return false;
}
