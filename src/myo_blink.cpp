#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "ros/ros.h"

/*
* message definitions from the std_msgs package. This package defines the
* fundamental message types.
*/
#include "myo_blink/moveMotor.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <boost/optional.hpp>
#include <chrono>
#include <sstream>
#include <thread>

class MyoMotor
{
public:
  /*
  * Implements the service to move the motors.
  */
  bool moveMotor(myo_blink::moveMotor::Request &req, myo_blink::moveMotor::Response &res)
  {
    if (req.action == "move to")
    {
      flexray.set(0, 0, FlexRayHardwareInterface::Controller::Position, req.setpoint);
      res.is_success = true;
    }
    else if (req.action == "move with")
    {
      flexray.set(0, 0, FlexRayHardwareInterface::Controller::Velocity, req.setpoint);
      res.is_success = true;
    }
    else if (req.action == "keep")
    {
      flexray.set(0, 0, FlexRayHardwareInterface::Controller::Force, req.setpoint);
      res.is_success = true;
    }
    else
    {
      res.is_success = false;
    }
    return true;
  }

  FlexRayHardwareInterface flexray;

  MyoMotor(UsbChannel&& usb) : flexray{std::move(usb)}
  {
    //   using namespace std::chrono_literals;
    flexray.initPositionControl((uint)0, (uint)0);
    //   std::this_thread::sleep_for(2s);
    flexray.initVelocityControl((uint)0, (uint)0);
    //   std::this_thread::sleep_for(2s);
    flexray.initForceControl((uint)0, (uint)0);
    //  std::this_thread::sleep_for(2s);
  }
};

void blink(MyoMotor &myo_control)
{
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the
   * last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

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
  auto numOGang_pubber = n.advertise<std_msgs::String>("/myo_blink/numberOfGanglionsConnected", 1000, true);
  auto displacement_pubber = n.advertise<std_msgs::Float32>("/myo_blink/muscles/0/sensors/displacement", 1000);
  auto j0_pubber = n.advertise<std_msgs::Float32>("/myo_blink/muscles/0/sensors/joint_angle", 1000, true);
  auto j1_pubber = n.advertise<std_msgs::Float32>("/myo_blink/muscles/1/sensors/joint_angle", 1000, true);

  /*
  * This advertises the services with the roscore, making them available to call
  * from other ROS nodes or via the command line. For this demo it is likely
  * convenient to call them from the command line:
  * rosservice call /myo_blink/setup /myo_blink/<TAB>- will autocomplete*
  */
  auto moveMotor_service = n.advertiseService("/myo_blink/move", &MyoMotor::moveMotor, &myo_control);
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
  ros::Rate loop_rate(100);

  /**
     * This is a message object. You stuff it with data, and then publish it.
     */
  std_msgs::String msg;

  /**
  * We are 'stuffing' the message with data
  */
  std::stringstream ss;
  ss << "We currently have " << myo_control.flexray.connected_ganglions().count() << " ganglia connected.";
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
  while (ros::ok())
  {
    /*
    * Access the motor connected on SPI 0 at ganglion 0
    */
      auto state = myo_control.flexray.read_muscle(0, 0);
    pos = state.actuatorPos * myo_control.flexray.radPerEncoderCount;
    vel = state.actuatorVel * myo_control.flexray.radPerEncoderCount;

    float polyPar[4];
    polyPar[0] = 0;
    polyPar[1] = 0.237536;
    polyPar[2] = -0.000032;
    polyPar[3] = 0;
    float tendonDisplacement = state.tendonDisplacement;
    eff = polyPar[0] + polyPar[1] * tendonDisplacement + polyPar[2] * powf(tendonDisplacement, 2.0f) +
          polyPar[3] * powf(tendonDisplacement, 3.0f);

    /*
    * Publish the spring displacement.
    */
    std_msgs::Float32 msg_displacement;
    msg_displacement.data = state.tendonDisplacement / powf(2, 16);
    msg_displacement.data = state.tendonDisplacement / powf(2, 16);
    displacement_pubber.publish(msg_displacement);
    std_msgs::Float32 msg_aux;
    msg_aux.data = state.jointPos;
    j0_pubber.publish(msg_aux);
    msg_aux.data = myo_control.flexray.read_muscle(0, 1).jointPos;
    j1_pubber.publish(msg_aux);
    /**
    * This lets ROS read all messages, etc. There are a number of these
    * 'spinners'
    */
    ros::spinOnce();

    loop_rate.sleep();
  }
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS
 * system.
 */
int main(int argc, char **argv)
{
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

  /*
  * Instantiate the MyoMotor class. The class only really exists so that we do
  * not have to have the FlexRayHardwareInterface instance a global - to use
  * them in the functions outside of main.
  */
  while (UsbChannel::open("FTVDIMQW").match(
      [](UsbChannel usb) {
        MyoMotor motor{std::move(usb)};
        blink(motor);
        return false;
      },
      [](FtResult result) {
        ROS_ERROR_STREAM("Could not connect to the myo motor: " << result.str());
        return true;
      }))
    ;
}
