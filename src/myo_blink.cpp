#include "flexrayusbinterface/FlexRayHardwareInterface.hpp"
#include "flexrayusbinterface/Parsers.hpp"
#include "ros/ros.h"
#include "stdlib.h"

/*
* message definitions from the std_msgs package. This package defines the
* fundamental message types.
*/
#include "myo_blink/moveMotor.h"
#include "myo_blink/muscleState.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

#include <boost/optional.hpp>
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>

class MyoMotor {
    public:
      /*
      * Implements the service to move the motors.
      */
      bool moveMotor(myo_blink::moveMotor::Request &req,
                     myo_blink::moveMotor::Response &res) {
        if (req.action == "move to") {
          flexray.set(req.muscle, ControlMode::Position, req.setpoint);
          res.is_success = true;
        } else if (req.action == "move with") {
          flexray.set(req.muscle, ControlMode::Velocity, req.setpoint);
          res.is_success = true;
        }
        else if (req.action == "keep")
        {
          if (req.setpoint < minForce)
          {
            ROS_ERROR_STREAM("Cannot keep requested force of " << req.setpoint << " N. Minimum possible force is: " << minForce << " N.");
            res.is_success=false;
          }
          else if (req.setpoint > maxForce)
          {
              ROS_ERROR_STREAM("Cannot keep requested force of " << req.setpoint << " N. Maximum possible force is: " << maxForce << " N.");
              res.is_success = false;
          }
          else
          {
            flexray.set(req.muscle, ControlMode::Force, req.setpoint);
            res.is_success = true;
          }
        }
        else
        {
          res.is_success = false;
        }
        return true;
      }




      FlexRayHardwareInterface flexray;

      double minForce;
      double maxForce;
      std::map<std::string, int> maxContractileDisplacement;
      std::map<std::string, double> offset; //offset between position 0 and actuators position when link is perpendicular to the ground
      bool initialized;

      MyoMotor(FlexRayHardwareInterface &&flexray) : flexray{ std::move(flexray) }
      {
          for (auto &name : flexray.get_muscle_names())
          {
              offset[name] = 0;

          }
          initialized = false;
          maxForce = 70.0;
          maxContractileDisplacement["biceps"] = 12;
          maxContractileDisplacement["triceps"] = 12;
          maxContractileDisplacement["hand"] = 5;
          maxContractileDisplacement["forearm"] = 5;
      }
    };


/*
* Implements the service to initialize the motors, so that the arm is perpendicular to the base
*/
void initialize(MyoMotor &myo_control)
  {

      for (auto &name : myo_control.flexray.get_muscle_names())
      {
        myo_control.flexray.set(name, ControlMode::Force, 43);
      }

      ROS_INFO_STREAM("Press ENTER to initialize the current positions as 0");
      if (std::cin.get() == '\n')
      {
          for (auto &name : myo_control.flexray.get_muscle_names()) {
            myo_control.flexray.read_muscle(name).match(
            [&](muscleState_t &state)
            {
              // keep motors in the current position
              myo_control.flexray.set(name, ControlMode::Position, (int) state.actuatorPos*0.00005788606746738269); //TODO: read this magic number from yaml file
              ROS_INFO_STREAM("Motor: " << name << " Setting offset to: " << std::to_string((int) state.actuatorPos*0.00005788606746738269));
              myo_control.offset[name] = (int) state.actuatorPos*0.00005788606746738269;
            },
            [](FlexRayHardwareInterface::ReadError) {});
          myo_control.initialized = true;
        }
      }
    // }
  }



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
  auto numOGang_pubber = n.advertise<std_msgs::String>(
      "/myo_blink/numberOfGanglionsConnected", 1000, true);

  std::map<std::string, ros::Publisher> joint_pubs;
  joint_pubs.emplace("upper", n.advertise<std_msgs::Float32>("/myo_blink/joints/upper/angle", 1000)); // upper joint
  joint_pubs.emplace("lower", n.advertise<std_msgs::Float32>("/myo_blink/joints/lower/angle", 1000)); //lower joint


  std::map<std::string, ros::Publisher> muscle_pubs;
  std::vector<std::string> motorNames = myo_control.flexray.get_muscle_names();

  for (auto &name : motorNames) {
    muscle_pubs.emplace(
        name,
        n.advertise<myo_blink::muscleState>(std::string{"/myo_blink/muscles/"} +
                                                name + "/sensors",
                                            1000, true));
  }


  /*
  * This advertises the services with the roscore, making them available to call
  * from other ROS nodes or via the command line. For this demo it is likely
  * convenient to call them from the command line:
  * rosservice call /myo_blink/setup /myo_blink/<TAB>- will autocomplete*
  */
  auto moveMotor_service =
      n.advertiseService("/myo_blink/move", &MyoMotor::moveMotor, &myo_control);
  /*
  * The actual flexrayusbinterface. It resides in the ROS package
  * flexrayusbinterface. If you look into both the CMakeLists.txt and
  * package.xml you will find references to it. If catkin cannot find it at
  * buildtime it will fail the checks before actually starting the compilation.
  */

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
  ss << "We currently have "
     << myo_control.flexray.connected_ganglions().count()
     << " ganglia connected.";
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
    /*
    * Access the motor connected on SPI 0 at ganglion 0
    */
    myo_blink::muscleState msg_state;
    std_msgs::Float32 angle;

    for (auto &name : motorNames) {
      myo_control.flexray.read_muscle(name).match(
          [&](muscleState_t &state) {
            msg_state.elasticDisplacement = state.tendonDisplacement; //encoder ticks
            msg_state.contractileDisplacement = state.actuatorPos*0.00005788606746738269 - myo_control.offset[name]; // deviation from initial upright position
            msg_state.actuatorCurrent = state.actuatorCurrent;
            msg_state.actuatorVel = state.actuatorVel;
            msg_state.actuatorPos = state.actuatorPos;
            msg_state.jointPos = state.jointPos;
            muscle_pubs.at(name).publish(msg_state);

            // if the physical limit of joint angle is reached, fix current position and don't move the motor
            // works only if the motors were initialized
            // values are currently tuned for the straight upright position for all links
            if (myo_control.initialized)
            {
                if (msg_state.contractileDisplacement >= myo_control.maxContractileDisplacement[name])
                {
                    ROS_WARN_STREAM("Muscle " << name << " has reached its limit. Setting force to minimum (" << std::to_string(myo_control.minForce) << " N).");
                    myo_control.flexray.set(name, ControlMode::Force, myo_control.minForce);
                }
            }
            // TODO get rid of this magic!
            if (name=="forearm")
            {
              angle.data = state.jointPos;
              joint_pubs["upper"].publish(angle);
            }
              else if (name=="hand")
            {
                angle.data = state.jointPos;
                joint_pubs["lower"].publish(angle);
            }
          },
          [](FlexRayHardwareInterface::ReadError) {});
    }
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

  /*
  * Instantiate the MyoMotor class. The class only really exists so that we do
  * not have to have the FlexRayHardwareInterface instance a global - to use
  * them in the functions outside of main.
  */
  ros::NodeHandle nh;
  std::string bridge_description;
  if (!nh.getParam("/flex_bridge", bridge_description)) {
    ROS_ERROR_STREAM(
        "Please provide an ftdi device in ros parameter /myo_blink/ftdi_id");
    return 1;
  }
  try {
    auto node = YAML::Load(bridge_description);
    ROS_INFO_STREAM("Description parsed");
    node = node["FlexRay"];
    ROS_INFO_STREAM("Fetched yaml data");

    FlexRayBus fbus = node.as<FlexRayBus>();
    while (FlexRayHardwareInterface::connect(std::move(fbus))
               .match(
                   [&](FlexRayHardwareInterface &flex) {
                    ROS_INFO_STREAM("Connected");
                    MyoMotor motor{std::move(flex)};
                    motor.minForce = node["_SoftSpring"]["constant"].as<double>();
                    ROS_INFO_STREAM("Minimum force to apply to the motor: " << motor.minForce);

                    ROS_INFO_STREAM("Press ENTER to initialize, S to skip initialization");
                    if (std::cin.get() == '\n') {
                        initialize(motor);
                    }
                    blink(motor);
                    return false;
                   },
                   [&](std::pair<FlexRayBus, FtResult> &result) {
                     ROS_ERROR_STREAM("Could not connect to the myo motor: "
                                      << result.second.str());
                     fbus = std::move(result.first);
                     return true;
                   }))
      ;
  } catch (YAML::Exception e) {
    ROS_ERROR_STREAM("Error in /flex_bridge["
                     << e.mark.pos << "]:" << e.mark.line << ":"
                     << e.mark.column << ": " << e.msg);
  }
}
