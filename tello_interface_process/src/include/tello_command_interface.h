/*!*******************************************************************************
 *  \brief      This is the command interface package for Tello Interface.
 *  \authors    Alberto Rodelgo Perales
 *  \copyright  Copyright (c) 2020 Universidad Politecnica de Madrid
 *              All rights reserved
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/ 

#include <iostream>

//// ROS  ///////
#include "ros/ros.h"
#include "cvg_string_conversions.h"
#include <robot_process.h>
#include "math.h"

#include "socket_tello.h"

#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "aerostack_msgs/FlightActionCommand.h"
#include "mav_msgs/RollPitchYawrateThrust.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <std_msgs/String.h>

class TelloSocketClient;

class CommandInterface : public RobotProcess
{
//Constructors and destructors
public:
    CommandInterface();
    ~CommandInterface();

protected:
    bool resetValues();    
private: /*RobotProcess*/
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();

    CommandSocket* commandSocket;
    std_msgs::String command_msg;
    std::string command_alive;
    ros::Time last_time_command;

    bool keep_hover;

    //Subscribers
    ros::Subscriber flight_action_sub;
    aerostack_msgs::FlightActionCommand flight_action_msg;
    void flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg);
    ros::Subscriber actuator_command_thrust_sub;
    void actuatorThrustCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg);
    ros::Subscriber reference_pose_sub;
    void referencePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    ros::Subscriber reference_speed_sub;
    void referenceSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    void sendCommandToTello(std::string msg);
};
