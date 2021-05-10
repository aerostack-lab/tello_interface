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

#include "tello_command_interface.h"

using namespace std;

CommandInterface::CommandInterface(){}

CommandInterface::~CommandInterface(){}

void CommandInterface::ownSetUp()
{
    this->commandSocket = new CommandSocket(TELLO_CLIENT_ADDRESS, TELLO_COMMAND_PORT, PC_COMMAND_PORT);
    cout<<"[ROSNODE] Command Interface"<<endl;
}

void CommandInterface::ownStart()
{
    this->commandSocket->send_command("command");
    usleep(200);
    this->commandSocket->send_command("streamon");

    ros::NodeHandle n;
    //Subscribers    
    flight_action_sub = n.subscribe("actuator_command/flight_action", 1, &CommandInterface::flightActionCallback, this);
    actuator_command_thrust_sub = n.subscribe("actuator_command/roll_pitch_yaw_rate_thrust", 1, &CommandInterface::actuatorThrustCallback, this);
    reference_pose_sub = n.subscribe("motion_reference/pose", 1, &CommandInterface::referencePoseCallback, this);
    reference_speed_sub = n.subscribe("motion_reference/speed", 1, &CommandInterface::referenceSpeedCallback, this);
    keep_hover = true;
    command_alive = "rc 0 0 0 0"; //Force to send this command so the drone doesn't shut itself off
}

//Stop
void CommandInterface::ownStop()
{
    flight_action_sub.shutdown();
    actuator_command_thrust_sub.shutdown();
}

//Reset
bool CommandInterface::resetValues()
{
    return true;
}

//Run
void CommandInterface::ownRun()
{
    ros::Duration diff = ros::Time::now() - last_time_command;
    //std::cout << "Segundos: " << diff.toSec() << std::endl;
    if (diff.toSec() > 12 && keep_hover){
        sendCommandToTello(command_alive);
    }
}

//Get motion_reference/pose msg
void CommandInterface::referencePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::ostringstream command;

    // Move_vertical behavior
    if(msg->pose.position.z != 0){
        if(msg->pose.position.z > 0){
            command << "up " << static_cast<int>(round(std::abs(msg->pose.position.z*100)));
        }else{
            command << "down " << static_cast<int>(round(std::abs(msg->pose.position.z*100)));
        }
        sendCommandToTello(command.str().c_str());  
    }else
    { // Rotate
        tf2::Matrix3x3 pose_ref_m(tf2::Quaternion (msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w));
        double r = 0; double p = 0; double yaw = 0;
        pose_ref_m.getRPY(r, p, yaw);
        if (std::isnan(yaw)) yaw = 0.0; 

        yaw = round((180*yaw)/M_PI); //To degrees
        if (yaw > 0){
            command << "cw " << static_cast<int>(std::abs(yaw));
        }else{
            command << "ccw " << static_cast<int>(std::abs(yaw));
        }
        sendCommandToTello(command.str().c_str());    
    }

    //sendCommandToTello(command.str().c_str());
}

//Get motion_reference/speed
void CommandInterface::referenceSpeedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    std::ostringstream command;
    
    int dir_x,dir_y,dir_z;

    dir_x = round (msg->twist.linear.x*100);
    dir_y = round (msg->twist.linear.y*100);
    dir_z = round (msg->twist.linear.z*100);

    if(dir_x > 100){
        dir_x = 100;
    }
    if(dir_x < -100){
        dir_x = -100;
    }
    if(dir_y > 100){
        dir_y = 100;
    }
    if(dir_y < -100){
        dir_y = -100;
    }
    if(dir_z > 100){
        dir_z = 100;
    }
    if(dir_z < -100){
        dir_z = -100;
    }

    command << "rc " << static_cast<int>(dir_x)
    << " " << static_cast<int>(dir_y)
    << " " << static_cast<int>(dir_z)
    << " " << static_cast<int>(0);

    sendCommandToTello(command.str().c_str());
}

//Send commandToTello
void CommandInterface::sendCommandToTello(std::string msg){
    last_time_command = ros::Time::now();
    keep_hover = false;
    // Making sure the program doesn't interrupt a recurrent task
    if (msg.find("up") != string::npos){
      keep_hover = true;
      command_alive = "rc 0 0 0 0";
    }
    if (msg.find("down") != string::npos){
      keep_hover = true;
      command_alive = "rc 0 0 0 0" ;
    }
    if (msg == "rc 0 0 0 0"){
      keep_hover = true;
      command_alive = "rc 0 0 0 0" ;
    }else{
        if (msg.find("rc") != string::npos){
            command_alive = msg;
            keep_hover = true;
        }
    }
    
    std::string response = this->commandSocket->send_command(msg);
    cout << "Command: " << msg << endl;
    cout << "Response: " << response <<endl;   
    if(response.find("error") != string::npos){
        cout << "Trying again... " << msg << endl;
        response = this->commandSocket->send_command(msg);
        cout << "Command: " << msg << endl;
        cout << "Response: " << response <<endl;
    }
}

void CommandInterface::actuatorThrustCallback(const mav_msgs::RollPitchYawrateThrust::ConstPtr& msg){
    //TODO: topic actuator_command/roll_pitch_yaw_rate_thrust implementation


    ////Normalize (-100,100) roll and pitch (max value = 0.4)
    int normalized_roll = round((msg->roll/0.4)*50);
    int normalized_pitch = round((msg->pitch/0.4)*50);
    ////Normalize (-100,100) yaw_rate (max value = 1)
    int normalized_yaw = round((msg->yaw_rate/1)*50);
    ////Normalize (-100,100) altitude (max value = 1.40)
    int normalized_altitude = round((0/1.40)*50);


    std::ostringstream rc; // rc a b c d
    rc << "rc " << static_cast<int>(normalized_roll)
    << " " << static_cast<int>(normalized_pitch)
    << " " << static_cast<int>(0)
    << " " << static_cast<int>(0);

    //sendCommandToTello(rc.str().c_str());
}

void CommandInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg)
{
    flight_action_msg = *msg;
    switch(msg->action){
    case aerostack_msgs::FlightActionCommand::TAKE_OFF:
        sendCommandToTello("takeoff");
        break;
    case aerostack_msgs::FlightActionCommand::LAND:
        sendCommandToTello("land");
        break;
    case aerostack_msgs::FlightActionCommand::HOVER:
        sendCommandToTello("rc 0 0 0 0");
        break;
    default:
        break;
    }
    return;
}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "CommandInterface");

    cout<<"[ROSNODE] Starting CommandInterface"<<endl;

    //Vars
    CommandInterface command_interface;
    command_interface.setUp();
    command_interface.start();
    ros::Rate loop_rate(100); //100 Hz

    try
    {
        while(ros::ok())
        {  
            ros::spinOnce();
            command_interface.run();
            loop_rate.sleep();
        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}