/*!*******************************************************************************
 *  \brief      This is the state interface package for Tello Interface.
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

#include "tello_state_interface.h"

using namespace std;

StateInterface::StateInterface(){
}   

StateInterface::~StateInterface(){
}

void StateInterface::ownSetUp() {
    this->stateSocket = new StateSocket(TELLO_STATE_PORT);
}

void StateInterface::ownStart(){
    //Publisher
    ros::NodeHandle n;

    flight_action_sub = n.subscribe("actuator_command/flight_action", 1, &StateInterface::flightActionCallback, this);

    speed_pub = n.advertise<geometry_msgs::TwistStamped>("sensor_measurement/linear_speed", 1, true);
    imu_pub = n.advertise<sensor_msgs::Imu>("sensor_measurement/imu", 1, true);
    battery_pub = n.advertise<sensor_msgs::BatteryState>("sensor_measurement/battery_state", 1, true);
    temperature_pub = n.advertise<sensor_msgs::Temperature>("sensor_measurement/temperature", 1, true);
    sea_level_pub = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/sea_level_altitude", 1, true);
    altitude_pub = n.advertise<geometry_msgs::PointStamped>("sensor_measurement/altitude", 1, true);
    flight_state_pub = n.advertise<aerostack_msgs::FlightState>("self_localization/flight_state", 1, true);

    flight_action_msg.action = aerostack_msgs::FlightActionCommand::UNKNOWN;
    this->state_thread = new std::thread(&StateInterface::get_state, this);
}

void StateInterface::get_state()
{
    ros::Time current_timestamp = ros::Time::now();
    ros::Time prev_timestamp = ros::Time::now();
    double prev_roll = 0;
    double prev_pitch = 0;
    double prev_yaw = 0;
    while (true)
    {
        std::string state = this->stateSocket->listen_once();


        if (!state.empty())
        {
            std::istringstream iss(state);

            std::string token;
            map<std::string, float> state_map;
            while (std::getline(iss, token, ';'))
            {
                if (!token.empty())
                {
                    std::string key;
                    std::string value_str;
                    std::istringstream iss_token(token);
                    std::getline(iss_token, key, ':');
                    std::getline(iss_token, value_str, ':');

                    state_map.insert(pair<std::string, float>(key, strtof(value_str.c_str(), 0)));
                }
            }

            current_timestamp = ros::Time::now();

            // ROTATION
            float pitch = state_map.find("pitch")->second;
            float roll = state_map.find("roll")->second;
            float yaw = state_map.find("yaw")->second;

            // VELOCITY
            float vgx = state_map.find("vgx")->second; 
            float vgy = state_map.find("vgy")->second;
            float vgz = state_map.find("vgz")->second;

            // degrees to radians
            float roll_rad = roll * M_PI / 180.0f;
            float pitch_rad = -pitch * M_PI / 180.0f;
            float yaw_rad = yaw * M_PI / 180.0f;

            //current_time_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);
            double diffTime = (current_timestamp - prev_timestamp).nsec / 1E9;

            double dRoll = 0;
            double dPitch = 0;
            double dYaw = 0;
            if (diffTime != 0)
            {
                dRoll = roll_rad / diffTime;
                dPitch = pitch_rad / diffTime;
                dYaw = yaw_rad / diffTime;
            }

            dRoll = roll_rad - prev_roll;
            dPitch = pitch_rad - prev_pitch;
            dYaw = yaw_rad - prev_yaw;

            prev_roll = roll_rad;
            prev_pitch = pitch_rad;
            prev_yaw = yaw_rad;

            float x = (1 * dRoll) - (sin(pitch_rad) * dYaw);
            float y = (cos(roll_rad) * dPitch) + (cos(pitch_rad)*sin(roll_rad) * dYaw);
            float z = (-sin(roll_rad) * dPitch) + (cos(roll_rad)*cos(pitch_rad) * dYaw);
            
            speed_msg.header.stamp = current_timestamp;
            speed_msg.twist.linear.x = vgx / 10;
            speed_msg.twist.linear.y = -vgy / 10;
            speed_msg.twist.linear.z = -vgz / 10;
            speed_msg.twist.angular.x = x;
            speed_msg.twist.angular.y = y;
            speed_msg.twist.angular.z = z;
            speed_pub.publish(speed_msg);


            // ACCELERATION
            float agx = state_map.find("agx")->second;
            float agy = state_map.find("agy")->second;
            float agz = state_map.find("agz")->second;

            // BATTERY
            float battery = state_map.find("bat")->second;
            battery_msg.header.stamp = current_timestamp;
            battery_msg.voltage = 3.8;
            battery_msg.capacity = 1.1;
            //battery percentage goes from 0 to 1
            battery_msg.percentage = battery / 100;
            battery_pub.publish(battery_msg);

            // IMU
            imu_msg.header.stamp = current_timestamp;
            imu_msg.angular_velocity.x = x;
            imu_msg.angular_velocity.y = y;
            imu_msg.angular_velocity.z = z;
            // to m^2/s
            imu_msg.linear_acceleration.x = agx / 100;
            imu_msg.linear_acceleration.y = agy / -100;
            imu_msg.linear_acceleration.z = agz / -100;

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(roll_rad, pitch_rad, yaw_rad);

            imu_msg.orientation.x = quaternion.getX();
            imu_msg.orientation.y = quaternion.getY();
            imu_msg.orientation.z = quaternion.getZ();
            imu_msg.orientation.w = quaternion.getW();

            imu_pub.publish(imu_msg);

            // TEMPERATURE
            float templ = state_map.find("templ")->second;
            float temph = state_map.find("temph")->second;
            float mean = (templ + temph) / 2;
            float variance = pow(temph - mean, 2.0);
            temperature_msg.header.stamp = current_timestamp;
            temperature_msg.temperature = mean;
            temperature_msg.variance = variance;
            temperature_pub.publish(temperature_msg);

            // SEA LEVEL
            float sea_level = state_map.find("baro")->second;
            sea_level_msg.header.stamp = current_timestamp;
            sea_level_msg.point.z = sea_level;
            sea_level_pub.publish(sea_level_msg);

            // ALTITUDE
            double altitude = state_map.find("h")->second;
            altitude_msg.header.stamp = current_timestamp;
            // to meters
            altitude_msg.point.z = -altitude / 100;
            altitude_pub.publish(altitude_msg);

            prev_timestamp = current_timestamp;

            sendFlightStatus(speed_msg,altitude_msg);
        }
    }
}

// Send flight state based on flight action and sensor measurements
void StateInterface::sendFlightStatus(geometry_msgs::TwistStamped sensor_speed_msg, geometry_msgs::PointStamped sensor_altitude_msg){
    switch(flight_action_msg.action){
        case aerostack_msgs::FlightActionCommand::TAKE_OFF:
            if (flight_state_msg.state == aerostack_msgs::FlightState::LANDED || flight_state_msg.state == aerostack_msgs::FlightState::UNKNOWN){
                flight_state_msg.state = aerostack_msgs::FlightState::TAKING_OFF;
                time_status = ros::Time::now();
            }else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::TAKING_OFF){
                    ros::Duration diff = ros::Time::now() - time_status;
                    if (std::abs(sensor_speed_msg.twist.linear.z) < 0.05 && std::abs(sensor_altitude_msg.point.z) > 0.4 && diff.toSec() >= 5){
                        flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
                    }
                }
            }
        break;
        case aerostack_msgs::FlightActionCommand::HOVER:
        {
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
            std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::LAND:
        {
            if (flight_state_msg.state == aerostack_msgs::FlightState::HOVERING || flight_state_msg.state == aerostack_msgs::FlightState::FLYING){
                if (sensor_speed_msg.twist.linear.z < 0){
                    flight_state_msg.state = aerostack_msgs::FlightState::LANDING;
                }
            }else{
                if (flight_state_msg.state == aerostack_msgs::FlightState::LANDING){
                    if (std::abs(sensor_altitude_msg.point.z) < 0.1 && std::abs(sensor_speed_msg.twist.linear.z < 0.05)){
                        flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
                    }
                }
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::MOVE:
        {
            //std::cout << "Imprimo datos: " << sensor_speed_msg << std::endl;
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && (std::abs(sensor_speed_msg.twist.linear.x) > 0.05 || std::abs(sensor_speed_msg.twist.linear.y) > 0.05 || std::abs(sensor_speed_msg.twist.linear.z) > 0.05 ||
            std::abs(sensor_speed_msg.twist.angular.x) > 0.05 || std::abs(sensor_speed_msg.twist.angular.y) > 0.05 || std::abs(sensor_speed_msg.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
            std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
        case aerostack_msgs::FlightActionCommand::UNKNOWN:
        default:
        {
            if(std::abs(sensor_altitude_msg.point.z) < 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
            std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::LANDED;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && (std::abs(sensor_speed_msg.twist.linear.x) > 0.05 || std::abs(sensor_speed_msg.twist.linear.y) > 0.05 || std::abs(sensor_speed_msg.twist.linear.z) > 0.05 ||
            std::abs(sensor_speed_msg.twist.angular.x) > 0.05 || std::abs(sensor_speed_msg.twist.angular.y) > 0.05 || std::abs(sensor_speed_msg.twist.angular.z) > 0.05)){
                flight_state_msg.state = aerostack_msgs::FlightState::FLYING;
            }
            if(std::abs(sensor_altitude_msg.point.z) > 0.1 && std::abs(sensor_speed_msg.twist.linear.x) < 0.05 && std::abs(sensor_speed_msg.twist.linear.y) < 0.05 && std::abs(sensor_speed_msg.twist.linear.z) < 0.05 &&
            std::abs(sensor_speed_msg.twist.angular.x) < 0.05 && std::abs(sensor_speed_msg.twist.angular.y) < 0.05 && std::abs(sensor_speed_msg.twist.angular.z) < 0.05){
                flight_state_msg.state = aerostack_msgs::FlightState::HOVERING;
            }
        }
        break;
    }
    flight_state_pub.publish(flight_state_msg);
}

void StateInterface::flightActionCallback(const aerostack_msgs::FlightActionCommand::ConstPtr& msg)
{
    flight_action_msg = *msg;
}

//Stop
void StateInterface::ownStop()
{
    flight_state_pub.shutdown();
    flight_action_sub.shutdown();
    speed_pub.shutdown();
    battery_pub.shutdown();
    temperature_pub.shutdown();
    imu_pub.shutdown();
    sea_level_pub.shutdown();
    altitude_pub.shutdown();
}

//Reset
bool StateInterface::resetValues()
{
    return true;
}

//Run
void StateInterface::ownRun()
{

}

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "StateInterface");

    cout<<"[ROSNODE] Starting StateInterface"<<endl;

    //Vars
    StateInterface state_interface;
    state_interface.setUp();
    state_interface.start();
    try
    {
        //Read messages
        ros::spin();
        return 1;
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}