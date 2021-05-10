/*!*******************************************************************************************
 * \brief     Keyboard teleoperation test for tello.
 * \authors   Alberto Rodelgo
 * \copyright Copyright (c) 2020 Universidad Politecnica de Madrid
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
 *******************************************************************************/

#include <string>
#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <boost/thread/thread.hpp>
#include <curses.h>
#include <thread>
#include <locale.h>

#include "socket_tello.h"

//Inputs
#define ASCII_KEY_UP 65
#define ASCII_KEY_DOWN 66
#define ASCII_KEY_RIGHT 67
#define ASCII_KEY_LEFT 68

int main(int argc, char** argv){
  ros::init(argc, argv, "KEYBOARD TELEOPERATION TEST FOR TELLO");
  ros::NodeHandle n("~");

  CommandSocket* commandSocket = new CommandSocket(TELLO_CLIENT_ADDRESS, TELLO_COMMAND_PORT, PC_COMMAND_PORT);

  // ncurses initialization
  setlocale(LC_ALL, "");
  std::setlocale(LC_NUMERIC, "C");
  initscr();
  start_color();
  use_default_colors();  
  curs_set(0);
  noecho();
  nodelay(stdscr, TRUE);
  erase();
  refresh();
  init_pair(1, COLOR_BLUE, -1);
  init_pair(2, COLOR_GREEN, -1);
  init_pair(3, COLOR_CYAN, -1);
  init_pair(4, COLOR_RED, -1);
  init_pair(5, COLOR_YELLOW, -1);

  //Input variable
  char command = 0;
  sleep(3);

  std_msgs::String order;

  move(0,0);clrtoeol();
  printw("                   - KEYBOARD TELEOPERATION TEST FOR TELLO -");
  move(3,0);clrtoeol();
  printw("--------------------------------------------------------------------------------");
  //Print controls
  printoutAttitudeControls();
  //LOOP
  ros::Rate loop_rate(50);

  while (ros::ok()){
    // Read messages
    ros::spinOnce();
    move(16,0);
    printw("                        Last key pressed: ");
    //Read command
    command = getch();
    switch (command){
      case 't':  // Take off
        commandSocket->send_command("takeoff");
        printw("t       ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Take off           ");clrtoeol();
        break;
      case 'y':  // Land
        commandSocket->send_command("land");
        printw("y        ");clrtoeol(); 
        move(17, 0); 
        printw("                        Last command:     Land             ");clrtoeol();            
        break;
      case 'h':  // Hover   
      {
        commandSocket->send_command("rc 0 0 0 0");
        printw("h      ");clrtoeol();
        move(17, 0); printw("                        Last command:     Hover             ");clrtoeol();refresh();
        break;
      }
      case 'q':  // Move upwards
        commandSocket->send_command("up 100");
        printw("q      ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Increase altitude         ");clrtoeol();
        break;
      case 'a':  //Move downwards
        commandSocket->send_command("down 100");
        printw("a        ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Decrease altitude         ");clrtoeol(); 
        break;         
      case 'z':  //(yaw) turn counter-clockwise
      {        
        commandSocket->send_command("ccw 90");
        printw("                        Last command:     Turn counter-clockwise       ");clrtoeol();  
        break;    
      }        
      case 'x':  // (yaw) turn clockwise
      {      
        commandSocket->send_command("cw 90");
        printw("                        Last command:     Turn clockwise          ");clrtoeol();
      }
        break;                 
      case ASCII_KEY_RIGHT:
        commandSocket->send_command("rc 50 0 0 0");
        printw("\u2192            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Move to the right        ");clrtoeol();  
        break;               
      case ASCII_KEY_LEFT:
        commandSocket->send_command("rc -50 0 0 0");
        printw("\u2190            ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Move to the left         ");clrtoeol();
        break;       
      case ASCII_KEY_DOWN:     
        commandSocket->send_command("rc 0 50 0 0");
        printw("\u2193     ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Move backward      ");clrtoeol();
        break;        
      case ASCII_KEY_UP:     
        commandSocket->send_command("rc 0 -50 0 0");
        printw("\u2191    ");clrtoeol();
        move(17, 0); 
        printw("                        Last command:     Move forward            ");clrtoeol();
        break;                    
    }
    refresh();
    loop_rate.sleep();
  }

  endwin();
  return 0;
}