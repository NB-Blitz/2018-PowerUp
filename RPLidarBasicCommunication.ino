/*
   RoboPeak RPLIDAR Arduino Example
   This example shows the easy and common way to fetch data from an RPLIDAR

   You may freely add your application code based on this template

   USAGE:
   ---------------------------------
   1. Download this sketch code to your Arduino board
   2. Connect the RPLIDAR's serial port (RX/TX/GND) to your Arduino board (Pin 0 and Pin1)
   3. Connect the RPLIDAR's motor ctrl pin to the Arduino board pin 3
********************************************************************************************************
   Copyright (c) 2014, RoboPeak
   All rights reserved.
   RoboPeak.com

   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
   EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
   SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
   OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
   HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
   TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//Third Party Includes
#include <RPLidar.h>
#include <Wire.h>


//Inclusions Made By Me
#include "LidarUtility.hpp"


//Object Declarations
RPLidar lidar;         //You need to create an driver instance


#define ARDUINO_I2C_ADDRESS 0x11  //The Arduino's I2C slave Address
#define ROBORIO_REQUEST_BYTE 0x10 //The RoboRio's data request Byte

#define LIDAR_MOTOR_CONTROL_BYTE 0x20 //Allows the RoboRIO to control if the Motor is on or off
#define LIDAR_MOTOR_ON  0x21 //Turns the Lidar Motor on
#define LIDAR_MOTOR_OFF 0x22 //Turns the lidar motor off

#define MAX_SENT_BYTES 3          //Maximum amount of command bytes the Rio can send
#define MAX_SEND_ARRAY_SIZE 1    //Largest Amount of Data the arduino can send across to the RIO

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor. This pin should connected with the RPLIDAR's MOTOCTRL signal



uint8_t RIO_DATA_BUFFER[MAX_SENT_BYTES];

byte receivedCommands[MAX_SENT_BYTES];

bool lidarMotorOn = false;

int DistanceArray[360];
const int MINIMUM_DISTANCE = 24; //Minimum Distance for the robot

int previousAngle = 0;
int currentAngle = 0;

int CurrentDistanceValue[4];

int expirationCounter = 0;
int expirationTime = 4;

void setup()
{
  lidar.begin(Serial); // bind the RPLIDAR driver to the arduino hardware serial
  Serial.begin(115200);

  Wire.begin(ARDUINO_I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  for (int i = 0; i < 4; i++)
  {
    CurrentDistanceValue[i] = 255;
  }

  pinMode(RPLIDAR_MOTOR, OUTPUT);  //set pin modes
}

void loop()
{
  previousAngle = currentAngle;
  if (IS_OK(lidar.waitPoint()))
  {
    int distance = (lidar.getCurrentPoint().distance / 25.4);  //distance value in inch unit
    currentAngle = lidar.getCurrentPoint().angle;              //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit;

    if (startBit)
    {
      Serial.print("ExpirationCounter: ");
      Serial.println(expirationCounter);
      
      if (expirationCounter < expirationTime)
      {
        expirationCounter++;
      }
      else if (expirationCounter >= expirationTime)
      {
        expirationCounter = 0;
        for (int i = 0; i < 4; i++)
        {
          Serial.print(CurrentDistanceValue[i]);
          Serial.print(", ");
          CurrentDistanceValue[i] = 255;
        }
        Serial.println();
      }
    }
    if (currentAngle > 0 && currentAngle < 45)
    {
      //Serial.println("Quad 1");
      if (distance < CurrentDistanceValue[0] && distance != 0)
      {
        CurrentDistanceValue[0] = distance;
      }
    }
    else if (currentAngle > 45 && currentAngle < 90)
    {
      //Serial.println("Quad 2");
      if (distance < CurrentDistanceValue[1] && distance != 0)
      {
        CurrentDistanceValue[1] = distance;
      }
    }
    else if (currentAngle > 90 && currentAngle < 135)
    {
      //Serial.println("Quad 3");
      if (distance < CurrentDistanceValue[2] && distance != 0)
      {
        CurrentDistanceValue[2] = distance;
      }
    }
    else if (currentAngle > 135 && currentAngle < 180)
    {
      //Serial.println("Quad 4");
      if (distance < CurrentDistanceValue[3] && distance != 0)
      {
        CurrentDistanceValue[3] = distance;
      }
    }
  }
  else
  {
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) //try to detect RPLIDAR...
    {
      // detected...
      lidar.startScan();
      delay(1000);
    }
  }
  if (lidarMotorOn)
  {
    analogWrite(RPLIDAR_MOTOR, 100); //stop the rplidar motor
  }
  if (!lidarMotorOn)
  {
    analogWrite(RPLIDAR_MOTOR, 0); //stop the rplidar motor
  }
}


void requestEvent()
{
  uint8_t tempArray[4];
  for (int i = 0; i < 4; i++)
  {
    tempArray[i] = (uint8_t)CurrentDistanceValue[i];
  }
  Wire.write(tempArray, 4);
}


void receiveEvent(int bytesReceived)
{
  for (int a = 0; a < bytesReceived; a++)
  {
    if ( a < MAX_SENT_BYTES)
    {
      receivedCommands[a] = Wire.read();
    }
    else
    {
      Wire.read();  // if we receive more data then allowed just throw it away
    }
  }

  if (receivedCommands[0] == LIDAR_MOTOR_CONTROL_BYTE)
  {
    if (receivedCommands[1] == LIDAR_MOTOR_ON)
    {
      lidarMotorOn = true;
    }
    else if (receivedCommands[1] == LIDAR_MOTOR_OFF)
    {
      lidarMotorOn = false;
    }
  }
}
