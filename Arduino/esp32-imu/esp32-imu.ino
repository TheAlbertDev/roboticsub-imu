/******************************************************************************
roboticsub-imu.ino - MPU Arduino library example
Copyright (C) 2021  Albert Álvarez-Carulla
Repository: https://github.com/TheAlbertDev/roboticsub-imu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
******************************************************************************/

#include "src/RoboticsUB.h"

const int PIN_IMU_INT = 18;
const int PIN_S1 = 14;
const int PIN_S2 = 27;

enum class Command : byte
{
  GET_RPW = 1
};

Command command = Command::GET_RPW;

IMU imu;

float *rpw;
float *q;
int s1Status = HIGH;
int s2Status = HIGH;

void setup()
{

  Serial.begin(115200);

  pinMode(PIN_IMU_INT, INPUT_PULLUP);
  pinMode(PIN_S1, INPUT);
  pinMode(PIN_S2, INPUT);

  imu.Install();
}

void loop()
{

  if (digitalRead(PIN_IMU_INT) == HIGH)
  {
    imu.ReadSensor();
    rpw = imu.GetRPW();
    q = imu.GetQuaternion();
  }

  s1Status = digitalRead(PIN_S1);
  s2Status = digitalRead(PIN_S2);

  if (Serial.available() > 0)
  {
    command = (Command)Serial.read();

    switch (command)
    {
    case Command::GET_RPW:
      Serial.println(rpw[0], DEC);
      Serial.println(rpw[1], DEC);
      Serial.println(rpw[2], DEC);
      Serial.println(s1Status, DEC);
      Serial.println(s2Status, DEC);
      break;
    }
  }
}
