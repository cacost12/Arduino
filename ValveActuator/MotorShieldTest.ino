/*
 * Code to control 2 DC motors using Arduino Motor Shield (Basic Code)
 * Written by Ahmad Shamshiri for Robojax.com on Aug 28, 2018 at 21:33 in Ajax, Ontario, Canada
 * Watch video instruction for this code:https://youtu.be/kIgbjyqNrV8
 * Watch how to use current sensing with Motor Shield : https://youtu.be/-uQKBDTWHPM
 * 
 * This code has been download from Robojax.com
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
 * 
 */
const int MotorPinA = 12; // for motor A
const int MotorSpeedPinA = 11; // for motor A
const int MotorBrakePinA = 9; // for motor A

const int CW  = HIGH;
const int CCW = LOW;

void setup() {
  // motor A pin assignment
  pinMode(MotorPinA, OUTPUT);
  pinMode(MotorSpeedPinA, OUTPUT);
  pinMode(MotorBrakePinA, OUTPUT);

  pinMode(3,OUTPUT);

  Serial.begin(9600);//  seial monitor initialized 

}

void loop() {

  int motorSpeed = 255;


  digitalWrite(3,HIGH);
  //start motor A at maximum speed
  digitalWrite(MotorPinA, CW);// set direction
      Serial.println("Direction CW"); 
  analogWrite(MotorSpeedPinA, motorSpeed);// set speed at maximum
      Serial.println("Speed 200");
  delay(5000);// run for 5 seconds
  analogWrite(MotorSpeedPinA, 0);
  delay(1000);
  digitalWrite(MotorPinA,CCW);
 analogWrite(MotorSpeedPinA, motorSpeed);
      Serial.println("Speed 0");

  digitalWrite(3,LOW);
  delay(5000);// 





}// loop end
