#include<math.h>

// ####### Define Variables ####### //

// Angular Encoder
volatile long EncoderCount = 0;        // variable to keep track of number of pulses sent by the encoder
int ChannelA;                          // flag variables for channel A and B
int ChannelB;
float ShaftAngle, PreviousShaftAngle;  // Angular Position of Shaft
float ShaftAngleDeg;
float ShaftSpeed;                      // Shaft Speed 

// Time Variables
unsigned long Time, PreviousTime;     
int StartTime; 


// Motor Variables
const int motorDirectionPin = 13; // Establish direction of motor
const int motorSpeedPin = 11; // Pin For Setting motor speed
const int motorBrakePin = 9; // Pin for Motor brake
int motorSpeed;  // Variable for setting the PWM duty cycle

// Valve/Shaft Variables
const int CLOSE = HIGH; // Logical signal to close valve, positive controller output
const int OPEN = LOW; // Logical signal to open valve, negative controller output
float desiredAngle; // Reference Angular Command
float desiredThrottle; // Throttle Reference Command
float throttle; // Proportional Variable: Keeps track of flow throttle

// Controller Variables
float error, previousError; // Error
float integral = 0.0; // Integral Sum
const float kp = 100.0; // Proportional Constant
const float kI = 30.0; // Integral Constant
float out = 0; // Controller Output

// Create a Function to convert Controller output to PWM signal
int LinearPWM(float input){
  // Check for Sign
  int sign;
  if (input>=0){sign = 1;}
  else{sign = -1;}

  //Create Dummy Variable
  float val = abs(input)/100;

  // Allocate Memory for output
  float y = 0;

  // Check Range of Input and Convert
  if (0<=val<0.3399){y = 3*val/0.3446;}
  else if (0.3399<=val<0.6614){y = (val-0.33)/0.0033;}
  else{y = 100 - 20*log((0.71-val)/0.0486);}

 // Round the output
 int result = floor(y) + 80;

 // Add Sign back to value
 result *= sign;

 return(result);
 
  }



// Setup Code
void setup() {

  // Start Serial Monitor
  Serial.begin(38400);

  // Initialize Motor Pins as Outputs
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
  pinMode(motorBrakePin, OUTPUT);

  // Initialize Encoder Pins as Interupt Inputs
  pinMode(2, INPUT);                   
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), checkChannelA, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(3), checkChannelB, CHANGE); 

  // Pause Code to give time for setup
  delay(3000);
 
  // Setup Variables
  PreviousShaftAngle = -EncoderCount*2*M_PI/600.0;
  ShaftAngleDeg = -EncoderCount*360.0/600.0;
  previousError = 0;
  PreviousTime =0;
  delay(30);

  // Record Time at which main loop began
  StartTime = millis();
}

void loop() {

  // Reference Command: Throttle Percentage
  desiredThrottle = 50.0;

  // Convert to reference angular position of shaft
  desiredAngle = desiredThrottle*22.797490/100.0;

  // Record Shaft Angle
  ShaftAngle = -EncoderCount*2*M_PI/600.0;
  ShaftAngleDeg = -EncoderCount*360.0/600.0;

  // Compute Throttle
  throttle = ShaftAngle*100.0/22.79749;
  
  // Record Time
  Time = millis() - StartTime;

  // Compute Error
  error = desiredAngle - ShaftAngle;

  // Compute Error Integral
  if(abs(out)<70.88){
  integral += 0.5*((Time-PreviousTime)/1000.0)*(error + previousError);
  }
  
  // Compute Controller Output
  out = kp*error + kI*integral;

  // Saturate Controller
  if(abs(out)>70.88){
    out = 70.88;
    motorSpeed = 255;
    }else{ // Compute MotorSpeed from controller Output
      
      motorSpeed = LinearPWM(out);
      }
  

  // Set Motor Speed
  if(motorSpeed>=0){
    digitalWrite(motorDirectionPin, CLOSE);// set direction
    analogWrite(motorSpeedPin, motorSpeed);// set speed 
    }
    else if(motorSpeed<0){
      
    digitalWrite(motorDirectionPin, OPEN);// set direction
    analogWrite(motorSpeedPin, -motorSpeed);// set speed 
      }
      else{
        Serial.println("Error");
        }
    
  // Print Data to Serial Monitor
  Serial.print(Time);
  Serial.print(" ");
  //Serial.print(ShaftAngle);
  //Serial.print(" ");
  Serial.print(throttle);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
  //Serial.print(kp*error);
  //Serial.print(" ");
  Serial.print(out);
  Serial.print(" ");
  Serial.print(integral);
  Serial.print(" ");
  Serial.println(motorSpeed);
  
  // Update the Previous variables
  PreviousShaftAngle = ShaftAngle;
  PreviousTime = Time;    
  previousError = error;
   
  delay(1);
}

/*************** Functions to count encoder ticks*****************/
void checkChannelA() {

  // Low to High transition?
  if (digitalRead(2) == HIGH) {
    ChannelA = 1;
    if (!ChannelB) {
      EncoderCount = EncoderCount + 1;

    }
  }

  // High-to-low transition?
  if (digitalRead(2) == LOW) {
    ChannelA = 0;
  }

}


// Interrupt on B changing state
void checkChannelB() {

  // Low-to-high transition?
  if (digitalRead(3) == HIGH) {
    ChannelB = 1;
    if (!ChannelA) {
      EncoderCount = EncoderCount - 1;
    }
  }

  // High-to-low transition?
  if (digitalRead(3) == LOW) {
    ChannelB = 0;
  }
}
