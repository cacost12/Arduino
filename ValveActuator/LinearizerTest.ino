#include<math.h>


float linearPWM = 0.0;
int PWM = 0;
const int potPin = A0;
int potVal = 0;
int sign;


int LinearPWM(float input){
  // Check for Sign
  if (input>=0){sign = 1;}
  else{sign = -1;}

  //Create Dummy Variable
  float val = abs(input)/100;

  // Allocate Memory for output
  float out = 0;

  // Check Range of Input and Convert
  if (0<=val<0.3399){out = 3*val/0.3446;}
  else if (0.3399<=val<0.6614){out = (val-0.33)/0.0033;}
  else{out = 100 - 20*log((0.71-val)/0.0486);}

 // Round the output
 int result = floor(out) + 80;

 // Add Sign back to value
 result *= sign;

 return(result);
 
  }


void setup() {
  // put your setup code here, to run once:
  pinMode(potPin,INPUT);
  Serial.begin(9600);

}

void loop() {
  potVal = analogRead(potPin);
  linearPWM = 0.138*potVal - 70.88;

  PWM = LinearPWM(linearPWM);
  Serial.print(potVal);
  Serial.print(" ");
  Serial.print(linearPWM);
  Serial.print(" ");
  Serial.println(PWM);
  
delay(100);
}
