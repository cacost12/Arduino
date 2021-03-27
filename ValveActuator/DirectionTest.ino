volatile long EncoderCount = 0;        // variable to keep track of number of pulses sent by the encoder
int ChannelA;                          // flag variables for channel A and B
int ChannelB;


const int motorDirectionPin = 13; // for motor A
const int motorSpeedPin = 11; // for motor A
const int motorBrakePin = 9; // for motor A

const int CW  = HIGH;
const int CCW = LOW;

float shaftAngle;

int motorSpeed = 255;

void setup() {
  // motor A pin assignment
  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
  pinMode(motorBrakePin, OUTPUT);

 
  pinMode(2, INPUT);                   // define encoder pins as inputs
  pinMode(3, INPUT);
  attachInterrupt(digitalPinToInterrupt(2), checkChannelA, CHANGE); // start interrupts on the encoder pins. 
  attachInterrupt(digitalPinToInterrupt(3), checkChannelB, CHANGE); 
 
  
  Serial.begin(9600);//  seial monitor initialized 

 

}

void loop() {

  shaftAngle = -EncoderCount*360/600;

  
  digitalWrite(motorDirectionPin, CW);// set direction
  analogWrite(motorSpeedPin, motorSpeed);// set speed at maximum
  Serial.println(shaftAngle);
  delay(10);
 
}// loop end

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
