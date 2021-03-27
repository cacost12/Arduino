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

const int Close  = HIGH; // Variables for assigning rotational directions logical values
const int Open = LOW;

int motorSpeed;  // Variable for setting the PWM duty cycle

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
  PreviousTime = millis();
  delay(30);

  // Record Time at which main loop began
  StartTime = millis();
}

void loop() {

  // Set PWM duty cycle
  motorSpeed = 80;

  // Start Motor
  digitalWrite(motorDirectionPin, Close);// set direction
  analogWrite(motorSpeedPin, motorSpeed);// set speed at maximum

  // Record Shaft Angle
  ShaftAngle = -EncoderCount*2*M_PI/600.0;
  ShaftAngleDeg = -EncoderCount*360.0/600.0;

  // Record Time
  Time = millis() - StartTime;

  // Calculate the Shaft Speed
  ShaftSpeed = (ShaftAngle - PreviousShaftAngle)/((Time-PreviousTime)/1000.0);

  // Print Data to Serial Monitor
  Serial.print(Time);
  Serial.print(" ");
  Serial.print(ShaftAngleDeg);
  Serial.print(" ");
  Serial.println(ShaftSpeed);

  // Update the Previous variables
  PreviousShaftAngle = ShaftAngle;
  PreviousTime = Time;
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
